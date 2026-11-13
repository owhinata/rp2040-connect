#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""KV（設定と校正値）を Zephyr 版ファームウェアから退避し、NuttX 版へ書き戻す。

Zephyr 版と NuttX 版は flash 上の KV の保存形式が違い、NuttX 版を書き込むと Zephyr 版が
保存した値は読めなくなる。そこで、Zephyr 版が動いている間にシェルの `kv get` で値を読んで
PC のファイル（JSON）に保存し、NuttX 版を書き込んだあとで `kv sethex` で書き戻す。

手順:
  1. Zephyr 版が動いている状態で backup を実行する。
  2. NuttX 版を書き込む。
  3. restore を実行する。KV を書き戻し、Wi-Fi の資格情報を `wifi cred add` で登録する。

Wi-Fi のパスワードはボードから読み出さない（退避ファイルにも入らない）。restore のときに
標準入力から 1 行読む。

実行: host/.venv/bin/python script/kv_transfer.py {backup,restore} ...
"""
import argparse
import datetime
import getpass
import glob
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'host'))
from imu_host import board  # noqa: E402

DEFAULT_FILE = os.path.join(ROOT, 'host', 'recordings', 'kv-backup.json')
DEFAULT_SSID = 'rp2040-test'
FORMAT = 'rp2040-connect-kv-backup'

# 校正値のキーの数字は感度（加速度は ug/LSB、角速度は udps/LSB）。LSM6DSOX の各フルスケール
# に対応する値をすべて並べる（ドライバーの GAIN_UNIT_XL=61、GAIN_UNIT_G=4375 から）。
# 起動時は 8 g・2000 dps（244、70000）だが、`sensor attr_set` でフルスケールを変えて校正すれば
# 他のキーにも保存され得る。無いキーは null として記録されるだけ。
ACCEL_GAINS = (61, 122, 244, 488)            # ±2, 4, 8, 16 g
GYRO_GAINS = (4375, 8750, 17500, 35000, 70000)  # ±125, 250, 500, 1000, 2000 dps
DEFAULT_KEYS = (['imu.host', 'imu.port', 'imu.device', 'imu.odr', 'imu.watermark']
                + ['imu.cal.accel.%d' % g for g in ACCEL_GAINS]
                + ['imu.cal.gyro.%d' % g for g in GYRO_GAINS])
# セッションの通し番号は実行時の状態なので移さない（移行先では初期値から数え直す）。
# 退避ファイルに入っていても書き戻さない。
SESSION_KEY = 'imu.session'

KEY_RE = re.compile(r'[A-Za-z0-9_.-]{1,32}')
VALUE_MAX = 128
ENOENT = 2
PSK_MIN, PSK_MAX = 8, 63
SSID_MAX = 32

ZEPHYR_USB_GLOB = board.USB_CONSOLE_GLOB
NUTTX_USB_GLOB = '/dev/serial/by-id/usb-*NuttX_USB_Console*'
# プロンプト（Zephyr の USB は "uart:~$ "、Telnet は "~$ "、NuttX 版は "nsh$ "）。
_PROMPT = re.compile(r'^[\w.:~/-]*\$\s*')


class TransferError(RuntimeError):
    pass


# ---------------------------------------------------------------- 接続

def find_usb():
    """Zephyr 版と NuttX 版の USB コンソールを探し、1 つに決まればそれを返す。"""
    found = sorted(set(glob.glob(ZEPHYR_USB_GLOB) + glob.glob(NUTTX_USB_GLOB)))
    if len(found) == 1:
        return found[0]
    if not found:
        raise TransferError('USB コンソールが見つかりません（Zephyr 版・NuttX 版とも）。ボードを USB で'
                            '接続するか、--usb でデバイスを指定してください')
    raise TransferError('USB コンソールが複数あります（%s）。--usb で指定してください' % ', '.join(found))


def open_shell(args):
    """引数から接続を 1 本作る（実際に開くのは最初のコマンドのとき）。"""
    if args.usb is not None:
        return board.Shell(usb=args.usb or find_usb())
    return board.Shell(ip=args.board or board.find())


def _body_lines(output, line):
    """応答から、コマンドのエコーとプロンプトだけの行を除いた行を返す。"""
    lines = []
    for raw in output.splitlines():
        s = _PROMPT.sub('', raw.strip()).strip()
        if s and line not in s:
            lines.append(s)
    return lines


def detect_firmware(sh):
    """ファームウェアの種類と版を調べる（kernel version → uname -a）。"""
    out = sh.command('kernel version', wait=1.5)
    m = re.search(r'Zephyr version \S+', out)
    if m:
        return {'kind': 'zephyr', 'version': m.group(0)}
    out = sh.command('uname -a', wait=1.5)
    m = re.search(r'NuttX[^\r\n]*', out)
    if m:
        return {'kind': 'nuttx', 'version': m.group(0).strip()}
    return {'kind': 'unknown', 'version': None}


# ---------------------------------------------------------------- KV

def check_key(key):
    if not KEY_RE.fullmatch(key):
        raise TransferError('キー名が不正です: %r（英数字・_・-・. の 1〜32 文字）' % key)
    if key.startswith('sys.'):
        raise TransferError('sys. で始まるキーはシェルから扱えません: %s' % key)


def parse_kv_get(output, key):
    """`kv get <key>` の応答を解釈する。値（bytes）か、キーが無ければ None を返す。

    kv_store.c の cmd_get は、値を小文字の 16 進数で 1 行に表示し（bin2hex、最大 256 文字）、
    無ければ "KV read failed: -2"（-ENOENT）を表示する。端末の幅での折り返しに備え、
    16 進数だけの行は連結する。それ以外の応答はエラーにする。
    """
    cmd = 'kv get %s' % key
    lines = _body_lines(output, cmd)
    shown = '\n'.join(lines) or '（応答なし）'
    if not lines:
        raise TransferError('%s の応答がありません' % cmd)
    if len(lines) == 1:
        m = re.fullmatch(r'KV read failed: (-?\d+)', lines[0])
        if m and int(m.group(1)) == -ENOENT:
            return None
        if m:
            raise TransferError('%s を読めません（errno %s）' % (key, m.group(1)))
        if lines[0] == 'Reserved key':
            raise TransferError('%s はシェルから読めないキーです（Reserved key）' % key)
    if all(re.fullmatch(r'[0-9a-fA-F]+', s) for s in lines):
        hexstr = ''.join(lines).lower()
        if len(hexstr) % 2 == 0 and 2 <= len(hexstr) <= 2 * VALUE_MAX:
            return bytes.fromhex(hexstr)
    raise TransferError('%s の応答を解釈できません:\n%s' % (cmd, shown))


def read_key(sh, key):
    return parse_kv_get(sh.command('kv get %s' % key, wait=1.5), key)


def check_idle(sh):
    """計測中なら止める（calib.py の write と同じ判定）。"""
    if re.search(r'state:\s*(running|starting)', sh.command('imu status', wait=2.0)):
        raise TransferError('計測中は書き込めません。imu stop で止めてから実行してください')


def write_key(sh, key, value):
    """kv sethex で書き、Saved と読み返しを確かめる。"""
    out = sh.command('kv sethex %s %s' % (key, value.hex()), wait=2.0)
    if not re.search(r'^\s*Saved\s*$', out, re.M):
        raise TransferError('%s を書き込めませんでした:\n%s'
                            % (key, '\n'.join(_body_lines(out, 'kv sethex')) or '（応答なし）'))
    if read_key(sh, key) != value:
        raise TransferError('%s を読み返した内容が一致しません' % key)


def _text(value):
    try:
        s = value.decode('ascii')
    except UnicodeDecodeError:
        return None
    return s if s.isprintable() else None


def make_backup(values, firmware, link):
    return {
        'format': FORMAT,
        'version': 1,
        'created': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
        'firmware': dict(firmware, link=link),
        'note': 'Wi-Fi の資格情報は含みません。値は kv get の 16 進数。null は未設定。',
        'keys': {k: (None if v is None else v.hex()) for k, v in values.items()},
        'text': {k: _text(v) for k, v in values.items() if v is not None and _text(v) is not None},
    }


def save_backup(path, data, force=False):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | (os.O_TRUNC if force else os.O_EXCL)
    try:
        fd = os.open(path, flags, 0o600)
    except FileExistsError:
        raise TransferError('%s は既にあります。上書きするには --force を付けてください' % path) from None
    with os.fdopen(fd, 'w') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write('\n')


def load_backup(path):
    """退避ファイルを読み、{キー: bytes または None} を返す。"""
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError) as e:
        raise TransferError('%s を読めません: %s' % (path, e)) from e
    if not isinstance(data, dict) or data.get('format') != FORMAT or data.get('version') != 1 \
       or not isinstance(data.get('keys'), dict):
        raise TransferError('%s は kv_transfer.py の退避ファイルではありません' % path)
    values = {}
    for key, hexstr in data['keys'].items():
        check_key(key)
        if hexstr is None:
            values[key] = None
            continue
        if not isinstance(hexstr, str) or not re.fullmatch(r'(?:[0-9a-fA-F]{2}){1,%d}' % VALUE_MAX, hexstr):
            raise TransferError('%s の %s の値が不正です（1〜128 バイトの 16 進数）' % (path, key))
        values[key] = bytes.fromhex(hexstr)
    return data, values


# ---------------------------------------------------------------- Wi-Fi

def check_ssid(ssid):
    if not 1 <= len(ssid.encode()) <= SSID_MAX:
        raise TransferError('SSID は 1〜32 バイトにしてください')
    if not all(0x20 <= ord(c) <= 0x7e for c in ssid):
        raise TransferError('SSID には半角の英数字・記号・空白だけを使ってください（シェルで送れません）')


def check_password(password):
    # メッセージにパスワードの内容を入れない。
    if not PSK_MIN <= len(password) <= PSK_MAX:
        raise TransferError('パスワードは 8〜63 文字にしてください（%d 文字でした）' % len(password))
    if not all(0x20 <= ord(c) <= 0x7e for c in password):
        raise TransferError('パスワードには半角の英数字・記号・空白だけを使ってください')


# NuttX 版のシェル（nsh）の引用・展開（$、`、\ など）の扱いは設定次第でまだ決まっていない
# ので、引用なしで安全な文字と空白（二重引用符で囲む）だけを通す。
_NUTTX_SAFE = re.compile(r'[A-Za-z0-9 !%+,\-./:=?@^_~]+')


def quote_arg(value, firmware, what):
    """シェルの 1 引数として送る文字列を作る。送れない文字があれば TransferError。

    Zephyr のシェル（subsys/shell/shell_utils.c の make_argv）は、二重引用符の中で
    \\" と \\xHH を解釈し、それ以外の文字はそのまま渡す。\\、"、$ を \\xHH で送ると、
    エコーに引用符や $ がそのまま出ないので、board._exchange のプロンプト判定
    （行末の $）を誤らせない。
    """
    if firmware == 'zephyr':
        return '"%s"' % ''.join('\\x%02x' % ord(c) if c in '\\"$' else c for c in value)
    if firmware == 'nuttx':
        if not _NUTTX_SAFE.fullmatch(value):
            raise TransferError('%sに NuttX 版のシェルで送れない文字があります（使えるのは英数字、空白、'
                                '!%%+,-./:=?@^_~）' % what)
        return '"%s"' % value if ' ' in value else value
    raise TransferError('ファームウェアの種類が分からないため、%sの引用の仕方を決められません' % what)


def wifi_add_command(ssid, password, firmware):
    """送るコマンドと、パスワードを伏せた表示用の文字列を返す。"""
    s = quote_arg(ssid, firmware, 'SSID ')
    p = quote_arg(password, firmware, 'パスワード')
    return ('wifi cred add -s %s -k 1 -p %s' % (s, p),
            'wifi cred add -s %s -k 1 -p ********' % s)


def _hide(text, secret):
    return text.replace(secret, '********') if secret else text


def wifi_response_lines(output, line):
    """応答から、エコー（端末の幅で折り返した断片を含む）とプロンプトを除いた行を返す。"""
    lines = []
    for raw in output.splitlines():
        s = _PROMPT.sub('', raw.strip()).strip()
        if s and s not in line:
            lines.append(s)
    return lines


# wifi_credentials_shell.c の cmd_add_network が出すメッセージ。成功時は何も出さない。
_ADD_ERRORS = (
    (r'SSID too long', 'SSID が長すぎます'),
    (r'Passphrase should be minimum', 'パスワードが短すぎます'),
    (r'Passphrase too long', 'パスワードが長すぎます'),
    (r'Passphrase provided without security', 'セキュリティの種類が指定されていません'),
    (r'SSID not provided', 'SSID が渡っていません（引用の解釈の違いの可能性）'),
    (r'Failed to add network credentials, err: (-?\d+)', '資格情報を保存できませんでした（err %s）'),
    (r'Invalid option', 'オプションが受け付けられませんでした'),
    (r'command not found|wrong parameter count', 'wifi cred add がありません、または引数が合いません'),
)


def wifi_add(sh, ssid, password, firmware):
    line, shown = wifi_add_command(ssid, password, firmware)
    out = sh.command(line, wait=3.0)
    lines = [_hide(s, password) for s in wifi_response_lines(out, line)]
    if not lines and out.rstrip().endswith('$'):
        return shown
    for pattern, message in _ADD_ERRORS:
        for s in lines:
            m = re.search(pattern, s)
            if m:
                raise TransferError('Wi-Fi の資格情報を登録できません: ' + (message % m.groups()))
    if not lines:
        raise TransferError('wifi cred add のあとプロンプトが戻りません')
    # Zephyr 版は成功時に何も出さないので、知らない出力は失敗として扱う。
    raise TransferError('wifi cred add の応答を確認できません:\n' + '\n'.join(lines))


def wifi_auto_connect(sh, password):
    out = sh.command('wifi cred auto_connect', wait=5.0)
    lines = [_hide(s, password) for s in wifi_response_lines(out, 'wifi cred auto_connect')]
    for s in lines:
        if re.search(r'error occurred|command not found|Failed', s):
            raise TransferError('wifi cred auto_connect が失敗しました: ' + s)
    if not out.rstrip().endswith('$'):
        return '応答待ちの間にプロンプトが戻りませんでした。wifi status で確かめてください'
    return '\n'.join(lines) or 'OK'


def read_password(stream=None):
    """パスワードを 1 行読む。端末ならエコーなし、パイプならそのまま。"""
    stream = stream or sys.stdin
    if stream.isatty():
        try:
            return getpass.getpass('Wi-Fi のパスワード（8〜63 文字）: ')
        except EOFError:
            raise TransferError('パスワードを読み取れません') from None
    line = stream.readline()
    if not line:
        raise TransferError('パスワードを読み取れません（標準入力が空です）')
    return line[:-1] if line.endswith('\n') else line


# ---------------------------------------------------------------- コマンド

def run_backup(sh, keys, path, force=False, out=None):
    out = out or sys.stdout
    firmware = detect_firmware(sh)
    print('ボード: %s（%s）' % (sh.name, firmware['version'] or '種類不明'), file=out)
    if firmware['kind'] != 'zephyr':
        print('注意: Zephyr 版ではないボードから読み出しています', file=out)
    values = {}
    for key in keys:
        values[key] = read_key(sh, key)
        v = values[key]
        print('  %-20s %s' % (key, '（未設定）' if v is None else '%d バイト' % len(v)), file=out)
    if all(v is None for v in values.values()):
        print('注意: どのキーも設定されていません', file=out)
    save_backup(path, make_backup(values, firmware, sh.name), force)
    print('保存しました: %s' % path, file=out)
    return values


def run_restore(sh, values, kv=True, wifi=None, out=None):
    """values を書き戻し、wifi=(ssid, password) なら資格情報を登録する。結果の一覧を返す。"""
    out = out or sys.stdout
    password = wifi[1] if wifi else None
    results = []
    firmware = detect_firmware(sh)
    print('ボード: %s（%s）' % (sh.name, firmware['version'] or '種類不明'), file=out)
    if wifi:
        wifi_add_command(wifi[0], password, firmware['kind'])   # 送れない文字なら書く前に止める
    try:
        if kv:
            check_idle(sh)
            written = set()
            for key, value in values.items():
                if value is None:
                    results.append((key, '飛ばした（退避時に未設定）'))
                    continue
                if key == SESSION_KEY:
                    results.append((key, '飛ばした（実行時の状態なので書き戻さない）'))
                    continue
                write_key(sh, key, value)
                written.add(key)
                results.append((key, '書き込み・読み返し OK（%d バイト）' % len(value)))
            if any(k.startswith('imu.cal.') for k in written):
                status = sh.command('imu cal reload', wait=1.5)
                lines = [s.strip() for s in status.splitlines() if re.match(r'\s*(accel|gyro):', s)]
                if len(lines) != 2:
                    raise TransferError('imu cal reload の応答を解釈できません:\n' + status.strip())
                for s in lines:
                    m = re.search(r'key (\S+?)\)', s)
                    applied = 'calibrated' in s.split('(')[0]
                    if m and m.group(1) in written and not applied:
                        raise TransferError('書き戻した校正値をボードが適用しません: ' + s)
                    results.append(('imu cal reload', s))
        if wifi:
            shown = wifi_add(sh, wifi[0], password, firmware['kind'])
            results.append((shown, 'OK'))
            results.append(('wifi cred auto_connect', wifi_auto_connect(sh, password)))
    finally:
        print('結果:', file=out)
        for key, result in results:
            print('  %-24s %s' % (key, _hide(result, password)), file=out)
    return results


def dry_run(args, out=None):
    """ボードに繋がずに、送るコマンドを表示する（パスワードは伏せる）。"""
    out = out or sys.stdout
    print('（--dry-run: ボードには接続しません）', file=out)
    if args.cmd == 'backup':
        print('  kernel version（Zephyr でなければ uname -a）', file=out)
        for key in args.keys:
            print('  kv get %s' % key, file=out)
        print('保存先: %s' % args.out, file=out)
        return 0
    _, values = load_backup(args.infile)
    print('  kernel version（Zephyr でなければ uname -a）', file=out)
    if not args.no_kv:
        print('  imu status（計測中なら止める）', file=out)
        for key, value in values.items():
            if value is None:
                print('  # %s は未設定なので書かない' % key, file=out)
                continue
            if key == SESSION_KEY:
                print('  # %s は実行時の状態なので書かない' % key, file=out)
                continue
            print('  kv sethex %s %s' % (key, value.hex()), file=out)
            print('  kv get %s' % key, file=out)
        if any(k.startswith('imu.cal.') and v is not None for k, v in values.items()):
            print('  imu cal reload', file=out)
    if not args.no_wifi:
        check_ssid(args.ssid)
        password = read_password()
        check_password(password)
        _, shown = wifi_add_command(args.ssid, password, args.firmware)
        print('  %s（%s 版の引用）' % (shown, args.firmware), file=out)
        print('  wifi cred auto_connect', file=out)
    return 0


EPILOG = '''例:
  # Zephyr 版が動いている間に退避する（USB、デバイスは自動で探す）
  host/.venv/bin/python script/kv_transfer.py backup --usb
  # Telnet で、出力先を指定して
  host/.venv/bin/python script/kv_transfer.py backup --board 10.42.0.23 --out kv.json
  # NuttX 版を書き込んだあとで書き戻す（パスワードは端末で入力）
  host/.venv/bin/python script/kv_transfer.py restore --usb
  # パスワードをパイプで渡し、SSID を変える
  pass show wifi/ap | host/.venv/bin/python script/kv_transfer.py restore --usb --ssid my-ap
  # 送るコマンドだけを表示する
  host/.venv/bin/python script/kv_transfer.py restore --dry-run --no-wifi

sys. で始まるキーはシェルから読めない（Reserved key）ので対象外です。
Wi-Fi のパスワードは退避ファイルに書きません。restore で標準入力から 1 行読みます。
'''


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        prog='kv_transfer.py', formatter_class=argparse.RawDescriptionHelpFormatter,
        description='KV（設定と校正値）を Zephyr 版から退避し、NuttX 版へ書き戻す', epilog=EPILOG)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--board', help='ボードの IP アドレス（Telnet。既定: AP に繋がっている機器から探す）')
    common.add_argument('--usb', nargs='?', const='', metavar='DEVICE',
                        help='Telnet ではなく USB コンソールを使う（デバイス省略時は Zephyr 版・'
                             'NuttX 版の by-id 名から探す）')
    common.add_argument('--dry-run', action='store_true',
                        help='ボードに接続せず、送るコマンドを表示する（パスワードは伏せる）')
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('backup', parents=[common], help='既知のキーを kv get で読んでファイルに保存する',
                       formatter_class=argparse.RawDescriptionHelpFormatter, epilog=EPILOG)
    p.add_argument('--out', default=DEFAULT_FILE, help='保存先（既定: %(default)s）')
    p.add_argument('--key', action='append', default=[], metavar='KEY',
                   help='追加で読むキー（複数回指定可）。sys. で始まるキーは対象外')
    p.add_argument('--force', action='store_true', help='保存先が既にあれば上書きする')

    p = sub.add_parser('restore', parents=[common], help='ファイルの値を kv sethex で書き戻し、Wi-Fi を登録する',
                       formatter_class=argparse.RawDescriptionHelpFormatter, epilog=EPILOG)
    p.add_argument('--in', dest='infile', default=DEFAULT_FILE, help='退避ファイル（既定: %(default)s）')
    p.add_argument('--ssid', default=DEFAULT_SSID, help='Wi-Fi の SSID（既定: %(default)s）')
    p.add_argument('--no-wifi', action='store_true', help='Wi-Fi を登録しない（パスワードを聞かない）')
    p.add_argument('--no-kv', action='store_true', help='KV を書き戻さない')
    p.add_argument('--firmware', choices=('zephyr', 'nuttx'), default='nuttx',
                   help='--dry-run での引用の仕方（実行時はボードから判定する。既定: %(default)s）')
    args = ap.parse_args(argv)
    if args.cmd == 'backup':
        args.keys = list(dict.fromkeys(DEFAULT_KEYS + args.key))
        for key in args.keys:
            check_key(key)
    return args


def main(argv=None):
    try:
        args = parse_args(argv)
        if args.dry_run:
            return dry_run(args)
        if args.cmd == 'backup':
            if os.path.exists(args.out) and not args.force:
                raise TransferError('%s は既にあります。上書きするには --force を付けてください' % args.out)
            with open_shell(args) as sh:
                run_backup(sh, args.keys, args.out, args.force)
            return 0
        values = {}
        if not args.no_kv:
            _, values = load_backup(args.infile)
        wifi = None
        if not args.no_wifi:
            check_ssid(args.ssid)
            password = read_password()
            check_password(password)
            wifi = (args.ssid, password)
        if args.no_kv and not wifi:
            print('--no-kv と --no-wifi を両方付けると何もしません', file=sys.stderr)
            return 2
        with open_shell(args) as sh:
            run_restore(sh, values, kv=not args.no_kv, wifi=wifi)
        return 0
    except (TransferError, board.BoardError) as e:
        print('エラー: %s' % e, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    sys.exit(main())
