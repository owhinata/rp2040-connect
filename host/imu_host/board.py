# SPDX-License-Identifier: Apache-2.0
"""ボードの IP アドレスの特定と、Telnet シェルでのコマンド実行。"""
import glob
import json
import os
import re
import select
import socket
import subprocess
import termios
import time
import tty

TELNET_PORT = 23
_ANSI = re.compile(rb'\x1b\[[0-9;]*[A-Za-z]')
_IAC = re.compile(rb'\xff[\xfb-\xfe].|\xff[\xf0-\xfa]', re.S)


class BoardError(RuntimeError):
    pass


def _ip_json(*args):
    out = subprocess.run(('ip', '-j') + args, capture_output=True, text=True, check=True).stdout
    return json.loads(out or '[]')


def ap_interface(gateway='10.42.0.1'):
    """gateway の IPv4 アドレスを持つインターフェース名を返す。なければ None。

    10.42.0.1 は NetworkManager の共有接続（script/wifi_ap.sh の AP）の既定値。
    """
    for iface in _ip_json('-4', 'addr'):
        for a in iface.get('addr_info', []):
            if a.get('local') == gateway:
                return iface['ifname']
    return None


def neighbours(ifname):
    """ifname の近隣テーブルにある、到達できている IPv4 アドレスの一覧。"""
    return [n['dst'] for n in _ip_json('-4', 'neigh', 'show', 'dev', ifname)
            if 'lladdr' in n and not set(n.get('state', [])) & {'FAILED', 'INCOMPLETE'}]


def _answers_telnet(ip):
    try:
        socket.create_connection((ip, TELNET_PORT), timeout=1.0).close()
        return True
    except OSError:
        return False


def find(gateway='10.42.0.1'):
    """AP に繋がっているボードの IP アドレスを返す。決められなければ BoardError。"""
    ifname = ap_interface(gateway)
    if not ifname:
        raise BoardError('%s を持つインターフェースがありません。AP が起動しているか確認するか、'
                         '--board で IP アドレスを指定してください' % gateway)
    found = neighbours(ifname)
    if len(found) > 1:
        found = [ip for ip in found if _answers_telnet(ip)]
    if len(found) == 1:
        return found[0]
    if not found:
        raise BoardError('%s に繋がっている機器が見つかりません。ボードの電源と Wi-Fi 接続を'
                         '確認するか、--board で指定してください' % ifname)
    raise BoardError('%s に Telnet へ応答する機器が複数あります（%s）。--board で指定してください'
                     % (ifname, ', '.join(found)))


def _clean_bytes(data):
    return _ANSI.sub(b'', _IAC.sub(b'', data))


def _clean(data):
    text = _clean_bytes(data).decode('utf-8', errors='replace')
    return text.replace('\r', '')


PROMPT = b'$ '


def _exchange(read, write, line, wait, drain=True):
    """line を送り、実行が終わってプロンプトが戻るまで（最長 wait 秒）の出力を返す。

    送る前に、接続直後のネゴシエーションやプロンプトなど、届いているものを読み捨てる。
    接続直後に送ると、シェルが受け付けを始める前のデータとして捨てられることがある
    ので、静かになるまで待つ意味もある。

    終わりの判定は「出力の後にプロンプトが来た」こと。読み捨てそこねた古いプロンプト
    だけを見て終わらないよう、プロンプトの前にコマンドの出力があることを条件にする
    （使うコマンドはどれも何かを出力する）。
    """
    quiet_since = time.monotonic()
    end = quiet_since + (3.0 if drain else 0.0)
    while time.monotonic() < end and time.monotonic() - quiet_since < 0.5:
        if read(0.1):
            quiet_since = time.monotonic()
    write(line.encode() + b'\r\n')
    data = b''
    end = time.monotonic() + wait
    while time.monotonic() < end:
        data += read(min(0.2, max(0.0, end - time.monotonic())))
        text = _clean(data)
        body = text.rstrip()
        if body.endswith(PROMPT.strip().decode()):
            # エコー（USB コンソール）と末尾のプロンプトを除いて、出力が残るか。
            lines = [ln for ln in body.splitlines()[:-1] if ln.strip() and line not in ln]
            if lines:
                break
    return _clean(data)


class _Telnet:
    """Telnet シェルへの 1 本の接続。"""

    def __init__(self, ip):
        self.ip = ip
        self.sock = socket.create_connection((ip, TELNET_PORT), timeout=5.0)

    def read(self, seconds):
        self.sock.settimeout(max(seconds, 0.01))
        try:
            data = self.sock.recv(4096)
        except socket.timeout:
            return b''
        if not data:
            raise ConnectionResetError('closed by the board')
        return data

    def write(self, data):
        self.sock.sendall(data)

    def close(self):
        self.sock.close()


class _Serial:
    """USB コンソールのシェルへの接続。"""

    def __init__(self, device):
        self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            tty.setraw(self.fd)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
            # 入力途中の行があれば捨てる。
            os.write(self.fd, b'\x03')
        except (OSError, termios.error):
            os.close(self.fd)
            raise

    def read(self, seconds):
        if select.select([self.fd], [], [], seconds)[0]:
            return os.read(self.fd, 4096)
        return b''

    def write(self, data):
        os.write(self.fd, data)

    def close(self):
        os.close(self.fd)


USB_CONSOLE_GLOB = '/dev/serial/by-id/usb-Zephyr_Project_Zephyr_USB_Console_*'


def usb_console():
    """USB コンソールのデバイスを返す。1 つに決まらなければ BoardError。"""
    found = sorted(glob.glob(USB_CONSOLE_GLOB))
    if len(found) == 1:
        return found[0]
    if not found:
        raise BoardError('USB コンソールが見つかりません。ボードを USB で PC に接続するか、'
                         '--usb でデバイスを指定してください')
    raise BoardError('USB コンソールが複数あります（%s）。--usb で指定してください' % ', '.join(found))


class Shell:
    """ボードのシェルへの経路。Telnet と USB コンソールで同じコマンドを送る。

    接続は最初のコマンドで開き、close() まで使い回す。ボードの Telnet は同時に
    1 接続で、接続と切断を短い間隔で繰り返すと受け付けなくなることがあるため、
    コマンドごとに繋ぎ直さない。
    """

    def __init__(self, ip=None, usb=None):
        self.ip, self.usb = ip, usb
        self.name = 'USB %s' % usb if usb else 'Telnet %s' % ip
        self.link = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        if self.link:
            try:
                self.link.close()
            except OSError:
                pass
            self.link = None

    def command(self, line, wait=1.5):
        """line を実行し、その出力を文字列で返す（最長 wait 秒）。"""
        try:
            fresh = self.link is None
            if fresh:
                self.link = _Serial(self.usb) if self.usb else _Telnet(self.ip)
            return _exchange(self.link.read, self.link.write, line, wait, drain=fresh)
        except (OSError, termios.error) as e:
            self.close()
            raise BoardError('%s でコマンドを実行できません: %s' % (self.name, e)) from e


def command(ip, line, wait=1.5):
    """Telnet シェルで line を 1 つ実行し、その出力を文字列で返す。"""
    with Shell(ip=ip) as sh:
        return sh.command(line, wait)


def serial_command(device, line, wait=1.5):
    """USB コンソールのシェルで line を 1 つ実行し、その出力を文字列で返す。"""
    with Shell(usb=device) as sh:
        return sh.command(line, wait)


def shell(board=None, usb=None):
    """引数から経路を決める。usb は None（使わない）、''（自動で探す）、デバイス名。"""
    if usb is not None:
        return Shell(usb=usb or usb_console())
    return Shell(ip=board or find())
