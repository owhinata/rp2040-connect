# SPDX-License-Identifier: Apache-2.0
"""ボードの IP アドレスの特定と、Telnet シェルでのコマンド実行。"""
import json
import re
import socket
import subprocess
import time

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


def command(ip, line, wait=1.5):
    """Telnet シェルで line を実行し、wait 秒の間に返ってきた出力を文字列で返す。"""
    try:
        with socket.create_connection((ip, TELNET_PORT), timeout=5.0) as c:
            c.settimeout(0.2)
            data = b''
            # 接続直後のネゴシエーションとプロンプトを読み捨ててから送る。
            for end, send in ((time.monotonic() + 0.5, None), (time.monotonic() + 0.5 + wait, line)):
                if send is not None:
                    data = b''
                    c.sendall(send.encode() + b'\r\n')
                while time.monotonic() < end:
                    try:
                        chunk = c.recv(4096)
                    except socket.timeout:
                        continue
                    if not chunk:
                        break
                    data += chunk
    except OSError as e:
        raise BoardError('%s の Telnet に接続できません: %s' % (ip, e)) from e
    text = _ANSI.sub(b'', _IAC.sub(b'', data)).decode('utf-8', errors='replace')
    return text.replace('\r', '')
