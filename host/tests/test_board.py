# SPDX-License-Identifier: Apache-2.0
"""board.py のファームウェアの判定とプロンプトの扱いを、ボードなしで確かめる。

  cd host && .venv/bin/python -m unittest discover -s tests
"""
import os
import tempfile
import time
import unittest
from unittest import mock

from imu_host import board

IAC_WILL_ECHO = b'\xff\xfb\x01'
IAC_WONT_ECHO = b'\xff\xfc\x01'
ZEPHYR_BY_ID = '/dev/serial/by-id/usb-Zephyr_Project_Zephyr_USB_Console_503533748A8C9218-if00'
NUTTX_BY_ID = '/dev/serial/by-id/usb-NuttX_NuttX_USB_Console_503533748A8C9218-if00'


def by_name(name):
    return next(p for p in board.profiles() if p.name == name)


class FakeLink:
    """シェルの代わり。connect は接続直後に届くもの、reply(line) は 1 行への応答のバイト列。"""

    def __init__(self, connect, reply):
        self.pending = [connect] if connect else []
        self.reply = reply
        self.sent = []

    def read(self, seconds):
        if self.pending:
            return self.pending.pop(0)
        time.sleep(min(seconds, 0.01))
        return b''

    def write(self, data):
        line = data.decode().rstrip('\r\n')
        self.sent.append(line)
        # ネットワークのように、少しずつ届ける。
        out = self.reply(line)
        self.pending += [out[i:i + 7] for i in range(0, len(out), 7)]

    def close(self):
        pass


def zephyr_telnet(outputs):
    """Zephyr 版の Telnet: エコーなし、接続しただけでは何も来ない、プロンプト "~$ "。"""
    return FakeLink(b'', lambda line: b'\r\n' + outputs[line] + b'\x1b[1;32m~$ \x1b[m')


def zephyr_usb(outputs):
    """Zephyr 版の USB: エコーあり、プロンプト "uart:~$ "。"""
    return FakeLink(b'\r\n\x1b[1;32muart:~$ \x1b[m',
                    lambda line: line.encode() + b'\r\n' + outputs[line] + b'\x1b[1;32muart:~$ \x1b[m')


def nuttx(outputs, telnet=True):
    """NuttX 版: 接続直後にプロンプト、readline のエコー、Telnet では行ごとに ECHO の交渉。"""
    pre, post = (IAC_WILL_ECHO, IAC_WONT_ECHO) if telnet else (b'', b'')
    return FakeLink(pre + b'nsh$ \x1b[K',
                    lambda line: line.encode() + b'\r\n' + post + outputs[line] + pre + b'nsh$ \x1b[K')


OUTPUTS = {
    'imu status': b'state: idle\r\n',
    'kv sethex imu.cal.accel.244 0102': b'Saved\r\n',
    'kv get imu.cal.accel.244': b'0102\r\n',
    'imu cal reload': (b'conversion:\r\n  accel: calibrated (244 ug/LSB, key imu.cal.accel.244)\r\n'
                       b'  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)\r\n'),
}


class ProfileTest(unittest.TestCase):
    def test_file_has_both_firmwares_and_zephyr_first(self):
        names = [p.name for p in board.load_profiles()]
        self.assertEqual(names, ['zephyr', 'nuttx'])

    def test_zephyr_values_are_the_old_hard_coded_ones(self):
        z = board.load_profiles()[0]
        self.assertEqual(z.usb_glob, board.USB_CONSOLE_GLOB)
        self.assertEqual(z.usb_glob, '/dev/serial/by-id/usb-Zephyr_Project_Zephyr_USB_Console_*')
        self.assertEqual(z.prompt.encode(), board.PROMPT)
        self.assertEqual(z.prompt, '$ ')

    def test_missing_file_falls_back_to_zephyr(self):
        p = board.load_profiles('/nonexistent/firmware_profiles.toml')
        self.assertEqual([(f.name, f.usb_glob, f.prompt) for f in p],
                         [('zephyr', board.USB_CONSOLE_GLOB, '$ ')])

    def test_broken_file_is_an_error(self):
        with tempfile.NamedTemporaryFile('w', suffix='.toml', delete=False) as f:
            f.write('[[firmware]]\nname = "x"\n')
        try:
            with self.assertRaises(board.BoardError):
                board.load_profiles(f.name)
        finally:
            os.unlink(f.name)


class DetectTest(unittest.TestCase):
    def test_usb_by_id_name(self):
        self.assertEqual(board.firmware_from_device(ZEPHYR_BY_ID).name, 'zephyr')
        self.assertEqual(board.firmware_from_device(NUTTX_BY_ID).name, 'nuttx')
        self.assertIsNone(board.firmware_from_device('/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_X-if02'))

    def test_usb_device_behind_the_by_id_link(self):
        with tempfile.TemporaryDirectory() as d:
            tty = os.path.join(d, 'ttyACM7')
            open(tty, 'w').close()
            link = os.path.join(d, 'usb-NuttX_NuttX_USB_Console_X-if00')
            os.symlink(tty, link)
            fw = (board.Firmware('zephyr', os.path.join(d, 'usb-Zephyr_*'), '$ '),
                  board.Firmware('nuttx', os.path.join(d, 'usb-NuttX_NuttX_USB_Console_*'), 'nsh$ '))
            self.assertEqual(board.firmware_from_device(tty, fw).name, 'nuttx')
            self.assertIsNone(board.firmware_from_device(os.path.join(d, 'other'), fw))

    def test_prompt(self):
        cases = {'state: idle\nuart:~$ ': 'zephyr', 'state: idle\n~$ ': 'zephyr', '~$': 'zephyr',
                 'state: idle\nnsh$ ': 'nuttx', 'nsh$': 'nuttx', 'Saved\n': None, '': None}
        for text, name in cases.items():
            fw = board.firmware_from_prompt(text)
            self.assertEqual(fw.name if fw else None, name, text)

    def test_usb_console_searches_every_firmware(self):
        with tempfile.TemporaryDirectory() as d:
            fw = (board.Firmware('zephyr', os.path.join(d, 'usb-Zephyr_*'), '$ '),
                  board.Firmware('nuttx', os.path.join(d, 'usb-NuttX_*'), 'nsh$ '))
            with mock.patch.object(board, '_profiles', fw):
                with self.assertRaises(board.BoardError):
                    board.usb_console()
                open(os.path.join(d, 'usb-NuttX_A-if00'), 'w').close()
                self.assertEqual(board.usb_console(), os.path.join(d, 'usb-NuttX_A-if00'))
                open(os.path.join(d, 'usb-Zephyr_B-if00'), 'w').close()
                with self.assertRaises(board.BoardError):
                    board.usb_console()


class ShellTest(unittest.TestCase):
    def run_shell(self, link, **kw):
        with mock.patch.object(board, '_Telnet', lambda ip: link), \
             mock.patch.object(board, '_Serial', lambda dev: link):
            sh = board.Shell(**kw)
            out = [sh.command(line, wait=2.0) for line in OUTPUTS]
            sh.close()
        return sh, out, link

    def check(self, out):
        self.assertIn('state: idle', out[0])
        self.assertIn('Saved', out[1])
        self.assertIn('0102', out[2])
        self.assertEqual(sum('calibrated' in ln for ln in out[3].splitlines()), 2)

    def test_zephyr_telnet(self):
        sh, out, link = self.run_shell(zephyr_telnet(OUTPUTS), ip='10.42.0.97')
        self.check(out)
        self.assertEqual(sh.firmware.name, 'zephyr')
        self.assertEqual(link.sent, list(OUTPUTS))

    def test_zephyr_usb(self):
        sh, out, _ = self.run_shell(zephyr_usb(OUTPUTS), usb=ZEPHYR_BY_ID)
        self.check(out)
        self.assertEqual(sh.firmware.name, 'zephyr')

    def test_nuttx_telnet(self):
        sh, out, link = self.run_shell(nuttx(OUTPUTS), ip='10.42.0.97')
        self.check(out)
        self.assertEqual(sh.firmware.name, 'nuttx')
        self.assertNotIn('\xff', ''.join(out))
        self.assertEqual(link.sent, list(OUTPUTS))

    def test_nuttx_usb(self):
        sh, out, _ = self.run_shell(nuttx(OUTPUTS, telnet=False), usb=NUTTX_BY_ID)
        self.check(out)
        self.assertEqual(sh.firmware.name, 'nuttx')


class PromptInOutputTest(unittest.TestCase):
    """出力の行が "$" で終わっても、NuttX 版では "nsh$" が来るまで待つ。"""

    def link(self):
        link = FakeLink(b'', None)
        # 1 回目の読み出しで "$" で終わる行まで、2 回目で残りとプロンプト。
        link.write = lambda data: link.pending.extend(
            [b'i2c dump\r\n0000: 00\r\n0001: 24 24 24  $$$\r\n', b'0003: 00\r\nnsh$ '])
        return link

    def test_nuttx_waits_for_its_prompt(self):
        link = self.link()
        out = board._exchange(link.read, link.write, 'i2c dump', 2.0, drain=False,
                              firmware=by_name('nuttx'))
        self.assertTrue(out.rstrip().endswith('nsh$'), out)

    def test_unknown_firmware_ends_at_any_prompt(self):
        # どのファームウェアか決まる前は、以前と同じく "$" で終わる行で終わる。
        link = self.link()
        out = board._exchange(link.read, link.write, 'i2c dump', 2.0, drain=False)
        self.assertTrue(out.rstrip().endswith('$$$'), out)


if __name__ == '__main__':
    unittest.main()
