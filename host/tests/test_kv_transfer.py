# SPDX-License-Identifier: MIT
"""script/kv_transfer.py のファームウェアの判定と、版に依る部分を、ボードなしで確かめる。

  cd host && .venv/bin/python -m unittest discover -s tests
"""
import fnmatch
import importlib.util
import io
import os
import sys
import unittest
from unittest import mock

from imu_host import board

from test_board import FREERTOS_BY_ID, NUTTX_BY_ID, ZEPHYR_BY_ID, FakeLink

_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                     'script', 'kv_transfer.py')
_spec = importlib.util.spec_from_file_location('kv_transfer', _PATH)
kv_transfer = importlib.util.module_from_spec(_spec)
# script/ に __pycache__ を作らない（script/ は Git の管理下で、無視の設定が無い）。
_write_bytecode, sys.dont_write_bytecode = sys.dont_write_bytecode, True
try:
    _spec.loader.exec_module(kv_transfer)
finally:
    sys.dont_write_bytecode = _write_bytecode

ZEPHYR_VERSION = b'Zephyr version 4.5.0\r\n'
NUTTX_VERSION = b'NuttX 13.1.0 0f14ce0 Nov 17 2026 00:00:00 arm arduino-nano-rp2040-connect\r\n'
FREERTOS_VERSION = b'FreeRTOS kernel V11.3.3, Pico SDK 2.3.1, GCC 15.2.1 20251203\r\n'


def zephyr_telnet():
    out = {'kernel version': ZEPHYR_VERSION}
    return FakeLink(b'', lambda line: b'\r\n' + out.get(line, line.split()[0].encode()
                                                        + b': command not found\r\n')
                    + b'\x1b[1;32m~$ \x1b[m')


def nuttx_telnet():
    out = {'uname -a': NUTTX_VERSION}
    return FakeLink(b'nsh$ ', lambda line: line.encode() + b'\r\n' + out.get(
        line, b'nsh: ' + line.split()[0].encode() + b': command not found\r\n') + b'nsh$ ')


def freertos(usb):
    """FreeRTOS 版（USB はエコーあり、Telnet はエコーなしで接続直後は無言）。"""
    out = {'version': FREERTOS_VERSION}
    echo = (lambda line: line.encode() + b'\r\n') if usb else (lambda line: b'\r\n')
    return FakeLink(b'freertos:~$ ' if usb else b'', lambda line: echo(line) + out.get(
        line, line.split()[0].encode() + b': command not found\r\n') + b'freertos:~$ ')


def detect(link, **kw):
    with mock.patch.object(board, '_Telnet', lambda ip: link), \
         mock.patch.object(board, '_Serial', lambda dev: link):
        with board.Shell(**kw) as sh:
            return kv_transfer.detect_firmware(sh)


class DetectFirmwareTest(unittest.TestCase):
    def test_zephyr_telnet(self):
        link = zephyr_telnet()
        self.assertEqual(detect(link, ip='10.42.0.97'),
                         {'kind': 'zephyr', 'version': 'Zephyr version 4.5.0'})
        self.assertEqual(link.sent, ['kernel version'])

    def test_nuttx_telnet_after_the_prompt(self):
        link = nuttx_telnet()
        fw = detect(link, ip='10.42.0.97')
        self.assertEqual(fw['kind'], 'nuttx')
        self.assertTrue(fw['version'].startswith('NuttX 13.1.0'))
        self.assertEqual(link.sent, ['kernel version', 'uname -a'])

    def test_freertos_telnet_after_the_prompt(self):
        # kernel version は無いが、その応答のプロンプトで FreeRTOS 版と分かり、uname -a を飛ばす。
        link = freertos(usb=False)
        fw = detect(link, ip='10.42.0.97')
        self.assertEqual(fw, {'kind': 'freertos',
                              'version': 'FreeRTOS kernel V11.3.3, Pico SDK 2.3.1, GCC 15.2.1 20251203'})
        self.assertEqual(link.sent, ['kernel version', 'version'])

    def test_freertos_usb_by_name(self):
        link = freertos(usb=True)
        self.assertEqual(detect(link, usb=FREERTOS_BY_ID)['kind'], 'freertos')
        self.assertEqual(link.sent, ['version'])

    def test_known_firmware_goes_first(self):
        link = nuttx_telnet()
        link.pending = []
        self.assertEqual(detect(link, usb=NUTTX_BY_ID)['kind'], 'nuttx')
        self.assertEqual(link.sent, ['uname -a'])

    def test_unknown(self):
        link = FakeLink(b'', lambda line: b'\r\n?\r\n$ ')
        self.assertEqual(detect(link, usb=ZEPHYR_BY_ID), {'kind': 'unknown', 'version': None})
        self.assertEqual(link.sent, ['kernel version', 'uname -a', 'version'])


class FirmwareDifferencesTest(unittest.TestCase):
    def test_freertos_quotes_as_zephyr(self):
        for ssid, pw in (('rp2040-test', 'pass word$1'), ('a"b', 'x\\y"z$12')):
            self.assertEqual(kv_transfer.wifi_add_command(ssid, pw, 'freertos'),
                             kv_transfer.wifi_add_command(ssid, pw, 'zephyr'))
        line, shown = kv_transfer.wifi_add_command('rp2040-test', 'pass$word', 'freertos')
        self.assertEqual(line, 'wifi cred add -s "rp2040-test" -k 1 -p "pass\\x24word"')
        self.assertNotIn('pass', shown.split('-p')[1])

    def test_eilseq_of_each_build(self):
        # NuttX 版は 84、Zephyr 版と FreeRTOS 版（newlib）は 138。
        for errno in (84, 138):
            with self.assertRaisesRegex(kv_transfer.TransferError, 'kv wipe confirm'):
                kv_transfer.parse_kv_get('KV read failed: -%d\nfreertos:~$ ' % errno, 'imu.port')
        self.assertIsNone(kv_transfer.parse_kv_get('KV read failed: -2\nfreertos:~$ ', 'imu.port'))
        self.assertEqual(kv_transfer.parse_kv_get('kv get imu.port\n3138383330\nfreertos:~$ ',
                                                  'imu.port'), b'18830')

    def test_usb_search_includes_freertos(self):
        self.assertIn('/dev/serial/by-id/usb-*FreeRTOS_USB_Console*', kv_transfer.USB_GLOBS)
        self.assertTrue(any(fnmatch.fnmatchcase(FREERTOS_BY_ID, g) for g in kv_transfer.USB_GLOBS))

    def test_dry_run_with_freertos(self):
        args = kv_transfer.parse_args(['restore', '--dry-run', '--no-kv', '--firmware', 'freertos'])
        out = io.StringIO()
        with mock.patch.object(kv_transfer, 'read_password', lambda: 'secret-pass'):
            self.assertEqual(kv_transfer.dry_run(args, out), 0)
        text = out.getvalue()
        self.assertIn('kernel version（違えば uname -a、version）', text)
        self.assertIn('wifi cred add -s "rp2040-test" -k 1 -p ********（freertos 版の引用）', text)
        self.assertNotIn('secret-pass', text)


if __name__ == '__main__':
    unittest.main()
