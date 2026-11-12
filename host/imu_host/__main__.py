# SPDX-License-Identifier: Apache-2.0
import argparse
import sys


def main():
    ap = argparse.ArgumentParser(prog='imu-host', description='IMU の RAW データの記録・校正・診断')
    sub = ap.add_subparsers(dest='command', required=True)

    p = sub.add_parser('record', help='計測を開始して RAW を記録する')
    p.add_argument('--out', help='記録先のディレクトリ（既定: recordings/<日時>）')
    p.add_argument('--bind', default='0.0.0.0', help='待ち受ける PC の IPv4 アドレス')
    p.add_argument('--port', type=int, default=1883, help='待ち受けるポート（ボードの imu.port）')
    p.add_argument('--board', help='ボードの IP アドレス（既定: AP に繋がっている機器から探す）')
    p.add_argument('--no-board', action='store_true',
                   help='ボードを操作しない（imu start / stop は手動で行う）')
    p.add_argument('--duration', type=float, help='最初の RAW からこの秒数で終了')

    p = sub.add_parser('calib', help='記録から imu_tk で校正値を求める')
    p.add_argument('recording', help='record が作ったディレクトリ')
    p.add_argument('--gravity', type=float, default=9.7975, help='重力 [m/s^2]（既定は東京）')
    p.add_argument('--init-static', type=float, default=10.0, help='最初の静止時間 [秒]')
    p.add_argument('--interval', type=float, default=1.0,
                   help='各静止区間から使う長さと、静止判定の窓 [秒]')
    p.add_argument('--force', action='store_true', help='欠落や飽和があっても実行する')

    p = sub.add_parser('write', help='校正値をボードに書き込む')
    p.add_argument('calibration', help='calibration.json、または calib 済みの記録のディレクトリ')
    p.add_argument('--board', help='ボードの IP アドレス（Telnet。既定: AP に繋がっている機器から探す）')
    p.add_argument('--usb', nargs='?', const='', metavar='DEVICE',
                   help='Telnet ではなく USB コンソールで書き込む（デバイス省略時は自動で探す）')
    p.add_argument('--force', action='store_true', help='注意の付いた校正値でも書き込む')

    p = sub.add_parser('check', help='RAW の受信を診断する（保存しない）')
    p.add_argument('--bind', default='0.0.0.0', help='待ち受ける PC の IPv4 アドレス')
    p.add_argument('--port', type=int, default=1883, help='待ち受けるポート（ボードの imu.port）')
    p.add_argument('--session', help='期待する 32 桁 hex のセッション ID（省略時は最初の値）')
    p.add_argument('--duration', type=float, help='起動からこの秒数で終了')
    p.add_argument('--idle-timeout', type=float, help='RAW 受信後、この秒数受信がなければ終了')
    p.add_argument('--drop-at', type=float, help='最初の RAW からこの秒数後に切断し待ち受けも停止')
    p.add_argument('--drop-for', type=float, default=5.0)
    p.add_argument('--stall-at', type=float, help='最初の RAW からこの秒数後に読み出しを停止')
    p.add_argument('--stall-for', type=float, default=5.0)
    p.add_argument('--rcvbuf', type=int, help='SO_RCVBUF。小さくすると読み出し停止が送信側へ早く伝わる')

    args = ap.parse_args()
    if args.command == 'record':
        from . import record
        return record.run(args)
    if args.command == 'write':
        from . import calib
        return calib.write(args)
    if args.command == 'check':
        from . import check
        return check.run(args)
    from . import calib
    return calib.run(args)


if __name__ == '__main__':
    sys.exit(main())
