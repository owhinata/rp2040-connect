# SPDX-License-Identifier: Apache-2.0
"""計測を開始して RAW を記録し、停止する。"""
import json
import math
import os
import re
import signal
import sys
import time

import numpy as np

from . import board, mqtt, packet, recording
from .calib import MOVING_DPS, ROTATION_MIN_DEG, ROTATION_TARGET_DEG, informative_rotation

DPS_PER_RAD = 180.0 / math.pi


class PoseTracker:
    """静止している姿勢を数える。記録中の目安で、校正の静止判定とは別物である。

    約 0.1 秒ごとの窓で、角速度の平均が最初の静止時の値に近く、ばらつきも小さければ
    静止とみなす。静止が hold 秒続き、重力の向きが直前に数えた姿勢から angle 度以上
    違えば、新しい姿勢として数える。

    動いている間は、重力の向きを変える回転の量を軸ごとに積算する（rotation）。
    角速度の校正には、この回転が 3 軸に行き渡っている必要がある。
    """

    def __init__(self, gyro_dps_per_lsb, hold=3.0, angle=10.0, window=0.1):
        self.dps = gyro_dps_per_lsb
        self.hold, self.angle, self.window = hold, angle, window
        self.bias = None
        self.bias_samples = []
        self.gyro, self.accel = [], []
        self.gyro_n = 0
        self.static_since = None
        self.static_accel = np.zeros(3)
        self.static_n = 0
        self.counted = False
        self.last_dir = None
        self.poses = 0
        self.state = 'init'
        self.gyro_max_dps = 0.0
        self.saturated = 0
        self.rotation = np.zeros(3)

    def add(self, accel, gyro, odr, now):
        for data in (accel, gyro):
            if len(data):
                self.saturated += int((np.abs(data.astype(np.int32)) >= 32760).any(axis=1).sum())
        if len(gyro):
            self.gyro_max_dps = max(self.gyro_max_dps, float(np.abs(gyro).max()) * self.dps)
            self.gyro.append(gyro)
            self.gyro_n += len(gyro)
        if len(accel):
            self.accel.append(accel)
        if self.gyro_n < odr * self.window:
            return
        g = np.concatenate(self.gyro).astype(np.float64)
        a = np.concatenate(self.accel).astype(np.float64) if self.accel else np.zeros((0, 3))
        self.gyro, self.accel, self.gyro_n = [], [], 0
        mean = g.mean(axis=0)
        if self.bias is None:
            # 最初の 2 秒は静止している前提で、角速度のバイアスを見積もる。
            self.bias_samples.append(mean)
            if len(self.bias_samples) * self.window >= 2.0:
                self.bias = np.mean(self.bias_samples, axis=0)
            return
        rate = (mean - self.bias) * self.dps
        still = np.linalg.norm(rate) < MOVING_DPS and g.std(axis=0).max() * self.dps < 3.0
        if len(a) and np.linalg.norm(rate) >= MOVING_DPS:
            self.rotation += informative_rotation(a.mean(axis=0), rate, len(g) / odr)
        if not still:
            self.static_since = None
            self.state = 'moving'
            return
        if self.static_since is None:
            self.static_since = now
            self.static_accel = np.zeros(3)
            self.static_n = 0
            self.counted = False
        if len(a):
            self.static_accel += a.sum(axis=0)
            self.static_n += len(a)
        self.state = 'static'
        if not self.counted and now - self.static_since >= self.hold and self.static_n:
            direction = self.static_accel / np.linalg.norm(self.static_accel)
            if self.last_dir is None or \
               math.degrees(math.acos(float(np.clip(direction @ self.last_dir, -1, 1)))) >= self.angle:
                self.poses += 1
                self.last_dir = direction
            self.counted = True

    def static_seconds(self, now):
        return 0.0 if self.static_since is None else now - self.static_since


def _start(ip, port):
    out = board.command(ip, 'imu start', wait=4.5)
    m = re.search(r'port (\d+)', out)
    if 'Streaming' not in out:
        raise board.BoardError('imu start が受け付けられませんでした:\n' + out.strip())
    if m and int(m.group(1)) != port:
        board.command(ip, 'imu stop', wait=2.0)
        raise board.BoardError(
            'ボードの送信先ポートは %s ですが、待ち受けは %d です。--port %s を指定するか、'
            'ボードの KV の imu.port を変更してください' % (m.group(1), port, m.group(1)))


def run(args):
    out = args.out or os.path.join('recordings', time.strftime('%Y%m%d-%H%M%S'))
    if os.path.exists(out):
        print('%s は既にあります' % out, file=sys.stderr)
        return 2
    server = mqtt.Server(args.bind, args.port)
    ip = None
    try:
        if not args.no_board:
            ip = args.board or board.find()
            print('ボード: %s' % ip)
            _start(ip, args.port)
    except board.BoardError as e:
        server.close()
        print(e, file=sys.stderr)
        return 2
    writer = recording.Writer(out)
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    print('記録先: %s' % out)
    print('最初の 10 秒以上は静止させ、その後は姿勢を変えて 4 秒以上置く、を繰り返します。'
          '起こす・倒す・裏返す動きを 3 軸に均等に入れてください。Ctrl-C で終了します。')

    r = dict(batches=0, accel=0, gyro=0, invalid=0, missing_batches=0, epochs=[],
             session=None, saturated=0, poses=0)
    tracker = None
    odr = 6667.0
    next_seq = None
    first_raw = last_raw = None
    stopping_at = None
    stopped = False
    last_print = 0.0
    interactive = sys.stdout.isatty()

    while True:
        now = time.monotonic()
        if stop and stopping_at is None:
            stopping_at = now
            if ip:
                try:
                    board.command(ip, 'imu stop', wait=0.5)
                except board.BoardError as e:
                    print('\n%s' % e, file=sys.stderr)
        if stopping_at is None and args.duration and first_raw and now - first_raw >= args.duration:
            stop.append(1)
            continue
        if stopping_at is not None:
            # 停止後は、残りのバッチと stopped の status が届くのを待つ。
            idle = now - (last_raw or stopping_at)
            if (stopped and idle >= 0.5) or now - stopping_at >= 8.0 or (not ip and idle >= 1.0):
                break
        for topic, _qos, payload in server.poll(0.1):
            if not topic.endswith('/raw'):
                writer.add_message(topic, payload)
                try:
                    body = json.loads(payload)
                except ValueError:
                    continue
                if topic.endswith('/info') and tracker is None:
                    odr = float(body.get('gyro_odr_hz', odr))
                    tracker = PoseTracker(body.get('gyro_udps_per_lsb', 70000) * 1e-6)
                elif topic.endswith('/status') and body.get('state') == 'stopped':
                    stopped = True
                continue
            writer.add_raw(payload)
            try:
                b = packet.parse(payload)
            except packet.PacketError:
                r['invalid'] += 1
                continue
            last_raw = time.monotonic()
            if first_raw is None:
                first_raw = last_raw
            if r['session'] is None:
                r['session'] = b.session.hex()
            if next_seq is not None and b.seq > next_seq:
                r['missing_batches'] += b.seq - next_seq
            next_seq = b.seq + 1
            if b.epoch not in r['epochs']:
                r['epochs'].append(b.epoch)
            r['batches'] += 1
            r['accel'] += b.accel_count
            r['gyro'] += b.gyro_count
            if tracker is None:
                tracker = PoseTracker(0.07)
            a, g = recording.split(b)
            tracker.add(a, g, odr, last_raw)
        if now - last_print >= (1.0 if interactive else 10.0) and first_raw and stopping_at is None:
            last_print = now
            t = tracker
            state = {'init': '最初の静止', 'static': '静止 %4.1f 秒' % t.static_seconds(now),
                     'moving': '移動中'}[t.state]
            line = '%4d 秒  %-12s 姿勢 %2d  回転 X %4.0f Y %4.0f Z %4.0f 度  欠落 %d%s' % (
                now - first_raw, state, t.poses, *t.rotation, r['missing_batches'],
                '  飽和 %d' % t.saturated if t.saturated else '')
            print(('\r\x1b[K' + line) if interactive else line, end='' if interactive else '\n',
                  flush=True)
    if interactive:
        print()
    server.close()
    if tracker:
        r['saturated'], r['poses'] = tracker.saturated, tracker.poses
        r['gyro_max_dps'] = round(tracker.gyro_max_dps, 1)
        r['rotation_deg'] = [round(float(x), 1) for x in tracker.rotation]
    r.update(connects=server.connects, disconnects=server.disconnects,
             protocol_error=server.protocol_error, board=ip, port=args.port,
             seconds=round((last_raw - first_raw), 1) if first_raw else 0.0,
             stopped_status_received=stopped)
    writer.close(r)
    print('記録を終了: %d バッチ（加速度 %d、角速度 %d）、%.1f 秒、欠落 %d、姿勢の目安 %d'
          % (r['batches'], r['accel'], r['gyro'], r['seconds'], r['missing_batches'], r['poses']))
    if not r['batches']:
        print('RAW を受信できませんでした。ボードの Wi-Fi 接続と送信先ポートを確認してください',
              file=sys.stderr)
        return 1
    if r['missing_batches'] or len(r['epochs']) > 1 or r['invalid'] or r['saturated']:
        print('注意: 欠落・飽和などがあります。校正には使えない可能性があります', file=sys.stderr)
    if tracker:
        print('重力の向きを変えた回転の合計: X %.0f、Y %.0f、Z %.0f 度（目安 各 %.0f 以上）'
              % (*tracker.rotation, ROTATION_TARGET_DEG))
        short = ['XYZ'[i] for i in range(3) if tracker.rotation[i] < ROTATION_MIN_DEG]
        if short:
            print('注意: %s 軸まわりの回転が不足しています。角速度の校正には使えない可能性が'
                  'あります' % '・'.join(short), file=sys.stderr)
    print('校正: imu-host calib %s' % out)
    return 0
