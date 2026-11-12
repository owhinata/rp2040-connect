# SPDX-License-Identifier: Apache-2.0
"""RAW バッチの受信を診断する。

`imu/<device_id>/raw` の IMU1 ペイロードを検査し、件数・連番・サンプル番号の欠落と
時刻の整合を集計する。保存はしない（保存は record）。

終了時（SIGINT/SIGTERM、--duration、--idle-timeout）に集計を JSON で 1 行出力する。
試験用に、受信開始から指定秒後に接続を切って待ち受けも止める --drop-at/--drop-for と、
読み出しだけを止める --stall-at/--stall-for を備える。
"""
import json
import signal
import time

from . import mqtt, packet

MAX_GAP_RECORDS = 50


def percentiles(values):
    if not values:
        return None
    v = sorted(values)
    pick = lambda q: round(v[min(len(v) - 1, int(q * len(v)))], 3)
    return dict(n=len(v), min=round(v[0], 3), p50=pick(0.5), p90=pick(0.9), p99=pick(0.99),
                max=round(v[-1], 3))


class TimePoints:
    """status の (tick, unix_us) とバッチの先頭 tick・到着時刻から時刻対応を検証する。

    tick から PC 時刻への変換は、対応点全体への直線近似で行う。これは検証用の
    最も単純な方法で、専用アプリでの変換方法を決めるものではない。
    """

    def __init__(self):
        self.points = []   # (tick, unix_us, unix_estimate_us, window_us, delay_us)
        self.batches = []  # (first accel tick, arrival unix seconds)
        self.unsynchronized = 0

    def status(self, payload):
        try:
            m = json.loads(payload)
        except ValueError:
            return
        if 'tick' not in m:
            return
        if 'unix_us' not in m:
            self.unsynchronized += 1
            return
        p = (m['tick'], m['unix_us'], m['unix_estimate_us'], m['tick_window_us'], m['sync_delay_us'])
        if not self.points or p[0] != self.points[-1][0]:
            self.points.append(p)

    def compare(self, tick_unix):
        pts = self.points
        if len(pts) < 3 or not tick_unix:
            return None
        t0, u0 = pts[0][0], pts[0][1]
        xs = [p[0] - t0 for p in pts]
        ys = [p[1] - u0 for p in pts]
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
        intercept = my - slope * mx
        return percentiles([(u - u0 - (slope * (t - t0) + intercept)) / 1000 for t, u in tick_unix])

    def summary(self):
        pts = self.points
        out = dict(points=len(pts), unsynchronized=self.unsynchronized)
        if len(pts) < 3:
            return out
        t0, u0 = pts[0][0], pts[0][1]
        xs = [p[0] - t0 for p in pts]

        def fit(ys):
            n = len(xs)
            mx, my = sum(xs) / n, sum(ys) / n
            slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
            return slope, my - slope * mx

        slope, intercept = fit([p[1] - u0 for p in pts])
        slope_e, intercept_e = fit([p[2] - u0 for p in pts])
        out.update(
            span_s=round((pts[-1][1] - u0) / 1e6, 1),
            us_per_tick=round(slope, 6), ticks_per_second=round(1e6 / slope, 2),
            # 逆行: tick が進んだのに変換後の時刻が進んでいない点の数
            unix_not_increasing=sum(b[1] <= a[1] for a, b in zip(pts, pts[1:])),
            unix_estimate_not_increasing=sum(b[2] <= a[2] for a, b in zip(pts, pts[1:])),
            fit_residual_ms=percentiles([abs(p[1] - u0 - (slope * x + intercept)) / 1000
                                         for p, x in zip(pts, xs)]),
            fit_residual_estimate_ms=percentiles([abs(p[2] - u0 - (slope_e * x + intercept_e)) / 1000
                                                  for p, x in zip(pts, xs)]),
            tick_window_us=percentiles([p[3] for p in pts]),
            sync_delay_ms=percentiles([p[4] / 1000 for p in pts]))
        if self.batches:
            # 先頭サンプルの時刻から到着までの時間。バッチの長さ（約 29 ms）より
            # 短ければ、どこかの対応が間違っている。
            out['batch_first_sample_to_arrival_ms'] = percentiles(
                [(arrival * 1e6 - (u0 + slope * (tick - t0) + intercept)) / 1000
                 for tick, arrival in self.batches])
        return out


class Stats:
    def __init__(self, session):
        self.session = session
        self.r = dict(
            packets=0, accel=0, gyro=0, bytes=0, first_seq=None, last_seq=None,
            missing_batches=0, missing_accel=0, missing_gyro=0, reordered=0,
            index_regressions=0, invalid=0,
            max_gap_ms=0.0, max_gap_before_seq=None,
            time_valid=0, time_invalid=0, tick_regressions=0,
            unix_valid=0, unix_invalid=0, unix_regressions=0)
        self.gaps = []
        self.epoch_changes = []
        self.last_epoch = None
        # センサー時刻: 直前バッチの (種類, 先頭番号, 先頭 tick) と 1 サンプルあたりの tick 数
        self.tick_prev = {}
        self.ticks_per_sample = {}
        self.tick_first = self.tick_last = None
        self.time_points = TimePoints()
        # バッチに付いた PC 時刻: 直前バッチの (epoch, 先頭番号, 時刻) と集計
        self.unix_prev = {}
        self.unix_us_per_sample = []
        self.unix_to_arrival_ms = []
        self.unix_vs_points = []  # (tick, unix_us) 対応点の直線との比較用
        self.sessions, self.topics, self.flags, self.epochs = set(), set(), set(), set()
        self.other = {}
        self.next_accel = self.next_gyro = 0
        self.last_time = None

    def other_message(self, topic, payload):
        if topic.endswith('/status'):
            self.time_points.status(payload)
        entry = self.other.setdefault(topic, dict(count=0, last=None))
        entry['count'] += 1
        entry['last'] = payload[:2048].decode('utf-8', errors='replace')

    def raw(self, topic, qos, p):
        r = self.r
        self.topics.add('%s (QoS %d)' % (topic, qos))
        try:
            b = packet.parse(p)
        except packet.PacketError:
            r['invalid'] += 1
            return
        if self.session is None:
            self.session = b.session
        self.sessions.add(b.session.hex())
        if b.session != self.session:
            r['invalid'] += 1
            return
        seq, ia, ig = b.seq, b.accel_index, b.gyro_index
        accel, gyro = b.accel_count, b.gyro_count
        now = time.monotonic()
        arrival = time.time()
        gap_ms = 0.0 if self.last_time is None else round((now - self.last_time) * 1000, 1)
        expected = 0 if r['last_seq'] is None else r['last_seq'] + 1
        if seq < expected:
            r['reordered'] += 1
            return
        missing_a = missing_g = 0
        if accel:
            if ia < self.next_accel:
                r['index_regressions'] += 1
            missing_a = max(0, ia - self.next_accel)
            self.next_accel = ia + accel
        if gyro:
            if ig < self.next_gyro:
                r['index_regressions'] += 1
            missing_g = max(0, ig - self.next_gyro)
            self.next_gyro = ig + gyro
        if seq > expected or missing_a or missing_g:
            if len(self.gaps) < MAX_GAP_RECORDS:
                self.gaps.append(dict(before_seq=seq, batches=seq - expected,
                                      accel=missing_a, gyro=missing_g, gap_ms=gap_ms))
            r['missing_batches'] += seq - expected
            r['missing_accel'] += missing_a
            r['missing_gyro'] += missing_g
        if r['first_seq'] is None:
            r['first_seq'] = seq
        r['last_seq'] = seq
        if gap_ms > r['max_gap_ms']:
            r['max_gap_ms'], r['max_gap_before_seq'] = gap_ms, seq
        self.last_time = now
        r['packets'] += 1
        r['accel'] += accel
        r['gyro'] += gyro
        r['bytes'] += len(p)
        self.flags.add(b.flags)
        epoch = b.epoch
        ticks = (b.accel_tick, b.gyro_tick)
        if accel and b.flags & 1:
            self.time_points.batches.append((ticks[0], arrival))
        unix = (b.accel_unix_us, b.gyro_unix_us)
        for kind, bit, present, index, value, tick in (('accel', 4, accel, ia, unix[0], ticks[0]),
                                                       ('gyro', 8, gyro, ig, unix[1], ticks[1])):
            if not present:
                continue
            if not b.flags & bit:
                r['unix_invalid'] += 1
                continue
            r['unix_valid'] += 1
            prev = self.unix_prev.get(kind)
            if prev and index > prev[1]:
                if value <= prev[2]:
                    r['unix_regressions'] += 1
                elif prev[0] == epoch:
                    self.unix_us_per_sample.append((value - prev[2]) / (index - prev[1]))
            self.unix_prev[kind] = (epoch, index, value)
            if kind == 'accel':
                self.unix_to_arrival_ms.append((arrival * 1e6 - value) / 1000)
                self.unix_vs_points.append((tick, value))
        for kind, bit, present, index, tick in (('accel', 1, accel, ia, ticks[0]),
                                                ('gyro', 2, gyro, ig, ticks[1])):
            if not present:
                continue
            if not b.flags & bit:
                r['time_invalid'] += 1
                continue
            r['time_valid'] += 1
            prev = self.tick_prev.get(kind)
            if prev and prev[0] == epoch and index > prev[1]:
                if tick <= prev[2]:
                    r['tick_regressions'] += 1
                else:
                    # 整数にならない比は分数のまま残し、まとめて表示する。
                    ratio = (tick - prev[2]) / (index - prev[1])
                    name = '%s:%g' % (kind, ratio)
                    if name in self.ticks_per_sample or len(self.ticks_per_sample) < 20:
                        self.ticks_per_sample[name] = self.ticks_per_sample.get(name, 0) + 1
            self.tick_prev[kind] = (epoch, index, tick)
            if self.tick_first is None:
                self.tick_first = (tick, now)
            if gap_ms < 200:  # 停止時などに遅れて届いたバッチは到着時刻の基準にしない
                self.tick_last = (tick, now)
        if self.last_epoch is not None and epoch != self.last_epoch and \
           len(self.epoch_changes) < MAX_GAP_RECORDS:
            # 番号が連続していても、epoch の境界では個数不明の欠落がある。
            self.epoch_changes.append(dict(before_seq=seq, epoch=epoch, gap_ms=gap_ms))
        self.last_epoch = epoch
        self.epochs.add(epoch)

    def summary(self):
        out = dict(self.r)
        if self.tick_first and self.tick_last[1] > self.tick_first[1]:
            # 到着時刻との粗い比較。受信の遅延やばらつきを含むため精度の検証にはならない。
            out['ticks_per_second_by_arrival'] = round(
                (self.tick_last[0] - self.tick_first[0]) / (self.tick_last[1] - self.tick_first[1]), 1)
        out['ticks_per_sample'] = self.ticks_per_sample
        out['time_points'] = self.time_points.summary()
        out['batch_unix'] = dict(
            # 連続するバッチの先頭時刻の差を番号の差で割った値（µs）。一定に近いほどよい。
            us_per_sample=percentiles(self.unix_us_per_sample),
            first_sample_to_arrival_ms=percentiles(self.unix_to_arrival_ms),
            # ボードが付けた時刻と、status の対応点から PC 側で求めた直線との差
            minus_point_fit_ms=self.time_points.compare(self.unix_vs_points))
        out.update(gaps=self.gaps, epoch_changes=self.epoch_changes, sessions=sorted(self.sessions), topics=sorted(self.topics),
                   flags=sorted(self.flags), epochs=sorted(self.epochs), other=self.other)
        return out


def run(args):
    session = bytes.fromhex(args.session) if args.session else None
    if session is not None and len(session) != 16:
        raise SystemExit('--session は 32 桁の hex')
    stats = Stats(session)
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    server = mqtt.Server(args.bind, args.port, rcvbuf=args.rcvbuf)
    started = time.monotonic()
    first_raw = None
    drop_done = args.drop_at is None
    drop_until = None
    print('READY', flush=True)
    while not stop:
        now = time.monotonic()
        if first_raw is None and stats.r['packets']:
            first_raw = now
        if args.duration is not None and now - started >= args.duration:
            break
        if args.idle_timeout is not None and stats.last_time is not None and \
           now - stats.last_time >= args.idle_timeout and drop_until is None:
            break
        since = None if first_raw is None else now - first_raw
        if not drop_done and since is not None and since >= args.drop_at:
            server.drop_client()
            server.stop_listening()
            drop_done = True
            drop_until = now + args.drop_for
            print('DROP', flush=True)
        if drop_until is not None and now >= drop_until:
            server.listen()
            drop_until = None
            print('LISTEN', flush=True)
        if (args.stall_at is not None and since is not None and
                args.stall_at <= since < args.stall_at + args.stall_for) or \
           not (server.listener or server.conn):
            time.sleep(0.05)
            continue
        for topic, qos, payload in server.poll(0.1):
            if topic.endswith('/raw'):
                stats.raw(topic, qos, payload)
            else:
                stats.other_message(topic, payload)
    server.close()
    out = stats.summary()
    out.update(connects=server.connects, disconnects=server.disconnects,
               protocol_error=server.protocol_error)
    print('RESULT ' + json.dumps(out), flush=True)
    return 0
