#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""IMU1 RAW バッチの診断用受信器。

MQTT 3.1.1 のクライアント 1 台を直接受け付け、`imu/<device_id>/raw` の
IMU1 ペイロードを検査して件数・連番・サンプル番号の欠落を集計する。
汎用ブローカーでもデータ保存アプリでもない。`script/imu_mqtt_probe.py`
（旧診断フォーマット用）とは互換性がない。

終了時（SIGINT/SIGTERM、--duration、--idle-timeout）に集計を JSON で 1 行出力する。
試験用に、受信開始から指定秒後に接続を切って待ち受けも止める --drop-at/--drop-for と、
読み出しだけを止める --stall-at/--stall-for を備える。

--sync-interval を指定すると、ボードが購読した `imu/<id>/sync/request` へ一定間隔で
要求を送り、`sync/response` の t2/t3 と PC 側の t1/t4 から往復時間を集計する。
これは経路の遅延の実測用で、往復の非対称は分からないため時刻精度の証明にはならない。
"""
import argparse
import json
import select
import signal
import socket
import struct
import sys
import time

HEADER = 96
VERSION = 2
MAX_WORDS = 384
MAX_GAP_RECORDS = 50


def percentiles(values):
    if not values:
        return None
    v = sorted(values)
    pick = lambda q: round(v[min(len(v) - 1, int(q * len(v)))], 3)
    return dict(n=len(v), min=round(v[0], 3), p50=pick(0.5), p90=pick(0.9), p99=pick(0.99),
                max=round(v[-1], 3))


def residuals(samples):
    """|offset - 直線近似| (ms)。samples は (t1_s, offset_ms)。"""
    n = len(samples)
    if n < 3:
        return None
    mx = sum(s[0] for s in samples) / n
    my = sum(s[1] for s in samples) / n
    sxx = sum((s[0] - mx) ** 2 for s in samples)
    slope = sum((s[0] - mx) * (s[1] - my) for s in samples) / sxx if sxx else 0.0
    out = percentiles([abs(s[1] - my - slope * (s[0] - mx)) for s in samples])
    out['drift_ppm'] = round(slope * 1000, 1)
    return out


class Sync:
    """PC が t1 で要求を送り、ボードの t2/t3、PC の t4 から往復時間を求める。"""

    def __init__(self):
        self.topic = None
        self.sent = {}
        self.next_id = 1
        self.samples = []  # (t1_s, rtt_ms, board_ms, offset_ms)
        self.unmatched = 0

    def send(self, conn):
        topic = self.topic.encode()
        payload = str(self.next_id).encode()
        body = len(topic).to_bytes(2, 'big') + topic + payload
        self.sent[self.next_id] = time.monotonic_ns()
        self.next_id += 1
        conn.sendall(b'\x30' + bytes([len(body)]) + body)

    def response(self, payload, t4_ns):
        try:
            m = json.loads(payload)
            t1_ns = self.sent.pop(int(m['id']))
            t2, t3 = int(m['t2_us']), int(m['t3_us'])
        except (ValueError, KeyError, TypeError):
            self.unmatched += 1
            return
        board_ms = (t3 - t2) / 1000
        rtt_ms = (t4_ns - t1_ns) / 1e6 - board_ms
        # ボード時刻 - PC 時刻。経路が対称であると仮定した値。
        offset_ms = ((t2 * 1000 - t1_ns) + (t3 * 1000 - t4_ns)) / 2e6
        self.samples.append((t1_ns / 1e9, rtt_ms, board_ms, offset_ms))

    def summary(self):
        s = self.samples
        best = sorted(s, key=lambda x: x[1])[:max(3, len(s) // 4)]
        return dict(
            sent=self.next_id - 1, answered=len(s), unanswered=len(self.sent),
            unmatched=self.unmatched,
            round_trip_ms=percentiles([x[1] for x in s]),
            board_hold_ms=percentiles([x[2] for x in s]),
            offset_residual_ms=residuals([(x[0], x[3]) for x in s]),
            offset_residual_ms_best_quarter=residuals([(x[0], x[3]) for x in best]),
            best_quarter_round_trip_max_ms=round(max(x[1] for x in best), 3) if best else None)


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
            index_regressions=0, invalid=0, connects=0, disconnects=0,
            max_gap_ms=0.0, max_gap_before_seq=None, protocol_error=None,
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
        if len(p) < HEADER or p[:4] != b'IMU1' or p[4] != VERSION or p[5] & ~15 or \
           struct.unpack_from('<H', p, 6)[0] != HEADER or p[76:80] != b'\0' * 4:
            r['invalid'] += 1
            return
        seq, ia, ig = struct.unpack_from('<QQQ', p, 24)
        count, bitmap_len = struct.unpack_from('<HH', p, 64)
        if not 1 <= count <= MAX_WORDS or bitmap_len != (count + 7) // 8 or \
           len(p) != HEADER + bitmap_len + count * 6 or \
           (count % 8 and p[HEADER + bitmap_len - 1] >> (count % 8)):
            r['invalid'] += 1
            return
        session = p[8:24]
        if self.session is None:
            self.session = session
        if session != self.session:
            self.sessions.add(session.hex())
            r['invalid'] += 1
            return
        self.sessions.add(session.hex())
        accel = sum(b.bit_count() for b in p[HEADER:HEADER + bitmap_len])
        gyro = count - accel
        none = 0xFFFFFFFFFFFFFFFF
        if (accel == 0) != (ia == none) or (gyro == 0) != (ig == none):
            r['invalid'] += 1
            return
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
        self.flags.add(p[5])
        epoch = struct.unpack_from('<I', p, 68)[0]
        ticks = struct.unpack_from('<QQ', p, 48)
        if accel and p[5] & 1:
            self.time_points.batches.append((ticks[0], arrival))
        unix = struct.unpack_from('<QQ', p, 80)
        for kind, bit, present, index, value, tick in (('accel', 4, accel, ia, unix[0], ticks[0]),
                                                       ('gyro', 8, gyro, ig, unix[1], ticks[1])):
            if not present:
                continue
            if not p[5] & bit:
                r['unix_invalid'] += 1
                if value:
                    r['invalid'] += 1
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
            if not p[5] & bit:
                r['time_invalid'] += 1
                if tick:
                    r['invalid'] += 1
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


def parse_packets(buf):
    """Yield (header, body) for each complete MQTT control packet in buf."""
    pos = 0
    while True:
        if len(buf) - pos < 2:
            break
        length = shift = 0
        i = pos + 1
        complete = False
        while i < len(buf) and i < pos + 5:
            length |= (buf[i] & 0x7F) << shift
            shift += 7
            i += 1
            if not buf[i - 1] & 0x80:
                complete = True
                break
        if not complete:
            if i >= pos + 5:
                raise ValueError('malformed remaining length')
            break
        if len(buf) - i < length:
            break
        yield buf[pos], bytes(buf[i:i + length])
        pos = i + length
    del buf[:pos]


def handle(conn, stats, header, body, raw_suffix, sync=None, t4_ns=0):
    kind = header >> 4
    if kind == 1:
        conn.sendall(b'\x20\x02\x00\x00')
        stats.r['connects'] += 1
    elif kind == 3:
        qos = (header >> 1) & 3
        n = int.from_bytes(body[:2], 'big')
        topic = body[2:2 + n].decode('utf-8', errors='replace')
        offset = 2 + n
        if qos:
            conn.sendall(b'\x40\x02' + body[offset:offset + 2])
            offset += 2
        if topic.endswith(raw_suffix):
            stats.raw(topic, qos, body[offset:])
        elif sync is not None and topic.endswith('/sync/response'):
            sync.response(body[offset:], t4_ns)
        else:
            stats.other_message(topic, body[offset:])
    elif kind == 8:
        pos, count = 2, 0
        while pos + 2 <= len(body):
            n = int.from_bytes(body[pos:pos + 2], 'big')
            topic = body[pos + 2:pos + 2 + n].decode('utf-8', errors='replace')
            if sync is not None and topic.endswith('/sync/request'):
                sync.topic = topic
            pos += 2 + n + 1
            count += 1
        conn.sendall(b'\x90' + bytes([2 + count]) + body[:2] + b'\x00' * count)
    elif kind == 12:
        conn.sendall(b'\xd0\x00')
    elif kind == 14:
        return False
    else:
        raise ValueError('unexpected MQTT packet type %d' % kind)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bind', required=True, help='待ち受ける PC の IPv4 アドレス')
    ap.add_argument('--port', type=int, default=18830)
    ap.add_argument('--session', help='期待する 32 桁 hex のセッション ID（省略時は最初の値）')
    ap.add_argument('--duration', type=float, help='起動からこの秒数で終了')
    ap.add_argument('--idle-timeout', type=float, help='RAW 受信後、この秒数受信がなければ終了')
    ap.add_argument('--drop-at', type=float, help='最初の RAW からこの秒数後に切断し待ち受けも停止')
    ap.add_argument('--drop-for', type=float, default=5.0)
    ap.add_argument('--stall-at', type=float, help='最初の RAW からこの秒数後に読み出しを停止')
    ap.add_argument('--stall-for', type=float, default=5.0)
    ap.add_argument('--sync-interval', type=float, help='同期要求を送る間隔（秒）')
    ap.add_argument('--rcvbuf', type=int, help='SO_RCVBUF。小さくすると読み出し停止が送信側へ早く伝わる')
    args = ap.parse_args()
    session = bytes.fromhex(args.session) if args.session else None
    if session is not None and len(session) != 16:
        ap.error('--session は 32 桁の hex')
    stats = Stats(session)
    sync = Sync() if args.sync_interval else None
    next_sync = 0.0
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))

    def listen():
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if args.rcvbuf:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, args.rcvbuf)
        s.bind((args.bind, args.port))
        s.listen(1)
        return s

    def close_conn():
        nonlocal conn
        if conn:
            conn.close()
            conn = None
            buf.clear()
            stats.r['disconnects'] += 1
            if sync:
                sync.topic = None
                sync.sent.clear()

    listener, conn, buf = listen(), None, bytearray()
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
            close_conn()
            listener.close()
            listener = None
            drop_done = True
            drop_until = now + args.drop_for
            print('DROP', flush=True)
        if drop_until is not None and now >= drop_until:
            listener = listen()
            drop_until = None
            print('LISTEN', flush=True)
        if args.stall_at is not None and since is not None and \
           args.stall_at <= since < args.stall_at + args.stall_for:
            time.sleep(0.05)
            continue
        readable = [s for s in (listener, conn) if s]
        if not readable:
            time.sleep(0.05)
            continue
        wait = 0.1
        if sync and conn and sync.topic:
            if now >= next_sync:
                try:
                    sync.send(conn)
                except OSError:
                    close_conn()
                    continue
                next_sync = now + args.sync_interval
            wait = max(0.0, min(wait, next_sync - now))
        try:
            ready, _, _ = select.select(readable, [], [], wait)
        except InterruptedError:
            continue
        if listener in ready:
            new, _ = listener.accept()
            close_conn()
            conn = new
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        if conn and conn in ready:
            try:
                data = conn.recv(65536)
                t4_ns = time.monotonic_ns()
                if not data:
                    close_conn()
                    continue
                buf += data
                for header, body in parse_packets(buf):
                    if not handle(conn, stats, header, body, '/raw', sync, t4_ns):
                        close_conn()
                        break
            except (OSError, ValueError) as e:
                stats.r['protocol_error'] = '%s: %s' % (type(e).__name__, e)
                close_conn()
    if conn:
        conn.close()
    if listener:
        listener.close()
    result = stats.summary()
    if sync:
        result['sync'] = sync.summary()
    print('RESULT ' + json.dumps(result), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
