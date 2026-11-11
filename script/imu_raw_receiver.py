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
"""
import argparse
import json
import select
import signal
import socket
import struct
import sys
import time

HEADER = 80
MAX_WORDS = 384
MAX_GAP_RECORDS = 50


class Stats:
    def __init__(self, session):
        self.session = session
        self.r = dict(
            packets=0, accel=0, gyro=0, bytes=0, first_seq=None, last_seq=None,
            missing_batches=0, missing_accel=0, missing_gyro=0, reordered=0,
            index_regressions=0, invalid=0, connects=0, disconnects=0,
            max_gap_ms=0.0, max_gap_before_seq=None, protocol_error=None)
        self.gaps = []
        self.epoch_changes = []
        self.last_epoch = None
        self.sessions, self.topics, self.flags, self.epochs = set(), set(), set(), set()
        self.other = {}
        self.next_accel = self.next_gyro = 0
        self.last_time = None

    def other_message(self, topic, payload):
        entry = self.other.setdefault(topic, dict(count=0, last=None))
        entry['count'] += 1
        entry['last'] = payload[:2048].decode('utf-8', errors='replace')

    def raw(self, topic, qos, p):
        r = self.r
        self.topics.add('%s (QoS %d)' % (topic, qos))
        if len(p) < HEADER or p[:4] != b'IMU1' or p[4] != 1 or p[5] & ~3 or \
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
        if self.last_epoch is not None and epoch != self.last_epoch and \
           len(self.epoch_changes) < MAX_GAP_RECORDS:
            # 番号が連続していても、epoch の境界では個数不明の欠落がある。
            self.epoch_changes.append(dict(before_seq=seq, epoch=epoch, gap_ms=gap_ms))
        self.last_epoch = epoch
        self.epochs.add(epoch)

    def summary(self):
        out = dict(self.r)
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


def handle(conn, stats, header, body, raw_suffix):
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
        else:
            stats.other_message(topic, body[offset:])
    elif kind == 8:
        count = max(0, len(body) - 2) and 1
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
    ap.add_argument('--rcvbuf', type=int, help='SO_RCVBUF。小さくすると読み出し停止が送信側へ早く伝わる')
    args = ap.parse_args()
    session = bytes.fromhex(args.session) if args.session else None
    if session is not None and len(session) != 16:
        ap.error('--session は 32 桁の hex')
    stats = Stats(session)
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
        try:
            ready, _, _ = select.select(readable, [], [], 0.1)
        except InterruptedError:
            continue
        if listener in ready:
            new, _ = listener.accept()
            close_conn()
            conn = new
        if conn and conn in ready:
            try:
                data = conn.recv(65536)
                if not data:
                    close_conn()
                    continue
                buf += data
                for header, body in parse_packets(buf):
                    if not handle(conn, stats, header, body, '/raw'):
                        close_conn()
                        break
            except (OSError, ValueError) as e:
                stats.r['protocol_error'] = '%s: %s' % (type(e).__name__, e)
                close_conn()
    if conn:
        conn.close()
    if listener:
        listener.close()
    print('RESULT ' + json.dumps(stats.summary()), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
