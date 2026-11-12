# SPDX-License-Identifier: Apache-2.0
"""記録の読み書き。

記録は 1 つのディレクトリで、受信したものを加工せずに持つ。

  raw.bin         RAW ペイロードの並び。各ペイロードの前に長さ（u32le）
  messages.jsonl  info / status など RAW 以外のメッセージ。1 行 1 件
  meta.json       記録の開始・終了時刻と受信の集計
"""
import json
import os
import struct
import time
from dataclasses import dataclass, field

import numpy as np

from . import packet

TICK_SECONDS_NOMINAL = 25e-6


class Writer:
    def __init__(self, path):
        os.makedirs(path)
        self.path = path
        self.raw = open(os.path.join(path, 'raw.bin'), 'wb')
        self.messages = open(os.path.join(path, 'messages.jsonl'), 'w')
        self.started = time.time()

    def add_raw(self, payload):
        self.raw.write(struct.pack('<I', len(payload)))
        self.raw.write(payload)

    def add_message(self, topic, payload):
        text = payload.decode('utf-8', errors='replace')
        try:
            body = json.loads(text)
        except ValueError:
            body = text
        self.messages.write(json.dumps(dict(t=time.time(), topic=topic, payload=body)) + '\n')

    def close(self, summary):
        self.raw.close()
        self.messages.close()
        meta = dict(started=self.started, ended=time.time(), summary=summary)
        with open(os.path.join(self.path, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=2)
            f.write('\n')


def split(batch):
    """Batch の RAW を (加速度, 角速度) の int16 配列（各 N×3）に分ける。"""
    words = np.frombuffer(batch.words, dtype='<i2').reshape(-1, 3)
    kind = np.unpackbits(np.frombuffer(batch.bitmap, dtype=np.uint8),
                         bitorder='little')[:batch.count].astype(bool)
    return words[kind], words[~kind]


@dataclass
class Recording:
    path: str
    info: dict
    session: str
    accel: np.ndarray            # N×3、int16
    gyro: np.ndarray
    accel_tick: np.ndarray       # 各サンプルのセンサー tick（最初の tick からの差、float64）
    gyro_tick: np.ndarray
    seconds_per_tick: float
    tick_rate_source: str        # seconds_per_tick の根拠
    batches: int
    problems: list = field(default_factory=list)   # 校正に使えない理由
    warnings: list = field(default_factory=list)

    def seconds(self, ticks):
        return ticks * self.seconds_per_tick


def _raw_payloads(path):
    with open(os.path.join(path, 'raw.bin'), 'rb') as f:
        data = f.read()
    pos = 0
    while pos + 4 <= len(data):
        n = struct.unpack_from('<I', data, pos)[0]
        pos += 4
        if pos + n > len(data):
            break
        yield data[pos:pos + n]
        pos += n


def _messages(path):
    out = []
    with open(os.path.join(path, 'messages.jsonl')) as f:
        for line in f:
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return out


def _sample_ticks(index, anchor_index, anchor_tick):
    """サンプル番号から tick を求める。アンカーの間は補間し、外側は平均の傾きで延ばす。"""
    a_i = np.asarray(anchor_index, dtype=np.float64)
    a_t = np.asarray(anchor_tick, dtype=np.float64)
    slope = (a_t[-1] - a_t[0]) / (a_i[-1] - a_i[0])
    ticks = np.interp(index, a_i, a_t)
    after, before = index > a_i[-1], index < a_i[0]
    ticks[after] = a_t[-1] + (index[after] - a_i[-1]) * slope
    ticks[before] = a_t[0] + (index[before] - a_i[0]) * slope
    return ticks


def _tick_rate(messages, session, unix_points):
    """1 tick の秒数と、その根拠を返す。

    status の対応点（センサーの tick とボードの時計を同時に読んだ値）から求める。
    ボードの時計は水晶で、NTP の同期がなくても使える。対応点が足りなければ
    バッチに付いた PC 時刻、それもなければ公称値を使う。
    """
    points = {}
    for m in messages:
        p = m.get('payload')
        if m.get('topic', '').endswith('/status') and isinstance(p, dict) and \
           'tick' in p and 'tick_mono_us' in p and p.get('session', session) == session:
            points[p['tick']] = p['tick_mono_us']
    for source, pts in (('status', sorted(points.items())), ('batch_unix', unix_points)):
        if len(pts) >= 2 and pts[-1][1] - pts[0][1] >= 5_000_000:
            x = np.array([p[0] - pts[0][0] for p in pts], dtype=np.float64)
            y = np.array([p[1] - pts[0][1] for p in pts], dtype=np.float64)
            return float(np.polyfit(x, y, 1)[0]) * 1e-6, '%s (%d points)' % (source, len(pts))
    return TICK_SECONDS_NOMINAL, 'nominal'


def load(path):
    messages = _messages(path)
    problems, warnings = [], []
    accel, gyro, accel_idx, gyro_idx = [], [], [], []
    anchors = dict(accel=([], []), gyro=([], []))
    unix_points = []
    session = None
    epochs = set()
    next_seq = next_accel = next_gyro = None
    batches = invalid = other_sessions = 0
    for payload in _raw_payloads(path):
        try:
            b = packet.parse(payload)
        except packet.PacketError:
            invalid += 1
            continue
        if session is None:
            session = b.session
        if b.session != session:
            other_sessions += 1
            continue
        if next_seq is not None and b.seq != next_seq:
            problems.append('バッチ %d の前で %d バッチ欠落' % (b.seq, b.seq - next_seq))
        next_seq = b.seq + 1
        epochs.add(b.epoch)
        a, g = split(b)
        for name, data, first, tick, tick_flag, store, store_idx in (
                ('accel', a, b.accel_index, b.accel_tick, packet.FLAG_ACCEL_TICK, accel, accel_idx),
                ('gyro', g, b.gyro_index, b.gyro_tick, packet.FLAG_GYRO_TICK, gyro, gyro_idx)):
            if not len(data):
                continue
            expected = next_accel if name == 'accel' else next_gyro
            if expected is not None and first != expected:
                problems.append('バッチ %d で %s のサンプル番号が不連続（%d → %d）'
                                % (b.seq, name, expected, first))
            if name == 'accel':
                next_accel = first + len(data)
            else:
                next_gyro = first + len(data)
            store.append(data)
            store_idx.append(np.arange(first, first + len(data), dtype=np.float64))
            if b.flags & tick_flag:
                anchors[name][0].append(first)
                anchors[name][1].append(tick)
        if b.flags & packet.FLAG_ACCEL_UNIX:
            unix_points.append((b.accel_tick, b.accel_unix_us))
        batches += 1
    if not batches:
        raise ValueError('%s に有効な RAW バッチがありません' % path)
    if invalid:
        problems.append('仕様に合わないペイロードが %d 件' % invalid)
    if other_sessions:
        warnings.append('別のセッションのバッチ %d 件を無視' % other_sessions)
    if len(epochs) > 1:
        problems.append('epoch が変化（%s）。個数不明の欠落がある' % sorted(epochs))
    if len(problems) > 20:
        problems = problems[:20] + ['…ほか %d 件' % (len(problems) - 20)]
    base = None
    ticks = {}
    for name, idx in (('accel', accel_idx), ('gyro', gyro_idx)):
        a_i, a_t = anchors[name]
        if len(a_i) < 2:
            raise ValueError('%s の tick が付いたバッチが足りません' % name)
        if base is None:
            base = min(anchors['accel'][1][0], anchors['gyro'][1][0])
        ticks[name] = _sample_ticks(np.concatenate(idx), a_i, [t - base for t in a_t])
    info = {}
    for m in messages:
        p = m.get('payload')
        if m.get('topic', '').endswith('/info') and isinstance(p, dict) and \
           p.get('session') == session.hex():
            info = p
    if not info:
        problems.append('このセッションの info がありません（感度と ODR が分からない）')
    seconds_per_tick, source = _tick_rate(messages, session.hex(), unix_points)
    if source == 'nominal':
        warnings.append('tick の速さを実測できず公称値（25 µs）を使用。角速度のスケールに'
                        '最大 1% 程度の誤差が入る')
    return Recording(path=path, info=info, session=session.hex(),
                     accel=np.concatenate(accel), gyro=np.concatenate(gyro),
                     accel_tick=ticks['accel'], gyro_tick=ticks['gyro'],
                     seconds_per_tick=seconds_per_tick, tick_rate_source=source,
                     batches=batches, problems=problems, warnings=warnings)
