# SPDX-License-Identifier: Apache-2.0
"""IMU1 バージョン 2 のペイロード（設計書 §6）を検査して取り出す。"""
import struct
from dataclasses import dataclass

HEADER_SIZE = 96
VERSION = 2
MAX_WORDS = 384
NO_INDEX = 0xFFFFFFFFFFFFFFFF

FLAG_ACCEL_TICK = 1
FLAG_GYRO_TICK = 2
FLAG_ACCEL_UNIX = 4
FLAG_GYRO_UNIX = 8


class PacketError(ValueError):
    """ペイロードが仕様に合わない。"""


@dataclass
class Batch:
    flags: int
    session: bytes
    seq: int
    accel_index: int   # このバッチに加速度がなければ NO_INDEX
    gyro_index: int
    accel_tick: int    # 有効かどうかは flags で判定する
    gyro_tick: int
    accel_unix_us: int
    gyro_unix_us: int
    epoch: int
    count: int         # ワード数
    accel_count: int
    bitmap: bytes      # ワード i の種類。ビットが 1 なら加速度
    words: bytes       # count × 6 バイト。各ワードは int16le の x, y, z

    @property
    def gyro_count(self):
        return self.count - self.accel_count


def parse(p):
    """ペイロードを検査して Batch を返す。合わなければ PacketError。"""
    if len(p) < HEADER_SIZE or p[:4] != b'IMU1':
        raise PacketError('magic')
    if p[4] != VERSION:
        raise PacketError('version %d' % p[4])
    flags = p[5]
    if flags & ~15 or (flags & FLAG_ACCEL_UNIX and not flags & FLAG_ACCEL_TICK) or \
       (flags & FLAG_GYRO_UNIX and not flags & FLAG_GYRO_TICK):
        raise PacketError('flags 0x%02x' % flags)
    if struct.unpack_from('<H', p, 6)[0] != HEADER_SIZE or p[76:80] != b'\0' * 4:
        raise PacketError('header')
    seq, accel_index, gyro_index, accel_tick, gyro_tick = struct.unpack_from('<QQQQQ', p, 24)
    count, bitmap_len, epoch = struct.unpack_from('<HHI', p, 64)
    accel_unix, gyro_unix = struct.unpack_from('<QQ', p, 80)
    if not 1 <= count <= MAX_WORDS or bitmap_len != (count + 7) // 8 or \
       len(p) != HEADER_SIZE + bitmap_len + count * 6:
        raise PacketError('length')
    bitmap = bytes(p[HEADER_SIZE:HEADER_SIZE + bitmap_len])
    if count % 8 and bitmap[-1] >> (count % 8):
        raise PacketError('bitmap padding')
    accel_count = sum(b.bit_count() for b in bitmap)
    if (accel_count == 0) != (accel_index == NO_INDEX) or \
       (accel_count == count) != (gyro_index == NO_INDEX):
        raise PacketError('first index')
    for bit, value in ((FLAG_ACCEL_TICK, accel_tick), (FLAG_GYRO_TICK, gyro_tick),
                       (FLAG_ACCEL_UNIX, accel_unix), (FLAG_GYRO_UNIX, gyro_unix)):
        if not flags & bit and value:
            raise PacketError('value without its flag')
    return Batch(flags=flags, session=bytes(p[8:24]), seq=seq,
                 accel_index=accel_index, gyro_index=gyro_index,
                 accel_tick=accel_tick, gyro_tick=gyro_tick,
                 accel_unix_us=accel_unix, gyro_unix_us=gyro_unix,
                 epoch=epoch, count=count, accel_count=accel_count,
                 bitmap=bitmap, words=bytes(p[HEADER_SIZE + bitmap_len:]))
