#!/usr/bin/env python3
"""Minimal MQTT 3.1.1 QoS-0 receiver for the experimental IMU probe.

This is a test fixture, not a general MQTT broker or a data recorder.
"""
import argparse
import socket
import struct
import time
import zlib


def read_exact(sock, count):
    data = bytearray()
    while len(data) < count:
        if hasattr(socket, "TCP_QUICKACK"):
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise EOFError("connection closed during MQTT packet")
        data.extend(chunk)
    return bytes(data)


def read_packet(sock):
    first = sock.recv(1)
    if not first:
        return None, b""
    length, multiplier = 0, 1
    for _ in range(4):
        byte = read_exact(sock, 1)[0]
        length += (byte & 127) * multiplier
        if not byte & 128:
            return first[0], read_exact(sock, length)
        multiplier *= 128
    raise ValueError("invalid MQTT remaining length")


def receive(conn):
    header, _ = read_packet(conn)
    if header != 0x10:
        raise ValueError("expected MQTT CONNECT")
    conn.sendall(b"\x20\x02\x00\x00")
    count = accel = gyro = payload_bytes = publishes = 0
    start = None
    last = None
    pending = bytearray()
    checksum = 0
    while True:
        header, body = read_packet(conn)
        if header is None or header >> 4 == 14:
            break
        if header == 0xC0:
            conn.sendall(b"\xd0\x00")
            continue
        if header != 0x30 or len(body) < 2:
            raise ValueError("expected QoS-0 PUBLISH")
        if start is None:
            start = time.monotonic()
        last = time.monotonic()
        publishes += 1
        topic_len = int.from_bytes(body[:2], "big")
        if body[2:2 + topic_len] != b"diagnostic/imu/raw":
            raise ValueError("unexpected topic")
        payload = body[2 + topic_len:]
        checksum = zlib.crc32(payload, checksum)
        pending.extend(payload)
        offset = 0
        while len(pending) - offset >= 8:
            seq, words, bitmap_len = struct.unpack_from("<IHH", pending, offset)
            frame_len = 8 + bitmap_len + 6 * words
            if seq != count:
                raise ValueError(f"sequence gap: expected {count}, received {seq}")
            if bitmap_len != (words + 7) // 8 or words > 160:
                raise ValueError("invalid IMU frame length")
            if frame_len > len(pending) - offset:
                break
            bitmap = pending[offset + 8:offset + 8 + bitmap_len]
            a = sum((bitmap[i // 8] >> (i % 8)) & 1 for i in range(words))
            accel += a
            gyro += words - a
            count += 1
            offset += frame_len
        del pending[:offset]
        payload_bytes += len(payload)
    if pending:
        raise ValueError("stream ended in the middle of an IMU frame")
    elapsed = last - start if start is not None else 0
    print(f"publishes={publishes} frames={count} accel={accel} gyro={gyro} payload_bytes={payload_bytes} "
          f"elapsed_s={elapsed:.3f} payload_kbit_s={payload_bytes * 8 / elapsed / 1000 if elapsed else 0:.1f} "
          f"crc32={checksum:08x}",
          flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=1883)
    args = parser.parse_args()
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.bind, args.port))
        server.listen(1)
        print(f"listening {args.bind}:{args.port}", flush=True)
        conn, peer = server.accept()
        print(f"peer={peer[0]}", flush=True)
        with conn:
            conn.settimeout(30)
            receive(conn)


if __name__ == "__main__":
    main()
