# SPDX-License-Identifier: MIT
"""The FreeRTOS build's MQTT transport (freertos/tests/mqtt/live.c) against
the host's receiver, "imu-host check" of host/imu_host, on 127.0.0.1 only.

    live.py <live program> <imu-host> [-v]

Three runs, each with a new receiver on a free port of 127.0.0.1 (never the
board's 18830):

- plain (4 s): every batch arrives, in order, with valid ticks and times; one
  connection; the info once (retained, QoS 1) and the status as the board
  counts it; the topic imu/rp2040/raw at QoS 0; the board's session ID.
- --stall-at 1 --stall-for 2 --rcvbuf 4096 (6 s): the receiver stops
  reading; the board's queue fills (queue_high 12) and the rest is dropped;
  the connection stays; the receiver's missing batches are the board's
  dropped ones, and everything else arrives.
- --drop-at 1 --drop-for 2 (6 s): the receiver drops the connection and
  stops listening for 2 s; the board reconnects (2 connections, the info
  again); the receiver's missing batches are the board's dropped ones plus
  the few that the board's socket had taken when the connection went
  (counted as published: "accepted by the socket, not confirmed delivered",
  common/imu/imu_raw_mqtt.h).
"""
import json
import socket
import subprocess
import sys
import time

SESSION = '503533748a8c92180000000100000000'
BOARD_PORT = 18830


def free_port():
    while True:
        with socket.socket() as s:
            s.bind(('127.0.0.1', 0))
            port = s.getsockname()[1]
        if port != BOARD_PORT:
            return port


def run(live, imu_host, seconds, receiver_args, verbose):
    port = free_port()
    receiver = subprocess.Popen(
        [imu_host, 'check', '--bind', '127.0.0.1', '--port', str(port), '--idle-timeout', '4',
         '--duration', str(seconds + 30)] + receiver_args,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        line = receiver.stdout.readline()
        if line.strip() != 'READY':
            raise SystemExit('receiver: %r' % line)
        board = subprocess.run([live, str(port), str(seconds)], capture_output=True, text=True,
                               timeout=seconds + 30)
        out, _ = receiver.communicate(timeout=seconds + 40)
    finally:
        if receiver.poll() is None:
            receiver.kill()
    if board.returncode != 0:
        raise SystemExit('live: exit %d\n%s%s' % (board.returncode, board.stdout, board.stderr))
    b = json.loads([x for x in board.stdout.splitlines() if x.startswith('BOARD ')][0][6:])
    r = json.loads([x for x in out.splitlines() if x.startswith('RESULT ')][0][7:])
    if verbose:
        print('receiver', ' '.join(receiver_args) or '(plain)')
        print('  board   ', b)
        print('  receiver', {k: r.get(k) for k in (
            'packets', 'first_seq', 'last_seq', 'missing_batches', 'missing_accel',
            'missing_gyro', 'invalid', 'reordered', 'unix_regressions', 'tick_regressions',
            'connects', 'disconnects', 'protocol_error')})
        print('  gaps', r['gaps'])
    return b, r


def expect(cond, what, b, r):
    if not cond:
        raise SystemExit('FAIL: %s\n  board %s\n  receiver %s' % (what, b, json.dumps(r)))


def common(b, r):
    expect(r['invalid'] == 0 and r['reordered'] == 0 and r['index_regressions'] == 0,
           'invalid or reordered batches', b, r)
    expect(r['unix_regressions'] == 0 and r['tick_regressions'] == 0 and
           r['unix_invalid'] == 0 and r['time_invalid'] == 0, 'times', b, r)
    expect(r['unix_valid'] == 2 * r['packets'] and r['time_valid'] == 2 * r['packets'],
           'valid times', b, r)
    expect(r['protocol_error'] is None, 'protocol error', b, r)
    expect(r['sessions'] == [SESSION] and r['epochs'] == [1], 'session', b, r)
    expect(r['topics'] == ['imu/rp2040/raw (QoS 0)'], 'raw topic', b, r)
    expect(r['first_seq'] == 0 and r['last_seq'] == b['batches'] - 1, 'first and last', b, r)
    expect(r['missing_accel'] == b['dropped_accel'] + 190 * (b['published'] - r['packets']) and
           r['missing_gyro'] == b['dropped_gyro'] + 190 * (b['published'] - r['packets']),
           'missing samples', b, r)
    # Each batch is refused by the queue or taken, and each taken one sent
    # or dropped: all are published or dropped.
    expect(b['published'] + b['dropped_batches'] == b['batches'] and
           b['refused'] <= b['dropped_batches'], 'board counts', b, r)
    info = r['other'].get('imu/rp2040/info')
    status = r['other'].get('imu/rp2040/status')
    expect(info and status and status['count'] == b['status_published'] and
           info['count'] == b['info_published'], 'info and status', b, r)
    expect(json.loads(info['last'])['session'] == SESSION and
           json.loads(info['last'])['device'] == 'rp2040', 'info', b, r)
    expect(json.loads(status['last'])['state'] == 'stopped', 'last status', b, r)


def main():
    if len(sys.argv) not in (3, 4):
        raise SystemExit(__doc__)
    live, imu_host = sys.argv[1], sys.argv[2]
    verbose = len(sys.argv) == 4 and sys.argv[3] == '-v'
    started = time.monotonic()

    b, r = run(live, imu_host, 4, [], verbose)
    common(b, r)
    expect(r['packets'] == b['batches'] == b['published'] and r['missing_batches'] == 0 and
           b['dropped_batches'] == 0, 'every batch', b, r)
    expect(b['connects'] == r['connects'] == 1 and b['failures'] == 0, 'one connection', b, r)
    print('ok  plain: %d batches, none missing' % r['packets'])

    b, r = run(live, imu_host, 6, ['--stall-at', '1', '--stall-for', '2', '--rcvbuf', '4096'],
               verbose)
    common(b, r)
    expect(b['dropped_batches'] > 0 and b['queue_high'] == 12, 'queue full', b, r)
    expect(r['missing_batches'] == b['dropped_batches'] and r['packets'] == b['published'],
           'missing = dropped', b, r)
    expect(b['connects'] == r['connects'] == 1, 'the connection stays', b, r)
    print('ok  stall: %d dropped and missing, %d arrived, one connection' %
          (b['dropped_batches'], r['packets']))

    b, r = run(live, imu_host, 6, ['--drop-at', '1', '--drop-for', '2'], verbose)
    common(b, r)
    lost = b['published'] - r['packets']
    expect(b['connects'] == r['connects'] == 2 and b['info_published'] == 2, 'reconnected', b, r)
    expect(0 <= lost <= 13 and r['missing_batches'] == b['dropped_batches'] + lost,
           'missing = dropped + taken by the socket', b, r)
    print('ok  drop: reconnected, %d dropped, %d taken by the socket and lost' %
          (b['dropped_batches'], lost))
    print('live: %.0f s' % (time.monotonic() - started))


if __name__ == '__main__':
    main()
