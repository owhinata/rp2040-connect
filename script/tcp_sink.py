#!/usr/bin/env python3
"""Count bytes and elapsed time for one incoming TCP connection."""

import argparse
import socket
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0", help="local IPv4 address")
    parser.add_argument("--port", type=int, default=5001, help="TCP port")
    args = parser.parse_args()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((args.bind, args.port))
        listener.listen(1)
        print(f"Listening on {args.bind}:{args.port}", flush=True)
        conn, peer = listener.accept()
        with conn:
            start = time.monotonic()
            total = 0
            while True:
                data = conn.recv(8192)
                if not data:
                    break
                total += len(data)
            elapsed = time.monotonic() - start
    rate = total * 8 / elapsed / 1000 if elapsed else 0
    print(f"peer={peer[0]} bytes={total} elapsed_s={elapsed:.3f} rate_kbit_s={rate:.1f}")


if __name__ == "__main__":
    main()
