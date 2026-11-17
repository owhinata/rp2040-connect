#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The client side of the host test of the Telnet shell and "zperf"
(freertos/tests/telnet/main.c is the board): starts the board program with a
free port, talks to it over TCP on 127.0.0.1 and checks what comes back,
including the board's log lines.  With the host's iperf3 (if there is one)
it runs "zperf tcp upload" against "iperf3 -s" and "iperf3 -c" against
"zperf tcp download".

Usage: client.py <telnet_test program> [-v]
"""

import json
import os
import queue
import shutil
import socket
import subprocess
import sys
import threading
import time

PROMPT = b"t$ "
VERBOSE = "-v" in sys.argv


def log(*args):
    if VERBOSE:
        print(*args, flush=True)


class Failure(Exception):
    pass


def check(cond, what):
    if not cond:
        raise Failure(what)


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Board:
    """The board program and the lines it prints."""

    def __init__(self, program, port):
        self.port = port
        self.proc = subprocess.Popen([program, str(port)], stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT)
        self.lines = queue.Queue()
        self.all = []
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for raw in self.proc.stdout:
            line = raw.decode(errors="replace").rstrip("\n")
            log("board:", line)
            self.all.append(line)
            self.lines.put(line)

    def wait_line(self, text, timeout=10.0):
        """Waits for a line that contains TEXT (looking at those not seen yet)."""
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            check(left > 0, "no board line with %r; got %r" % (text, self.all[-10:]))
            try:
                line = self.lines.get(timeout=left)
            except queue.Empty:
                continue
            if text in line:
                return line

    def drain(self):
        while True:
            try:
                self.lines.get_nowait()
            except queue.Empty:
                return


class Client:
    def __init__(self, port, rcvbuf=None):
        self.sock = socket.socket()
        if rcvbuf:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        self.sock.connect(("127.0.0.1", port))
        self.buf = b""

    def send(self, data):
        self.sock.sendall(data)

    def read_until(self, token, timeout=5.0):
        """Everything up to and including TOKEN; b"" if the board closed first."""
        end = time.monotonic() + timeout
        while token not in self.buf:
            left = end - time.monotonic()
            check(left > 0, "timeout waiting for %r; got %r" % (token, self.buf[-200:]))
            self.sock.settimeout(left)
            try:
                data = self.sock.recv(4096)
            except socket.timeout:
                continue
            except ConnectionResetError:
                data = b""
            if not data:
                return None
            self.buf += data
        i = self.buf.index(token) + len(token)
        out, self.buf = self.buf[:i], self.buf[i:]
        return out

    def command(self, line, timeout=10.0):
        """Sends LINE with CR LF; the output between the line end and the prompt."""
        self.send(line.encode() + b"\r\n")
        out = self.read_until(PROMPT, timeout)
        check(out is not None, "closed during %r" % line)
        check(out.startswith(b"\r\n"), "no line end first for %r: %r" % (line, out[:40]))
        return out[2:-len(PROMPT)]

    def nothing_for(self, seconds):
        self.sock.settimeout(seconds)
        try:
            data = self.sock.recv(4096)
        except socket.timeout:
            return True
        return data == b"" and False

    def wait_closed(self, timeout=5.0):
        """True when the board closes the connection within TIMEOUT."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.sock.settimeout(max(0.01, end - time.monotonic()))
            try:
                data = self.sock.recv(65536)
            except socket.timeout:
                continue
            except ConnectionResetError:
                return True
            if not data:
                return True
        return False

    def close(self):
        self.sock.close()


def lines_text(n):
    return b"".join(b"line %d\r\n" % i for i in range(1, n + 1))


def connect_session(board, tries=100, rcvbuf=None, exact=True):
    """A connection that the board takes (one may be refused while it has
    not yet seen the previous one go)."""
    refused = 0
    for _ in range(tries):
        c = Client(board.port, rcvbuf)
        c.send(b"lines 1\r\n")
        out = c.read_until(PROMPT, 5.0)
        if out is not None:
            if exact:
                check(out == b"\r\nline 1\r\n" + PROMPT, "first command: %r" % out)
            return c, refused
        c.close()
        refused += 1
        time.sleep(0.05)
    raise Failure("the board refused %d connections in a row" % tries)


def test_protocol(board):
    c = Client(board.port)
    # Nothing on connecting, no option negotiation.
    check(c.nothing_for(0.3), "the board sent something on connecting")
    # Telnet commands are taken out and not answered: DO ECHO, WILL SGA,
    # DO SGA, WILL NAWS, a two-byte NOP, an IAC inside a word.
    c.send(b"\xff\xfd\x01\xff\xfb\x03\xff\xfd\x03\xff\xfb\x1f\xff\xf1he\xff\xf1lp\r\n")
    out = c.read_until(PROMPT)
    check(out is not None and b"\xff" not in out, "IAC answered or kept: %r" % out)
    check(out.startswith(b"\r\nAvailable commands:\r\n"), "help: %r" % out[:60])
    check(b"  zperf" in out and out.endswith(b"\r\n" + PROMPT), "help: %r" % out)
    # Line ends: CR NUL, CR CR (two empty lines), LF, LF CR (one).
    c.send(b"lines 2\r\x00")
    check(c.read_until(PROMPT) == b"\r\nline 1\r\nline 2\r\n" + PROMPT, "CR NUL")
    c.send(b"\r\r")
    check(c.read_until(PROMPT) == b"\r\n" + PROMPT, "first empty line")
    check(c.read_until(PROMPT) == b"\r\n" + PROMPT, "second empty line")
    c.send(b"lines 1\n")
    check(c.read_until(PROMPT) == b"\r\nline 1\r\n" + PROMPT, "LF")
    c.send(b"lines 1\n\r")
    check(c.read_until(PROMPT) == b"\r\nline 1\r\n" + PROMPT, "LF CR")
    # Zephyr's process_nl(): after LF the CRs are skipped until another character.
    c.send(b"\r\r")
    check(c.nothing_for(0.3), "LF CR ended two lines, or CR after LF CR ended one")
    # A line in pieces, and a command split across reads.
    c.send(b"lin")
    time.sleep(0.1)
    c.send(b"es 3")
    time.sleep(0.1)
    c.send(b"\r")
    time.sleep(0.1)
    c.send(b"\n")
    check(c.read_until(PROMPT) == b"\r\n" + lines_text(3) + PROMPT, "pieces")
    check(c.nothing_for(0.3), "CR and LF in two reads ended two lines")
    # Echo off: Ctrl-C, Backspace, Delete, Tab, the arrows and bytes above 127
    # are dropped (the client edits the line itself).
    c.send(b"li\x03n\x08e\x7fs\t\x1b[A\x1b[D\x80\xfe 2\r\n")
    check(c.read_until(PROMPT) == b"\r\n" + lines_text(2) + PROMPT, "control characters")
    # A long output.
    out = c.command("lines 500")
    check(out == lines_text(500), "lines 500")
    # Unknown command, help of a command.
    check(c.command("nope") == b"nope: command not found\r\n", "unknown command")
    out = c.command("zperf version")
    check(out == b"Version: 1.1\r\nProtocol: iperf3\r\nConfig: unified wifi ipv4\r\n",
          "zperf version: %r" % out)
    # A subnegotiation ends the connection.
    c.send(b"\xff\xfa\x18")
    check(c.wait_closed(), "IAC SB did not end the connection")
    c.close()


def test_second_client(board):
    c, _ = connect_session(board)
    board.drain()
    d = Client(board.port)
    check(d.wait_closed(3.0), "the second client was not closed")
    board.wait_line("LOG shell_telnet: Telnet client already connected.")
    check(d.buf == b"", "the second client got %r" % d.buf)
    d.close()
    check(c.command("lines 2") == lines_text(2), "the first client after the second")
    c.close()


def test_reconnect(board):
    refused = 0
    t0 = time.monotonic()
    # 100 sessions, each closed at once after its prompt.
    for _ in range(100):
        c, r = connect_session(board)
        refused += r
        c.close()
    # 20 connections closed at once, with nothing sent (more would wait on
    # the host's SYN retries: the backlog is 1, as on the board).
    for _ in range(20):
        Client(board.port).close()
    # 20 more sessions, one second apart would be slow: 0.2 s apart.
    for _ in range(20):
        c, r = connect_session(board)
        refused += r
        c.close()
        time.sleep(0.2)
    c, r = connect_session(board)
    refused += r
    out = c.command("socks")
    check(out == b"socks 2\r\n", "descriptors left open: %r" % out)
    c.close()
    log("reconnect: %.1f s, %d refused (the previous one not seen yet)"
        % (time.monotonic() - t0, refused))
    return refused


def test_mixing(board):
    """The USB session runs "lines 40" all along; the Telnet session's long
    output must not get into it, nor its into this one."""
    c, _ = connect_session(board)
    for _ in range(5):
        check(c.command("lines 300") == lines_text(300), "lines 300 with the USB session")
    out = c.command("stats")
    check(b"usb_bad=0" in out, "the USB session saw other output: %r" % out)
    c.close()


def test_stall(board):
    """A client that stops reading: the session's send gives up after 10 s
    and the board ends the connection; another task printing to the session
    is held up 100 ms at most, the 10 ms task stays on time."""
    c, _ = connect_session(board)
    c.command("stats")
    c.command("async on")
    c.close()
    s, _ = connect_session(board, rcvbuf=4096, exact=False)
    board.drain()
    t0 = time.monotonic()
    s.send(b"lines 100000\r\n")
    board.wait_line("LOG shell_telnet: Failed to send 11, shutting down", timeout=30.0)
    took = time.monotonic() - t0
    board.wait_line("LOG shell_telnet: Error -11, shutting down")
    check(s.wait_closed(10.0), "the board kept the stalled connection")
    s.close()
    log("stall: shut down after %.1f s" % took)
    check(8.0 < took < 20.0, "stall ended after %.1f s" % took)
    d, _ = connect_session(board, exact=False)
    # The stalled "lines 100000" may still be running when this client
    # comes: the rest of its output and its prompt come first, then the
    # answer to "lines 1".  The async lines come in between.
    d.send(b"async off\r\nsocks\r\n")
    while True:
        out = d.read_until(PROMPT, 30.0)
        check(out is not None, "closed after the stall")
        if b"\r\nsocks " in out:
            break
    out = d.command("stats")
    line = [x for x in out.decode().split("\r\n") if x.startswith("stats ")]
    check(len(line) == 1, "stats: %r" % out)
    fields = dict(f.split("=") for f in line[0].split()[1:])
    log("stall:", fields)
    check(int(fields["async_max_ms"]) < 250, "another task's print waited: %r" % out)
    check(int(fields["late_max_ms"]) < 50, "the 10 ms task was held up: %r" % out)
    check(fields["usb_bad"] == "0", "USB session: %r" % out)
    d.close()


def test_netdown(board):
    c, _ = connect_session(board)
    board.drain()
    c.command("netdown")
    check(c.wait_closed(3.0), "the failed connection was not ended")
    line = board.wait_line("LOG shell_telnet: Telnet socket ")
    check(line.endswith(" error (0)"), line)
    c.close()
    d, _ = connect_session(board)
    check(d.command("socks") == b"socks 2\r\n", "descriptors after the drop")
    d.close()


def run_iperf3(args, timeout=30):
    p = subprocess.run(["iperf3"] + args, capture_output=True, timeout=timeout)
    log("iperf3", " ".join(args), "->", p.returncode)
    try:
        return p.returncode, json.loads(p.stdout.decode() or "{}"), p
    except json.JSONDecodeError:
        return p.returncode, {}, p


def test_zperf(board):
    c, _ = connect_session(board)

    # Board -> PC: "iperf3 -s" for one test.
    port = free_port()
    server = subprocess.Popen(["iperf3", "-s", "-1", "-p", str(port), "-J"],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(0.5)
    out = c.command("zperf tcp upload 127.0.0.1 %d 2 1024" % port, timeout=30.0)
    so, se = server.communicate(timeout=20)
    log(out.decode())
    head = (b"Remote port is %d\r\nConnecting to 127.0.0.1\r\nDuration:\t2.00 s\r\n"
            b"Packet size:\t1024 bytes\r\nRate:\t\t10 Kbps\r\nStarting...\r\n"
            b"-\r\nUpload completed!\r\nDuration:\t\t") % port
    check(out.startswith(head), "zperf tcp upload: %r" % out)
    lines = out.decode().split("\r\n")
    packets = int(lines[9].split("\t")[-1])
    check(lines[10] == "Num errors:\t\t0 (retry or fail)", lines[10])
    check(lines[11].startswith("Rate:\t\t\t") and lines[11].endswith("bps"), lines[11])
    check(server.returncode == 0, "iperf3 -s: %r" % se)
    result = json.loads(so.decode())
    received = result["end"]["sum_received"]["bytes"]
    # iperf3 stops reading at TEST_END: what is still on the way is not counted.
    check(0 < received <= packets * 1024 and packets * 1024 - received < 1024 * 1024,
          "iperf3 received %d bytes, the board sent %d blocks" % (received, packets))
    check(result["start"]["test_start"]["blksize"] == 1024, "block size")

    # An upload to a port with no server.
    out = c.command("zperf tcp upload 127.0.0.1 %d 1 1024" % free_port())
    check(out.endswith(b"TCP upload failed (-111)\r\n"), "upload without server: %r" % out)
    out = c.command("zperf tcp upload 300.1.2.3 5201")
    check(out == b"Remote port is 5201\r\nInvalid IPv4 address 300.1.2.3\r\n"
          b"Please specify the IP address of the remote server.\r\n", repr(out))
    out = c.command("zperf tcp upload -a 127.0.0.1")
    check(out == b"Unrecognized argument: -a\r\n", repr(out))

    # PC -> board: "zperf tcp download".
    port = free_port()
    out = c.command("zperf tcp download %d" % port)
    check(out == b"TCP server started on port %d\r\n" % port, repr(out))
    check(c.command("zperf tcp download %d" % port) == b"TCP server already started!\r\n",
          "second start")
    for _ in range(2):
        rc, result, p = run_iperf3(["-c", "127.0.0.1", "-p", str(port), "-t", "2", "-J"])
        check(rc == 0, "iperf3 -c: %r %r" % (p.stdout[-300:], p.stderr))
        sent = result["end"]["sum_sent"]["bytes"]
        received = result["end"]["sum_received"]["bytes"]
        check(0 < received <= sent, "iperf3 -c: sent %d, the board got %d" % (sent, received))
        report = c.read_until(b" rate:\t\t\t", 10.0) + c.read_until(b"\r\n", 2.0)
        check(b"New TCP session started.\r\nTCP session ended\r\n Duration:\t\t2." in report,
              "download report: %r" % report)
    # Refused: reverse, more than one stream; a client while a test runs.
    rc, result, p = run_iperf3(["-c", "127.0.0.1", "-p", str(port), "-t", "1", "-R", "-J"])
    check(rc != 0 and "not implemented" in json.dumps(result).lower() + p.stderr.decode().lower()
          or rc != 0, "iperf3 -R was taken")
    rc, result, p = run_iperf3(["-c", "127.0.0.1", "-p", str(port), "-t", "1", "-P", "2", "-J"])
    check(rc != 0, "iperf3 -P 2 was taken")
    first = subprocess.Popen(["iperf3", "-c", "127.0.0.1", "-p", str(port), "-t", "3"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)
    rc, result, p = run_iperf3(["-c", "127.0.0.1", "-p", str(port), "-t", "1", "-J"])
    check(rc != 0 and "busy" in (json.dumps(result) + p.stderr.decode()).lower(),
          "a second client during a test: %r" % result.get("error"))
    check(first.wait(timeout=20) == 0, "the first client during the second")
    c.read_until(b" rate:\t\t\t", 10.0)
    c.read_until(b"\r\n", 2.0)
    out = c.command("zperf tcp download stop")
    check(out == b"TCP server stopped\r\n", repr(out))
    check(c.command("zperf tcp download stop") == b"TCP server not running!\r\n", "stop again")
    out = c.command("zperf tcp download %d 192.168.0.1" % port)
    check(out == b"Failed to start TCP server!\r\n", "bind to an address: %r" % out)
    check(c.command("socks") == b"socks 2\r\n", "descriptors after zperf")
    c.close()


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    board = Board(sys.argv[1], free_port())
    try:
        board.wait_line("READY")
        line = board.all[0] if board.all else ""
        check(any("Telnet shell backend initialized" in x for x in board.all),
              "no start log: %r" % line)
        test_protocol(board)
        test_second_client(board)
        refused = test_reconnect(board)
        test_mixing(board)
        test_netdown(board)
        test_stall(board)
        if shutil.which("iperf3"):
            test_zperf(board)
            zperf = "zperf with iperf3"
        else:
            zperf = "zperf skipped: no iperf3"
        c, _ = connect_session(board)
        c.send(b"quit\r\n")
        board.wait_line("RESULT ok")
        board.proc.wait(timeout=5)
        print("telnet: %d connections refused while the previous one was not yet seen; %s"
              % (refused, zperf))
        return 0
    except Failure as e:
        print("FAIL:", e)
        if VERBOSE:
            import traceback
            traceback.print_exc()
        print("\n".join(board.all[-30:]))
        return 1
    finally:
        if board.proc.poll() is None:
            board.proc.kill()


if __name__ == "__main__":
    sys.exit(main())
