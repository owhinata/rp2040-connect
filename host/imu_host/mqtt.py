# SPDX-License-Identifier: Apache-2.0
"""ボード 1 台を直接受け付ける最小限の MQTT 3.1.1 の受け口。

汎用ブローカーではない。CONNECT、PUBLISH（QoS 0/1）、SUBSCRIBE、PINGREQ、
DISCONNECT に応答し、届いた PUBLISH を呼び出し側へ渡すだけである。
"""
import select
import socket


def _packets(buf):
    """buf に揃っている MQTT パケットを (先頭バイト, 本体) で順に返し、buf から除く。"""
    pos = 0
    while len(buf) - pos >= 2:
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


class Server:
    def __init__(self, bind, port, rcvbuf=None):
        self.bind, self.port, self.rcvbuf = bind, port, rcvbuf
        self.listener = None
        self.conn = None
        self.peer = None            # 接続中、または最後に接続していた相手の IP
        self.buf = bytearray()
        self.connects = self.disconnects = 0
        self.protocol_error = None
        self.listen()

    def listen(self):
        if self.listener:
            return
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if self.rcvbuf:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.rcvbuf)
        s.bind((self.bind, self.port))
        s.listen(1)
        self.listener = s

    def stop_listening(self):
        if self.listener:
            self.listener.close()
            self.listener = None

    def drop_client(self):
        if self.conn:
            self.conn.close()
            self.conn = None
            self.buf.clear()
            self.disconnects += 1

    def close(self):
        if self.conn:
            self.conn.close()
            self.conn = None
        self.stop_listening()

    def poll(self, timeout):
        """timeout 秒まで待ち、届いた PUBLISH を (topic, qos, payload) の一覧で返す。"""
        readable = [s for s in (self.listener, self.conn) if s]
        if not readable:
            return []
        try:
            ready, _, _ = select.select(readable, [], [], timeout)
        except InterruptedError:
            return []
        if self.listener in ready:
            new, addr = self.listener.accept()
            self.drop_client()
            self.conn = new
            self.peer = addr[0]
            new.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        out = []
        if self.conn and self.conn in ready:
            try:
                data = self.conn.recv(65536)
                if not data:
                    self.drop_client()
                    return out
                self.buf += data
                for header, body in _packets(self.buf):
                    if not self._handle(header, body, out):
                        self.drop_client()
                        break
            except (OSError, ValueError) as e:
                self.protocol_error = '%s: %s' % (type(e).__name__, e)
                self.drop_client()
        return out

    def _handle(self, header, body, out):
        kind = header >> 4
        if kind == 1:
            self.conn.sendall(b'\x20\x02\x00\x00')
            self.connects += 1
        elif kind == 3:
            qos = (header >> 1) & 3
            n = int.from_bytes(body[:2], 'big')
            topic = body[2:2 + n].decode('utf-8', errors='replace')
            offset = 2 + n
            if qos:
                self.conn.sendall(b'\x40\x02' + body[offset:offset + 2])
                offset += 2
            out.append((topic, qos, body[offset:]))
        elif kind == 8:
            pos, count = 2, 0
            while pos + 2 <= len(body):
                pos += 2 + int.from_bytes(body[pos:pos + 2], 'big') + 1
                count += 1
            self.conn.sendall(b'\x90' + bytes([2 + count]) + body[:2] + b'\x00' * count)
        elif kind == 12:
            self.conn.sendall(b'\xd0\x00')
        elif kind == 14:
            return False
        else:
            raise ValueError('unexpected MQTT packet type %d' % kind)
        return True
