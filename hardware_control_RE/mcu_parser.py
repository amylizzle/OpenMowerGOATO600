#!/usr/bin/env python3
"""
ECOVACS RE -- full MCU frame parser (binary stream -> classified + parsed frames).

Reads the raw ttys byte stream (e.g. message_dump.bin), walks frames, validates
framing + CRC8, classifies by (cmd0, cmd1), and parses each payload via the
node classes in mcu_nodes.py.

Wire frame (verified against message_dump.bin, 10030 frames, 0 CRC failures):

    frame = [0x60] [ack] [len] [cmd0] [cmd1] [data...] [crc8] [0x0A]
      len     = data_len + 2            (counts the 2 command bytes)
      crc8    = CRC-8 over (0x60, ack, len, cmd0, cmd1, data...)
                poly 0x07, init 0x00, xorout 0x00
      payload = data[0:]               (starts right after the 2 cmd bytes)

Usage:
    python mcu_parser.py message_dump.bin            # parse whole file, print
    python mcu_parser.py message_dump.bin --summary  # per-command counts
"""
import argparse, sys

from mcu_nodes import NODES

CRC_POLY   = 0x07
CRC_INIT   = 0x00
CRC_XOROUT = 0x00

# (cmd0, cmd1) -> (node key, parse method name)
CLASSIFY = {
    (0x47, 0x44): ("IMU",        "parse_GD"),   # imu/ImuSensor
    (0x57, 0x44): ("WheelNode",  "parse_WD"),   # wheel distance
    (0x4D, 0x53): ("MotorNode",  "parse_MS"),   # motor speed report
    (0x4D, 0x42): ("MotorNode",  "parse_MB"),   # motor current
    (0x57, 0x52): ("WheelNode",  "parse_WR"),   # wheel report
    (0x47, 0x53): ("IMU",        "parse_GS"),   # imu state
    (0x43, 0x43): ("BigData",    "parse_CC"),   # battery status
    (0x5A, 0x52): ("ScreenInfo", "parse_ZR"),   # rain detect
    (0x44, 0x42): ("TemperatureAlert", "parse_DB"),  # alert/alarm
    (0x54, 0x42): ("LogSave",    "parse_TB"),   # log string
    (0x42, 0x43): ("OnOffInfo",  "parse_BC"),   # sensor/GPIO status
    (0x5A, 0x54): ("ScreenInfo", "parse_ZT"),   # screen touch/control
}

def crc8(data: bytes) -> int:
    c = CRC_INIT
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) & 0xFF) ^ (CRC_POLY if c & 0x80 else 0)
    return c ^ CRC_XOROUT

def iter_frames(stream: bytes):
    """Yield (cmd0, cmd1, ack, data, ok, reason) for every 0x60-framed message."""
    i, n = 0, len(stream)
    while i < n - 7:
        if stream[i] != 0x60:
            i += 1
            continue
        ln = stream[i+2]
        end = i + ln + 5                      # 0x60,ack,len + cmd0,cmd1 + data + crc + 0x0A
        if end >= n:
            i += 1
            continue
        if stream[end-1] != 0x0A:
            i += 1
            continue
        ack = stream[i+1]
        c0, c1 = stream[i+3], stream[i+4]
        data = stream[i+5 : i+5+(ln-2)]
        stored = stream[i+5+ln-2]
        calc = crc8(bytes([0x60, ack, ln, c0, c1]) + data)
        yield c0, c1, ack, data, calc == stored, "crc" if calc != stored else ""
        i = end

def parse_frame(c0, c1, ack, data):
    """Classify + parse one frame. Returns dict with cmd/ack/parsed, or
    {'cmd':..., 'ack':..., 'unknown':True} for unclassified commands."""
    key = (c0, c1)
    cmd = chr(c0) + chr(c1) if (0x20 <= c0 < 0x7f and 0x20 <= c1 < 0x7f) else "?"
    if key in CLASSIFY:
        node, meth = CLASSIFY[key]
        fn = getattr(NODES[node], meth)
        parsed = fn(data, ack)
        return {"cmd": cmd, "ack": ack, "data": data.hex(), "parsed": parsed}
    return {"cmd": cmd, "ack": ack, "data": data.hex(), "unknown": True}

def parse_stream(stream: bytes, validate=True):
    """Parse all frames. Returns list of dicts. If validate, raises on CRC bad."""
    out = []
    for c0, c1, ack, data, ok, reason in iter_frames(stream):
        if not ok:
            raise ValueError("bad crc frame cmd=%s%c%c" % (hex(c0), c1, c1))
        out.append(parse_frame(c0, c1, ack, data))
    return out

import time
import os
class MCULink:
    def __init__(self, path, baud):
        self.path = path
        self.baud = baud
        self.ser = None

    def open(self):
        import termios
        self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attr = termios.tcgetattr(self.fd)
        # attr = [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]

        B = {9600: termios.B9600, 57600: termios.B57600,
             115200: termios.B115200}.get(self.baud, termios.B115200)

        attr[0] = 0        # iflag: raw
        attr[1] = 0        # oflag: raw
        attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag: 8N1, no parity
        attr[3] = 0        # lflag: raw mode
        attr[4] = B        # ispeed
        attr[5] = B        # ospeed
        attr[6][termios.VMIN] = 0
        attr[6][termios.VTIME] = 0

        termios.tcsetattr(self.fd, termios.TCSANOW, attr)
        return self

    def write(self, data: bytes):
        os.write(self.fd, data)

    def read_frame(self, timeout=1.0, poll=0.05):
        """Read one full framed (0x60..0x0A) message; returns (cmd, data) or None."""
        buf = bytearray()
        end = time.time() + timeout
        while time.time() < end:
            try:
                chunk = os.read(self.fd, 256)
                if chunk:
                    buf.extend(chunk)
            except BlockingIOError:
                pass
            while True:
                i = buf.find(b"\x60")
                if i < 0:
                    buf.clear()  # no delimiter at all left; nothing to keep
                    break
                if len(buf) - i < 3:
                    break
                ln = buf[i+2]
                total = ln + 5
                if len(buf) - i < total:
                    break
                frame = bytes(buf[i:i+total])

                # Validate BEFORE trusting this as a real frame.
                terminator_ok = frame[-1] == 0x0A
                check = crc8(frame[:-2])
                crc_ok = check == frame[-2]

                if terminator_ok and crc_ok:
                    del buf[:i+total]
                    return frame[3:5], frame[5:5+ln-2], frame

                # False positive: this 0x60 wasn't a real header.
                # Only drop the one leading byte so we don't destroy
                # a genuine frame that may start later in buf.
                print(f"desync/bad frame at offset {i} "
                      f"(terminator_ok={terminator_ok}, crc_ok={crc_ok}, "
                      f"expected_crc={check}, got={frame[-2]}) - resyncing")
                del buf[:i+1]
                # loop again to find the next 0x60 candidate
            time.sleep(poll)
        return None

def main():
    ap = argparse.ArgumentParser(description="ECOVACS RE MCU frame parser")
    ap.add_argument("file", nargs="?", default="message_dump.bin")
    args = ap.parse_args()

    link = MCULink(args.file, 115200)
    link.open()
    last = {}
    try:
        while True:
            got = link.read_frame(timeout=1.0)
            if got is not None:
                cmd, data, rawframe = got
                print(f"TYPE:{str(cmd)} ACK:{rawframe[1]} LEN:{rawframe[2]}") 
                parsed = parse_frame(cmd[0],cmd[1],0,data)
                if parsed is not None:
                    print(parsed)
                else:
                    print(f"Unknown message type {cmd}: {rawframe.hex()}")
            else:
                # no new frame; let user know about timeout
                print("Nothin...")
    finally:
        os.close(link.fd)
        
if __name__ == "__main__":
    main()
