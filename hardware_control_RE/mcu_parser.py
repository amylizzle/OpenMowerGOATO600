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

def main():
    ap = argparse.ArgumentParser(description="ECOVACS RE MCU frame parser")
    ap.add_argument("file", nargs="?", default="message_dump.bin")
    ap.add_argument("--summary", action="store_true", help="per-command counts only")
    ap.add_argument("--limit", type=int, default=0, help="max frames to print")
    args = ap.parse_args()

    stream = open(args.file, "rb").read()
    frames = parse_stream(stream)
    print("parsed %d frames from %s" % (len(frames), args.file))

    if args.summary:
        from collections import Counter
        c = Counter(f["cmd"] for f in frames)
        for cmd, n in c.most_common():
            print("%-4s %d" % (cmd, n))
        return

    limit = args.limit or len(frames)
    for f in frames[:limit]:
        print("%-4s ack=%d %s" % (f["cmd"], f["ack"], f["data"]))
        if not f.get("unknown"):
            print("    parsed: %r" % f["parsed"])

if __name__ == "__main__":
    main()
