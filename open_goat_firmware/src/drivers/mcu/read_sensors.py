#!/usr/bin/env python3
"""
ECOVACS RE -- read the current state of each sensor from the MCU over UART.

Reverse-engineered from the stripped aarch64 node (see decomp/findings.md §6b).

Sensor/GPIO state arrives as a "BC" frame on /dev/ttyS3 (OnOffInfo receive @ 0x9545a8,
which checks cmd0='B'=0x42, cmd1='C'=0x43). Wire framing is the same MA-style frame:

    frame = [0x60] [ack] [len] [cmd0] [cmd1] [data...] [crc8] [0x0A]
      len  = data_len + 2
      crc8 = getStrCrc8 over (0x60, ack, len, cmd0, cmd1, data...) i.e. 5+data_len bytes

"BC" payload is a list of [idx, state] byte pairs starting at data offset 6
(frame offset 11). idx maps to a hardware channel (OnOffInfo parse + idx map 0xb6ded0):

    0  = bump        (bumper)
    2  = fall        (cliff/fall sensor)
    4  = chargeState (charger contacts)
    6  = acczero     (accelerometer zero-cross)
    8  = rain        (rain sensor)
    10 = grass       (grass sensor)
    12 = roll        (roll/tilt)
    14 = Stop        (estop)
    16 = fan

NOTE / best-effort assumptions (adjust on real hardware):
  * CRC8 polynomial lives in external libEcoLogger.so (getStrCrc8), NOT in this
    binary. CRC_* below are the most likely candidate (CRC-8/SMBUS: poly 0x1D,
    init 0xFF, no reflect/no xorout). If frames are rejected, change CRC_POLY/
    CRC_INIT/CRC_XOROUT. Use --crc-hex to dump the exact table we build.
  * If your MCU omits the 0x60/ack/len/crc framing and just sends cmd0 cmd1 data,
    pass --bare so cmd0='B' cmd1='C' and pairs start at offset 8.
"""
import argparse, os, struct, sys, time

UART = "/dev/ttyS3"
BAUD = 115200


# --- CRC8 (getStrCrc8, from external libEcoLogger.so -- best-effort) ---
CRC_POLY   = 0x07
CRC_INIT   = 0x00
CRC_XOROUT = 0x00

def _crc_table():
    t = []
    for i in range(256):
        v = i
        for _ in range(8):
            v = ((v << 1) ^ CRC_POLY) & 0xFF if (v & 0x80) else ((v << 1) & 0xFF)
        t.append(v)
    return t

_CRCT = _crc_table()

def crc8(data: bytes) -> int:
    crc = CRC_INIT
    for b in data:
        crc = _CRCT[(crc ^ b) & 0xFF]
    return crc ^ CRC_XOROUT

class MCULink:
    def __init__(self, path=UART, baud=BAUD):
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

CMD_BC = b"BC"   # sensor/GPIO status feedback

# idx -> (name, unit)
SENSOR_MAP = {
    0:  "bump",
    2:  "fall",
    4:  "chargeState",
    6:  "acczero",
    8:  "rain",
    10: "grass",
    12: "roll",
    14: "Stop",
    16: "fan",
}

CHARGE_FORMAT = ('<BxHhbxB',[
    "chargeStep", 
    "chargeVol",
    "chargeCur",
    "batteryTemp",
    "batteryLevel",
    ]
)

MESSAGE_TYPES = {
    b"BC": SENSOR_MAP,
    b"CC": CHARGE_FORMAT,
}

def parse_message(cmd:bytes, data: bytes):
    """Return {idx: state} parsed from the BC payload."""
    if cmd not in MESSAGE_TYPES:
        return None
    else:
        map = MESSAGE_TYPES[cmd]
    print(cmd)
    if isinstance(map, dict):
        out = {k:"?" for k in map.values()}
        i=0
        while i + 1 < len(data):
            idx = data[i]
            state = data[i+1]
            if idx not in map:
                break
            out[map[idx]] = state
            i += 2
    else:
        datatuple = struct.unpack(map[0], data[:struct.calcsize(map[0])])
        out = {k:datatuple[i] for i,k in enumerate(map[1])}
    return out


def format_state(pairs: dict):
    lines = []
    for idx, name in sorted(SENSOR_MAP.items()):
        s = pairs.get(idx, "n/a")
        lines.append("%-12s idx=%-3d %s" % (name, idx, s))
    return "\n".join(lines)

def main():
    ap = argparse.ArgumentParser(description="GOAT MCU sensor status reader")
    ap.add_argument("--duration", type=float, default=-1.0, help="seconds to keep reading (default: forever)")
    ap.add_argument("--interval", type=float, default=0.2, help="print interval (s)")
    ap.add_argument("--crc-hex", action="store_true", help="print CRC table and exit")
    args = ap.parse_args()

    if args.crc_hex:
        print("CRC poly=0x%02X init=0x%02X xor=0x%02X table:" % (CRC_POLY, CRC_INIT, CRC_XOROUT))
        for i in range(0, 256, 16):
            print(" ".join("%02X" % v for v in _CRCT[i:i+16]))
        return

    link = MCULink(UART, BAUD)
    link.open()
    last = {}
    end = time.time() + args.duration
    try:
        while True:
            got = link.read_frame(timeout=1.0)
            if got is not None:
                cmd, data, rawframe = got
                print(f"TYPE:{str(cmd)} ACK:{rawframe[1]} LEN:{rawframe[2]}") 
                parsed = parse_message(cmd,data)
                if parsed is not None:
                    print(parsed)
                else:
                    print(f"Unknown message type {cmd}: {rawframe.hex()}")
            else:
                # no new frame; let user know about timeout
                print("Nothin...")
            if args.duration > 0 and time.time() > end:
                break
    finally:
        os.close(link.fd)

if __name__ == "__main__":
    main()
