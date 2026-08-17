#!/usr/bin/env python3
"""
ECOVACS RE -- MCU message node classes (pure parse + write, NO serial handling).

Reverse-engineered from the stripped aarch64 ROS1 node (see decomp/findings.md,
decomp/imu_mcu_msg_spec.md, decomp/on_sa_recv.c).

One class per MCU node (the 12 registered receive nodes from findings ##7).
Each class exposes:
  * parse_<cmd>(self, data, ack)  -> dict   # RECEIVE: parse raw data bytes + ACK byte
  * write_<cmd>(self, ...params)  -> bytes  # SEND: build the data payload only

No serial I/O happens here. The caller is responsible for framing: the full wire
frame is  [0x60][ack][len=data+2][cmd0][cmd1][data...][crc8][0x0A] (total data+7),
crc8 = getStrCrc8 over (0x60, ack, len, cmd0, cmd1, data...). The CRC lives in
the external libEcoLogger.so (candidate CRC-8/SMBUS poly 0x1D, init 0xFF, xor 0
-- see motor_control.py / sensor_status.py). Command id bytes are given per
class as CMD_<cmd> = (main, sub).

PAYLOAD EXTRACTION (which bytes to pass as `data` to parse_*):
  Pass `data` = the bytes immediately after the 2 command bytes (offset 0).
  Verified against message_dump.bin (10030 frames): every receive command's
  payload starts at data[0]. Field offsets below are 0-based relative to data.
"""
import struct

def _u16(b, o):  return struct.unpack_from("<H", b, o)[0]
def _s16(b, o):  return struct.unpack_from("<h", b, o)[0]
def _u32(b, o):  return struct.unpack_from("<I", b, o)[0]
def _s32(b, o):  return struct.unpack_from("<i", b, o)[0]
def _u8(b, o):   return b[o]
def _s8(b, o):
    v = b[o]
    return v - 256 if v & 0x80 else v


class MotorNode:
    """MotorNode -- ids MB,MA,ME,MD,MF,MS,MC,MT (callback 0x931c0c)."""
    CMD = {"MB": (0x4D, 0x42), "MA": (0x4D, 0x41), "ME": (0x4D, 0x45),
           "MD": (0x4D, 0x44), "MF": (0x4D, 0x46), "MS": (0x4D, 0x53),
           "MC": (0x4D, 0x43), "MT": (0x4D, 0x54)}

    # ---- receive (data = bytes after cmd, offset 0) ----
    def parse_MA(self, data, ack):
        """MA control ack. Payload = list of 4-byte {type, dir/brand, int16 value}."""
        groups = []
        for i in range(0, len(data) - (len(data) % 4), 4):
            groups.append({"type": data[i], "brand": data[i+1],
                           "value": _s16(data, i+2)})
        return {"groups": groups, "ack": ack}

    def parse_MB(self, data, ack):
        """MB motor current: 5 x int16 per-motor values -> MotorCurrent."""
        vals = [_s16(data, i) for i in range(0, min(len(data) - 1, 10), 2)]
        return {"currents": vals, "ack": ack}

    def parse_MC(self, data, ack):
        """MC current value: byte@2 (data), motor type 7."""
        return {"value": data[2] if len(data) > 2 else 0, "motor_type": 7, "ack": ack}

    def parse_MD(self, data, ack):
        """MD state report: flag{0,1,2}->state, type=11."""
        flag = data[0] if data else 0
        return {"state": flag, "type": 11, "ack": ack}

    def parse_ME(self, data, ack):
        """ME motor type: choice{1,2,10,4,5,6}->type."""
        m = {1: 1, 2: 2, 10: 0, 4: 5, 5: 6, 6: 4}
        c = data[0] if data else 0
        return {"motor_type": m.get(c, c), "ack": ack}

    def parse_MF(self, data, ack):
        """MF motor warning: {4,5,10}->warning."""
        w = {4: 5, 5: 6, 10: 0}
        c = data[0] if data else 0
        return {"warning": w.get(c, c), "ack": ack}

    def parse_MS(self, data, ack):
        """MS speed report: data[0]=type (==10 to decode), int16@1 RPM1,
        @3 RPM2, @5 RPM3; if len>14 also int16@7,@9 and bytes@11..14."""
        r = {}
        if len(data) > 1:
            r["motor_type"] = data[0]
            if data[0] == 10:
                r["rpm1"] = _s16(data, 1)
                r["rpm2"] = _s16(data, 3)
                r["rpm3"] = _s16(data, 5)
                if len(data) > 14:
                    r["rpm4"] = _s16(data, 7)
                    r["rpm5"] = _s16(data, 9)
                    r["aux"] = list(data[11:15])
        r["ack"] = ack
        return r

    def parse_MT(self, data, ack):
        """MT big-data machine info: 4 chars."""
        return {"info": data[:4].decode("latin1", "replace") if data else "", "ack": ack}

    # ---- send ----
    def write_MA(self, controls):
        """controls = list of 4-byte wire control values. Returns payload bytes."""
        return b"".join(controls)

    def write_ma_speed(self, brand, speed):
        """Build a single 4-byte SET-SPEED wire control (type 0x0A)."""
        import struct as _s
        from motor_control import brand_encode  # ROL brand encoding
        dir_ = 0 if speed >= 0 else 1
        enc = brand_encode(brand, abs(speed)) & 0xFFFF
        return bytes([0x0A, dir_]) + _s.pack("<H", enc)

    def write_ma_enable(self, motor_type):
        """Build a single 4-byte ENABLE wire control (type 0x0B)."""
        return bytes([0x0B, 0x01, motor_type & 0xFF, 0x00])

    def write_ma_stop(self):
        """Build a single 4-byte STOP wire control (type 0x02)."""
        return bytes([0x02, 0x00, 0x00, 0x00])


class OnOffInfo:
    """OnOffInfo -- id BC (callback 0x9545a8). Payload region starts at data+6."""
    CMD = {"BC": (0x42, 0x43), "DO": (0x44, 0x4F), "JA": (0x4A, 0x41)}
    SENSOR_MAP = {0: "bump", 2: "fall", 4: "chargeState", 6: "acczero",
                  8: "rain", 10: "grass", 12: "roll", 14: "Stop", 16: "fan"}

    def parse_BC(self, data, ack):
        """BC sensor/GPIO status: [idx,state] byte pairs from payload offset 0."""
        pairs = {}
        for i in range(0, len(data) - 1, 2):
            idx, state = data[i], data[i+1]
            if idx not in self.SENSOR_MAP:
                break
            pairs[self.SENSOR_MAP[idx]] = state
            pairs["_%d" % idx] = state
        pairs["ack"] = ack
        return pairs

    def write_DO(self):
        """DO: 2-byte payload [0x00, 0x01] (enable/on)."""
        return bytes([0x00, 0x01])

    def write_JA(self, value):
        """JA: u16 value (LE) sensor/status report."""
        return struct.pack("<H", value & 0xFFFF)


class BigData:
    """BigData -- ids CB,CC,CH,LP (callback 0x9abb60). Payload region at data+6."""
    CMD = {"CC": (0x43, 0x43)}

    def parse_CC(self, data, ack):
        """CC battery status: batteryID u8@0, chargeVol u16@2, chargeCur s16@4,
        temp s8@6, soc u8@8; +5x u16@9 if len>0x13."""
        r = {}
        if len(data) > 8:
            r["batteryID"] = data[0]
            r["chargeVol"] = _u16(data, 2)     # mV
            r["chargeCur"] = _s16(data, 4)     # mA
            r["temp"] = _s8(data, 6)
            r["soc"] = data[8]                  # %
            if len(data) > 0x13:
                r["ext"] = [_u16(data, 9 + 2*k) for k in range(5)]
        r["ack"] = ack
        return r

    # CB/CH/LP layouts not yet mapped -- raw passthrough.
    def parse_CB(self, data, ack):  return {"raw": data, "ack": ack}
    def parse_CH(self, data, ack):  return {"raw": data, "ack": ack}
    def parse_LP(self, data, ack):  return {"raw": data, "ack": ack}

    def write_CC(self, status):
        """CC send: 1-byte payload (battery status/request)."""
        return bytes([status & 0xFF])


class LogSave:
    """LogSave -- id TB (callback 0x93023c). RECEIVE-ONLY log sink (no send)."""
    CMD = {"TB": (0x54, 0x42)}

    def parse_TB(self, data, ack):
        """TB: whole data payload is one log string (mcu_log:%s)."""
        return {"log": data.decode("latin1", "replace"), "ack": ack}


class ClockSync:
    """ClockSync -- id UC (callback 0x91396c). Payload region at data+6."""
    CMD = {"UC": (0x55, 0x43)}

    def parse_UC(self, data, ack):
        """UC time-sync: 13-byte payload. data[0] must==1; t0 u32@1, t1 u32@5,
        t2 u32@9. delta = ((t1-t0)>>1)+((t2-t3)>>1) (t3 from node state)."""
        r = {"ack": ack}
        if len(data) >= 13 and data[0] == 1:
            r["t0"] = _u32(data, 1)
            r["t1"] = _u32(data, 5)
            r["t2"] = _u32(data, 9)
        return r

    def write_UC(self, time_seconds):
        """UC heartbeat: 4-byte u32 time seconds (LE)."""
        return struct.pack("<I", time_seconds & 0xFFFFFFFF)


class IMU:
    """IMU (common::IMU) -- ids GD,GF,GH,GS,OD,GI (callback 0x91c254).
    Payload region at data+6. GC is the only SEND. See imu_mcu_msg_spec.md."""
    CMD = {"GD": (0x47, 0x44), "GF": (0x47, 0x46), "GH": (0x47, 0x48),
           "GI": (0x47, 0x49), "GS": (0x47, 0x53), "OD": (0x4F, 0x44),
           "GC": (0x47, 0x43)}

    def parse_GD(self, data, ack):
        """GD -> imu/ImuSensor. data[0]=validity (must 0); i16 @1,3,5,7 (scaled
        by gyro scale), i16 @9,0xb,0xd (3 floats), i16 @0xf,0x11,0x13 (3 floats),
        i32 @0x15 timestamp/counter."""
        r = {"ack": ack, "valid": data[0] if data else 1}
        if not r["valid"]:
            r["gyro"] = [_s16(data, i) for i in (1, 3, 5, 7)]
            r["accel"] = [_s16(data, i) for i in (9, 0xB, 0xD)]
            r["mag"]   = [_s16(data, i) for i in (0xF, 0x11, 0x13)]
            if len(data) > 0x15:
                r["ts"] = _s32(data, 0x15)
        return r

    def parse_GF(self, data, ack):
        """GF -> imu/GyroBias: validity data[0] must 0; u16 @1,3,5,7 (3 floats)
        and @9,0xb (2 floats)."""
        r = {"ack": ack, "valid": data[0] if data else 1}
        if not r["valid"]:
            r["bias"] = [_u16(data, i) for i in (1, 3, 5, 7, 9, 0xB)]
        return r

    def parse_GH(self, data, ack):
        """GH -> imu/geomag: validity data[0] must 0; u16@1,@3, u8@5."""
        r = {"ack": ack, "valid": data[0] if data else 1}
        if not r["valid"]:
            r["geomag"] = [_u16(data, 1), _u16(data, 3), data[5]]
        return r

    def parse_GI(self, data, ack):
        """GI -> imu/Geomag: no validity; u16 @0,2,4."""
        return {"ack": ack, "geomag": [_u16(data, i) for i in (0, 2, 4)]}

    def parse_GS(self, data, ack):
        """GS state/status: 2-byte payload [state, value] -> ImuState."""
        return {"ack": ack,
                "state": data[0] if data else 0,
                "value": data[1] if len(data) > 1 else 0}

    def parse_OD(self, data, ack):
        """OD: [type,value] byte pairs; type 0->UltraSonicState, type 1->GyroType."""
        out = {}
        for i in range(0, len(data) - 1, 2):
            t, v = data[i], data[i+1]
            if t == 0:
                out["ultrasonic"] = v
            elif t == 1:
                out["gyro_type"] = v
        out["ack"] = ack
        return out

    def write_GC(self, state):
        """GC set IMU state: 2-byte payload [0x00, state(0/1/2)]. SEND-ONLY."""
        return bytes([0x00, state & 0xFF])


class Key:
    """Key -- id KA (callback 0x92ddb4). RECEIVE-ONLY (no KA send). Payload data+6."""
    CMD = {"KA": (0x4B, 0x41)}

    def parse_KA(self, data, ack):
        """KA key event: buf[8]=key type, buf[9]=key event (payload offset 0,1)."""
        r = {"ack": ack}
        if len(data) >= 2:
            r["type"] = data[0]
            r["event"] = data[1]
        return r


class OTADebug:
    """OTA/Debug -- ids VS,VB (callback 0x960190). Payload region at data+6.
    Sends: VP (ota req), VQ (ota status), FV (init)."""
    CMD = {"VS": (0x56, 0x53), "VB": (0x56, 0x42),
           "VP": (0x56, 0x50), "VQ": (0x56, 0x51), "FV": (0x46, 0x56)}

    def parse_VS(self, data, ack):
        """VS line-laser state: payload[0]; ==1 -> state 1, else 2."""
        v = data[0] if data else 0
        return {"line_laser_state": 1 if v == 1 else 2, "ack": ack}

    def parse_VB(self, data, ack):
        """VB version: payload = version string (e.g. mcu_manager...)."""
        return {"version": data.decode("latin1", "replace") if data else "", "ack": ack}

    def write_VP(self):
        """VP OTA request: 1-byte payload 0x00."""
        return bytes([0x00])

    def write_VQ(self, firmware_ota, rtk_ota):
        """VQ OTA status: 2-byte payload [firmware_ota, rtk_ota]."""
        return bytes([firmware_ota & 0xFF, rtk_ota & 0xFF])

    def write_FV(self):
        """FV init: 1-byte payload 0x00."""
        return bytes([0x00])


class RtcNode:
    """RtcNode -- ids RC,RT (callback 0xa94b48). Payload region at data+6.
    Sends: RB (set time), RA (sync request)."""
    CMD = {"RC": (0x52, 0x43), "RT": (0x52, 0x54),
           "RB": (0x52, 0x42), "RA": (0x52, 0x41)}

    def parse_RC(self, data, ack):
        """RC RTC calendar report: u16 year@0, u8 month@2, day@3, hour@4, min@5,
        sec@6 (year actual, month 1-12)."""
        r = {"ack": ack}
        if len(data) >= 7:
            r["year"] = _u16(data, 0)
            r["month"] = data[2]
            r["day"] = data[3]
            r["hour"] = data[4]
            r["min"] = data[5]
            r["sec"] = data[6]
        return r

    def parse_RT(self, data, ack):
        """RT rtc_tick report: u16 tick@0."""
        r = {"ack": ack}
        if len(data) >= 2:
            r["tick"] = _u16(data, 0)
        return r

    def write_RB(self, year, month, day, hour, minute, sec):
        """RB set time: 7 bytes [u16 year][month][day][hour][min][sec].
        year = actual calendar year, month 1-12 (mirrors RC)."""
        return struct.pack("<H", year & 0xFFFF) + bytes([month & 0xFF, day & 0xFF,
                                                         hour & 0xFF, minute & 0xFF,
                                                         sec & 0xFF])

    def write_RA(self):
        """RA sync request: 1-byte payload 0x00."""
        return bytes([0x00])


class ScreenInfo:
    """ScreenInfo -- ids ZC,ZE,ZR,ZT (callback 0xa9fb04). Payload at data+6.
    Sends: ZA (screen state), CI (power mode)."""
    CMD = {"ZC": (0x5A, 0x43), "ZE": (0x5A, 0x45), "ZR": (0x5A, 0x52),
           "ZT": (0x5A, 0x54), "ZA": (0x5A, 0x41), "CI": (0x43, 0x49)}

    def parse_ZT(self, data, ack):
        """ZT touch/test/control: data[0] sub-code (0xd key-shutdown, 9 wifi,
        4 factory test, 6 clear lock, 0xe roll-motor)."""
        return {"subcode": data[0] if data else 0, "ack": ack}

    def parse_ZC(self, data, ack):
        """ZC screen unlock: 16-bit signed code from data[0..1]."""
        r = {"ack": ack}
        if len(data) >= 2:
            r["code"] = _s16(data, 0)
        return r

    def parse_ZR(self, data, ack):
        """ZR rain detect: data[0]==1 -> state = data[1] (1 rain / 0 none)."""
        r = {"ack": ack}
        if len(data) >= 2 and data[0] == 1:
            r["rain"] = 1 if data[1] == 1 else 0
        return r

    def parse_ZE(self, data, ack):
        """ZE screen key reset / factory reset event (no data fields)."""
        return {"reset": True, "ack": ack}

    def write_ZA(self, lock, sim, wifi, page_num, err_low, err_high, b6=0, b7=0):
        """ZA screen state: 8 bytes [lock][sim][wifi][page_num][err_low][err_high][b6][b7]."""
        return bytes([lock & 0xFF, sim & 0xFF, wifi & 0xFF, page_num & 0xFF,
                      err_low & 0xFF, err_high & 0xFF, b6 & 0xFF, b7 & 0xFF])

    def write_CI(self, mode):
        """CI power mode: 1-byte payload (mode)."""
        return bytes([mode & 0xFF])


class TemperatureAlert:
    """TemperatureReport/Alert (UrgentAlarm) -- id DB (callback 0xac7ef4).
    Payload region at data+6. Sends: DA (clear alert)."""
    CMD = {"DB": (0x44, 0x42), "DA": (0x44, 0x41)}

    def parse_DB(self, data, ack):
        """DB alert report: mcuAlarmCode u32@0, motorFaultCode u32@4,
        liftFaultCode u16@8 (if len>9), grassFaultCode u16@10 (if len>11)."""
        r = {"ack": ack}
        if len(data) >= 8:
            r["mcuAlarmCode"] = _u32(data, 0)     # 32-bit bitmask
            r["motorFaultCode"] = _u32(data, 4)
            if len(data) > 9:
                r["liftFaultCode"] = _u16(data, 8)
            if len(data) > 11:
                r["grassFaultCode"] = _u16(data, 10)
        return r

    def write_DA(self, clear_alert):
        """DA clear alert: 4-byte u32 clearAlert bitmask (LE)."""
        return struct.pack("<I", clear_alert & 0xFFFFFFFF)


class WheelNode:
    """WheelNode -- receive WD,WF,WH,WR (callback 0xad94c8). Payload at data+6.
    (WE is registered but ignored; WH handled but not registered -- see findings.)
    Sends: WA (set wheel speed / set linear+angular)."""
    CMD = {"WD": (0x57, 0x44), "WF": (0x57, 0x46), "WH": (0x57, 0x48),
           "WR": (0x57, 0x52), "WA": (0x57, 0x41)}

    def parse_WD(self, data, ack):
        """WD wheel distance (14B payload): [u8 mode][i32 left@1][i32 right@5]
        [i32 ticks@9][u8 ?@13]."""
        r = {"ack": ack}
        if len(data) >= 14 and data[0] == 0:
            r["mode"] = data[0]
            r["left"] = _s32(data, 1)
            r["right"] = _s32(data, 5)
            r["ticks"] = _s32(data, 9)
        return r

    def parse_WF(self, data, ack):
        """WF wheel speed status: 1-byte status."""
        return {"status": data[0] if data else 0, "ack": ack}

    def parse_WH(self, data, ack):
        """WH wheel back/hub-motor protection: 2-byte [a,b] -> combined state
        (0,0)=0 (0,1)=1 (1,0)=2 (1,1)=3."""
        r = {"ack": ack}
        if len(data) >= 2:
            a, b = data[0], data[1]
            r["protection"] = {(0,0): 0, (0,1): 1, (1,0): 2, (1,1): 3}.get((a, b))
        return r

    def parse_WR(self, data, ack):
        """WR report: elem0==0 and len>0x10; i32 @1,5,9,0xd (left/right/linear/
        angular); if len>0x12 parse 2B @0x11,@0x12; if len>0x14 parse 2B @0x13,@0x14."""
        r = {"ack": ack}
        if len(data) > 0x10 and data[0] == 0:
            r["left"] = _s32(data, 1)
            r["right"] = _s32(data, 5)
            r["linear"] = _s32(data, 9)
            r["angular"] = _s32(data, 0xD)
            if len(data) > 0x12:
                r["ext1"] = [data[0x11], data[0x12]]
            if len(data) > 0x14:
                r["ext2"] = [data[0x13], data[0x14]]
        return r

    @staticmethod
    def _sign_val(v):
        """Encode a signed int as [sign][abs u32][4 zero] 9-byte block."""
        v = int(v)
        if v == 0:
            sign = 2
        else:
            sign = 0 if v > 0 else 1
        return bytes([sign]) + struct.pack("<I", abs(v) & 0xFFFFFFFF) + bytes(4)

    def write_WA(self, left, right):
        """WA set wheel speed: 19-byte payload
        [0x00][signL][u32 absL][4 zero][signR][u32 absR][4 zero]."""
        return bytes([0x00]) + self._sign_val(left) + self._sign_val(right)

    def write_WA_linear(self, linear, angular):
        """WA from linear/angular: left = linear - angular/2, right = linear + angular/2."""
        return self.write_WA(linear - angular / 2, linear + angular / 2)


# ---- registry: node name -> instance ----
NODES = {
    "MotorNode": MotorNode(), "OnOffInfo": OnOffInfo(), "BigData": BigData(),
    "LogSave": LogSave(), "ClockSync": ClockSync(), "IMU": IMU(), "Key": Key(),
    "OTADebug": OTADebug(), "RtcNode": RtcNode(), "ScreenInfo": ScreenInfo(),
    "TemperatureAlert": TemperatureAlert(), "WheelNode": WheelNode(),
}
