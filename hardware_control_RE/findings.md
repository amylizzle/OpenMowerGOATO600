# hardware_platform_node — Control Signal Mapping (findings)

Target: stripped aarch64 ROS1 node `hardware_platform_node` (ECOVACS RE).
Node name: `hardware_node`. Decompiles in `decomp/`.

## 1. MCU Serial Protocol (McuManager)

Device: `/dev/ttyS3` (config `mcuUartName` @0xb49b60; also /dev/ttyS4 @0xb4a528, /dev/ttyS2 @0xb4a548).

### Frame format (send — `comm::McuManager::onCommMsgSend` 0x8ec34c)
```
[0]  0x60        start delimiter (backtick)
[1]  ack id      (from getAck(), or 0 if ptr[40] ack-not-required flag)
[2]  len         = data_len + 2   (data byte count after [4])
[3]  mainCmd     = ptr[0]  (first  char of 2-char command id)
[4]  subCmd      = ptr[1]  (second char)
[5..5+len-1] data  (copied from ptr+8 vector, len bytes)
[5+len]     crc8    (getStrCrc8 over bytes [0..4+len])
[6+len]     0x0A    (newline terminator)
```
Total frame length = data_len + 7. Write via `UartInstance::writeBuffer_lock`.

Command id = ptr[0],ptr[1] (mainCmd,subCmd). Special ids seen in checks:
- `VP` (0x56 0x50) — exempt from flag this[320] "only-send" gate
- `WA` (0x57 0x41) and `MA` (0x4D 0x41) — suppressed when flag this[321] set (test/bench mode)

### Frame format (receive — onCommMsgReceived 0x8eca00, formatMsg 0x8eceec, processMSG 0x8ed144)
Same layout as above. Length byte at [2] must equal frame_len-4. Dispatch via `std::map` at this+104 keyed by the 2-char (mainCmd,subCmd) to callback handlers. ACK when [1] present: removes buffer ack + pushes queue.

## 2. Motor Control Signal (ROS ControlMotor handler 0x9332e8)

Log strings: `motor_ctrl :%d(motorType) %d(ctrlValue) %d(ctrlType)`, `set_cut type=%d controlType=%d value=%d`, `m_motorcontrolsSend.controls.size=%d ...`, `msgRecvd`/`sendCtrl`.

Sends a frame with command id `MA` (mainCmd=`M`=0x4D, subCmd=`A`=0x41):
```
0x60  ack  len  0x4D 0x41  [control data]  crc8  0x0A
```
Control data (SAData ptr):
- control type byte = 1 (set at buffer[0xb8])
- control value = int16 at buffer+8 (from ControlMotor.controls[index]: type at byte 1, value at int16 offset 2)

Build: `0x8eeac0` constructs message object; `0x8ef220` packs control data; `0x8ec34c` = onCommMsgSend sends frame to MCU. Called only if `this->mcuEnable` (offset 0x1b0) != 0.

## 3. GPIO / Sensor Verification (NOT direct motor output)

Function 0x8ac650 = "internal parameters" check, not a GPIO output:
- opens `/dev/block/platform/by-name/sys` (0xb48340)
- `ioctl(fd, 0x11002e03, &buf)` reads 4 bytes
- compares first 3 bytes to `"acc"` (0xb48418)
- returns 0 on match (`internal parameters sucess!!! buf=%s` @0xb48420), -1 otherwise

Function 0x8ac884 = main node init (sensor_type / rtk_gnss / se / marsh / tdk_imu), not GPIO.
GPIO output control (OnOffInfo/Key.cpp setOnOff) still to be mapped — likely via same sysfs path.

## 4. Config Struct (Config::load 0x8b6dcc)
- motorEnable @4, wheelR @8=221.0, wheelB @12=392.0, wheelOcCnt @16=12.0, wheelRate @20=65.0, imu_dir @24=1, mcuUartName @32="/dev/ttyS3"
- Sensor type check: `rtk_gnss`/`se`/`marsh`, `/data/enable_tdkimu` -> tdkImuThread

## 5. Notable Strings
- `cannot get value` @0xb4ca90/0xb56aa8/0xb5df98/0xb80e40/0xba7d38/0xbbc200/0xbe03e8
- `command [%s] failed,retry times [%d] expect value[%d]  real value[%d]` @0xb7ddf8/0xba1278
- `LORALOWPOWER ON/OFF` @0xb510a0/0xb510b8 (also 0xb90f40/58, 0xbaf5a0/b8, 0xbdc698/b0)
- Motor build path: `.../project/mr2509/x5_0416_rtk_se_ota/release/code/app/ecovacs/eros_hardware_plat...` @0xb68278

## Files
- `decomp/mcu_config_init.c`, `mcu_uart_callback.c`, `mcu_formatMsg.c`, `mcu_processMSG.c`, `mcu_sendMotor.c`
- `decomp/full_text_disasm.txt` (683572 lines raw disasm)

## MotorNode feedback + control (RESOLVED)
- MotorNode::Impl::initRegister = 0x934eb8: registers MCU command callbacks (via 0x93f760) for ids:
  (0x4d='M',0x42='B'), (0x4d,0x41='A'), (0x4d,0x45='E'), (0x4d,0x44='D'), (0x4d,0x46='F'), (0x4d,0x53='S'), (0x4d,0x43='C'), (0x4d,0x54='T')
  — all -> onSaMsgReceved = 0x931c0c. So MCU motor-feedback frames are M? ids (MB,MA,ME,MD,MF,MS,MC,MT).
- onSaMsgReceved (0x931c0c, decomp on_sa_recv.c) parses ptr3[1]=second cmd char:
  - 'E'(69): data[0]=motor type -> maps {1->1,2->2,10->0,4->5,5->6,6->4} -> publish motor::MotorType_ at this+32 (0x20).
  - 'F'(70): data[0] in {4,5,10} -> {5,6,0} -> publish MotorWarningReport at this+224 (0xe0).
  - 'S'(83): MotorSpeedReport. data layout (vector at ptr3+4): data[0]=motor type (==10 to decode), int16@1=RPM1, int16@3=RPM2, int16@5=RPM3; if size>14 also int16@7, int16@9, byte@11,12,13,14.
    Brand decode (raw int16 v46 -> speed): dechang -> ROL(((v46*2|v46>>31)&1)+v46,31); lianyi -> (unsigned short)((v46+(v46<0?3:0))>>2); kaihang -> v46. Publish MotorSpeedReport at this+240 (0xf0).
  - Other chars: MA/MB/MC/MD/MT -> state/current reports (lines before 240).
- 0x934350 (node setup, decomp after_pub context) creates publishers at this+0x10,0x20,0xc0,0xd0,0xe0,0xf0,0x100,0x110,0x170 for the motor topics (MotorCurrent 0xb68dd0, MotorStateReport 0xb68e00, MotorWarningReport 0xb68e18, MotorSpeedReport 0xb68e38, GrassMotorSpeedReport 0xb68e50, ControlMotor echo 0xb68db8 at this+0xc0, etc.); 0x914c7c=ros::Publisher::publish. Called via wrapper 0x935270 (this+0x4d8 sub-object, x1=this) then 0x934eb8(initRegister).
- 0x934eb8 registers via 0x93f760 with (mcu=this+0x1e8, 'M', x, cb, 0, this). 0x931000+0xc0c=0x931c0c is the callback (all 8 M? ids).
- Second MA sender 0x933ac8 (raw 138610+): builds MA data {byte0=0x0B(11), byte1=control.byte0(motor type), int16=control.byte1}; gated by this+0x1d8/0x1dc state; sends via 0x8eeac0->0x8ef220->0x8ec34c. So type 0x0B = STOP_AND_ENABLE motor-type command.
- Control type->MCU type (0x930fc0): {0->0x0A, 1->0x01, 3->0x02, 4->0x03, 8->0x09, 11->0x0B}. 0x93105c: type 0 = brand-encoded speed (dechang/lianyi/kaihang) + direction; type 11 = {0x0B,1,value}; else {mapped, type, value}.
- MA send frame (confirmed): [0x60][ack][len][M][A][data...][crc8][0x0A]; data = packed 4-byte controls (byte0=MCU type, byte1=dir/sub, int16=value); single-control fast path when controls.size()==1 && controls[0].byte0==0.

## Delivered: motor_control.py (best-effort script)
- `firmware/motor_control.py` sends MA control frames over `/dev/ttyS3` (115200 8N1) and reads RPM from MS feedback frames.
- Wire frame: `[0x60][ack][len=data+2][M][A][data][crc8][0x0A]`, total=data+7, crc8 over 5+data_len bytes.
- Commands (4-byte wire control): `--enable TYPE` -> {0x0B,1,type,0}; `--speed RPM` -> {0x0A,dir,brand-encoded-abs}; `--stop` -> {0x02,0,0,0}; `--read` -> parse MS (data[0]==10; int16@1=RPM1,int16@3=RPM2; brand-decode per on_sa_recv).
- Brand encode/decode implemented for dechang/lianyi/kaihang from 0x93105c/0x931c0c.
- UNKNOWN (external): CRC8 poly lives in libEcoLogger.so (getStrCrc8 import 0xb193f0; crc8 import 0xb1937c; lib not present on system, no in-binary table found). Default CRC-8/SMBUS (poly 0x1D, init 0xFF, xor 0). Adjust CRC_POLY/CRC_INIT/CRC_XOROUT if MCU rejects. `--crc-hex` dumps the table.
- Verified: py_compile OK; sample frames: enable type1 = 60 00 06 4d 41 0b 01 01 00 1d 0a; speed 100 kaihang = 60 00 06 4d 41 0a 00 64 00 f1 0a; speed -100 dechang = 60 00 06 4d 41 0a 01 c8 00 b5 0a; stop = 60 00 06 4d 41 02 00 00 00 df 0a.

## 6. GPIO: NO direct SoC GPIO in this binary (all pins live on the MCU)
- There is NO direct SoC GPIO control anywhere. Evidence (comprehensive):
  - No strings: no "/sys/class/gpio", no "gpio", no "gpiochip", no "/dev/mem", no "mmap", no GPIO register base (searched whole .rodata).
  - The only hardware I/O devices the node opens: /dev/ttyS3 (0xb49b60, MCU), /dev/ttyS2 (0xb4a548), /dev/ttyS4 (0xb4a528), /dev/pps0 (0xb76728, GPS PPS), /sys/devices/platform/pps/pwm_rise_count (0xb6db78), /sys/class/thermal/thermal_zone0/temp (0xbc3b78), /sys/bus/iio/devices/... (IIO), /dev/block/platform/by-name/sys (0xb48340).
  - The hardware-named GPIO strings are ALL dead/unreferenced rodata (no adrp+add refs): "OnOff rain IO=%d" (0xb6df88), "current OnOff chargeState=%d" (0xb6df50), "(idx 0:bump 2:fall 6:acczero 10:grass 12:roll 16:fan)" (0xb6ded0), "mcu rain detect: %d" (0xbb63c0). The 0xb6d000/0xbb6000 page offsets used in code never reach these strings.
  - The "cannot get value" getValue family (0x8d69d8/0x8d6980, 0x9f42ac, ...) is nlohmann::json::detail::iter_impl (JSON iterator), NOT a GPIO/ADC reader.
  - "LORALOWPOWER ON/OFF" (0x8baef0/0x8baf38 and repeats) are GNSS/RTK receiver serial commands, not GPIO toggles.
- Conclusion: this node is a pure ROS<->MCU bridge. ALL hardware pins (rain sensor, charge/charger contacts, bumpers, cliff/fall, wheel motors, fan, estop) are on the separate MCU and reached via the MA protocol on /dev/ttyS3 (M? subtypes: MA ack, MB motor fb, MC, MD, ME motor-type publish, MF warning, MS rpm).
- Hardware channels the node relays (logical, not physical pins):
  - OnOffInfo topics: onOffInfo/OnOffInfo, onOffInfo/EStopState, power/ChargeState, onOffInfo/EStopControl, /lidar/FanControl.
  - Sensor index->hardware (0xb6ded0): 0=bump, 2=fall(cliff), 6=acczero, 10=grass, 12=roll, 16=fan.
  - chargeState=charger contact state; rain IO=rain sensor; Stop state=estop.
- The literal pin numbers (e.g. which SoC/MCU GPIO line) are NOT in this binary; they are defined in the MCU firmware, which is NOT present in this workspace. To get a physical pin->hardware map you need the MCU binary/firmware (or hardware schematic).

## 6b. MCU protocol channel map (which hardware ↔ which message)
Two frame families on the same MCU UART (/dev/ttyS3):
- "M?" = motor-node messages (MotorNode::Impl::onSaMsgReceved, decomp/on_sa_recv.c)
- "BC" = sensor/GPIO-status message (OnOffInfo receive, fn 0x9545a8: checks buf[0]==0x42 'B', buf[1]==0x43 'C'; payload = [idx,state] byte pairs from offset 8, step 2)

Motor "M?" subtypes:
- MA (0x41) control ack: node verifies each motor command was acked; if not, resetMotorControl + restart SteadyTimer. Payload 4-byte groups {type,brand,value}.
- MB (0x42) motor current feedback: 5x16-bit per-motor values -> topic motor/MotorCurrent (ptr+16).
- MC (0x43) current value: byte@+2, MotorType=7 -> motor/MotorCurrentValue (ptr+304).
- MD (0x44) state report: flag{0,1,2}->state, type=11 -> motor/MotorStateReport (ptr+208).
- ME (0x45) motor type: choice{1,2,10,4,5,6}->type -> motor/MotorType (ptr+32).
- MF (0x46) motor warning: v43{4,5,10}->warning -> motor/MotorWarningReport (ptr+224).
- MS (0x53) speed report: rpm1@1, rpm2@3, 4 status chars@11..14, int16@5 -> motor/MotorSpeedReport (ptr+240) + std_msgs/Int16 (ptr+0x100). Brand encode dechang/lianyi/kaihang (0x93105c).
- MT (0x54) big-data machine info: 4 chars -> bigdata/BigDataMachineInfo (ptr+320).

"BC" sensor idx -> hardware (special-cased in OnOffInfo parse; generic ones published on onOffInfo/OnOffInfo with map 0xb6ded0):
- 0 = bump (bumper)
- 2 = fall (cliff/fall sensor)
- 4 = chargeState (charger contacts)  [obj+0x70; log "current OnOff chargeState=%d" 0xb6df50]
- 6 = acczero (accelerometer zero-cross)
- 8 = rain IO (rain sensor)  [obj+0x71; log "current OnOff rain IO=%d" 0xb6df88]
- 10 = grass (grass sensor)
- 12 = roll (roll/tilt)
- 14 = Stop state (estop)  [obj+0x9c; log "current Stop state=%d" 0xb6dfc0]
- 16 = fan
State change diff: fn 0x9542b8 compares arrays, logs "(idx 0:bump 2:fall 6:acczero 10:grass 12:roll 16:fan)idx=%d state=%d" (0xb6ded0, referenced at 0x9544e4).

OnOffInfo node topics: onOffInfo/OnOffInfo, onOffInfo/EStopState, power/ChargeState, onOffInfo/EStopControl, /lidar/FanControl.

## 6c. Battery status read path (RESOLVED — "CC" big-data frame)
- Battery status is read over the MCU serial link (/dev/ttyS3 @115200), NOT GPIO.
- Frame = big-data **"CC"** message: `buf[0]==0x43 'C'` && `buf[1]==0x43 'C'` (check at 0x9abb8c/0x9abba0; log `"SA=C C data.size=%d batteryID=%d"` 0xb77098 confirms).
- Payload starts at buf+8 (same as sensor BC frames). Handler 0x9abb8c-0x9ac744 parses:
  | payload off | size | value | cached at |
  |---|---|---|---|
  | +0 | u8 | mBatterySoc (battery level, %) | obj+0x1e |
  | +1 | u8 | chargeState | — (log 0xb77010 "chargeState") |
  | +2 | u16 | mChargeVol (charge voltage, mV) | obj+0x20 |
  | +4 | s16 | mChargeCur (charge current, mA) | obj+0x22 |
  | +6 | s8 | mBatteryTemperature | obj+0x1c |
  | +7 | u8 | batteryID | — (log 0xb77098 "batteryID") |
  | +8 | u8 | mChargeStep | obj+0x1d (log 0xb77038 "mChargeStep") |
  | +9..+0x12 | u16 ×5 | extended fields (if data.size>0x13) | obj+0x12..0x1a |
- Field order proven by Power.cpp log strings: 0xb76fc8 "mBatterySoc=.. mLastBigDataBatterySoc=.." reads payload+0 (diffed vs obj+8); 0xb77098 "SA=C C data.size=%d batteryID=%d" reads payload+7; 0xb77038 "mChargeVol=.. mChargeCur=.. mBatteryTemperature=.. mChargeStep=.." reads payload+8.
- Cached values diffed/deduped, then published on topics `bigdata/BigDataBatteryInfo`, `bigdata/BigDataPowerInfo`, `power/ChargeVolCur`, `power/Battery` (topic getters 0x9b96a8-0x9b9780), plus a `BatteryInfo` message (batteryCurrent mA / batteryVoltage mV / batteryLevel %, .msg at 0xb77eee).
- Log strings (0xb76dd6 "Battery", 0xb76f59/0xb76fa1/0xb76ffb/0xb7706d/0xb770c4) are debug-only; battery read itself is the CC payload above.
- To read battery on host: parse the `CC` frame (header "CC", payload@8): soc(+0), chargeState(+1), chargeVol(u16@2), chargeCur(s16@4), temp(s8@6), batteryID(+7), chargeStep(+8) — same CRC/link as BC/MA frames.

## 7. Complete receive dispatch map (ALL 12 nodes / ~40 command ids)
The `std::map` at McuManager+104 is populated by 12 distinct registration wrappers (not just one). All insert via the shared helper `0x8ec9c8` with body `(mcu=x0, mainCmd=w1, subCmd=w2, callback=x3/x4, this=x5)`. Each node's receive handler is `XxxNode::Impl::onSaMsgReceved`; the full receive command-id table:

| node (class) | reg wrapper | callback | ids (main,sub) |
|---|---|---|---|
| MotorNode | 0x93f760 | 0x931c0c | MB,MA,ME,MD,MF,MS,MC,MT |
| OnOffInfo | 0x9566b0 | 0x9545a8 | BC |
| BigData | 0x9b0628 | 0x9abb60 | CB,CC,CH,LP |
| LogSave | 0x930448 | 0x93023c | TB |
| ClockSync | 0x914e40 | 0x91396c | UC |
| IMU | 0x91ecdc | 0x91c254 | GD,GF,GH,GS,OD,GI |
| Key | 0x92e044 | 0x92ddb4 | KA |
| OTA/Debug | 0x968694 | 0x960190 | VS,VB |
| RtcNode | 0xa96ac8 | 0xa94b48 | RC,RT |
| ScreenInfo | 0xaa38b0 | 0xa9fb04 | ZC,ZE,ZR,ZT |
| TemperatureReport/Alert | 0xaca534 | 0xac7ef4 | DB |
| WheelNode | 0xadd0b8 | 0xad94c8 | WD,WF,WE,WR |

Node identities were confirmed from the handler rodata (topic/source strings each callback references):
- Motor 0x931c0c→0xb68000: `motor/ControlMotorCutter`, `motor/ControlRollMotor`, `motor/ControlMotor`, `motor/LawnMowerMotor`, `cut_motor_ctrl`/`lens_motor_ctrl`, brands kaihang/dechang/lianyi.
- OnOffInfo 0x9545a8→0xb6d000: estop control, `mFanPwmVal`, `pps/pwm_rise_count`.
- BigData 0x9abb60→0xb76000: battery/rtk_node/pps (CC handler as ## 6c).
- LogSave 0x93023c→0xb67000: `LogSave.cpp`, `mcu_log:%s`.

## 7b. LogSave is RECEIVE-ONLY — no "TB" send exists
Exhaustive search (all 28 `bl #0x8ec34c` onCommMsgSend call sites, plus every `#0x54`/`#0x42` command-byte write in the binary) found NO function that sends a TB frame. Every onCommMsgSend caller uses other cmds: HA(48,41) 0x8fdae4, FC(46,43) 0x8ff0dc, FB(46,42) 0x8fe808, EA(45,41) 0x909efc, GC(47,43) 0x91d844, FV(46,56) 0x960670/0x960d44, CA(43,41) 0x9aa89c, RB(52,42) 0xa9578c, ZA(5a,41) 0xa9d804, DA(44,41) 0xac72e4, WA(57,41) 0xadaa34, plus MotorNode sends (0x931804/0x931a10/0x931b88/0x9339bc/0x933bc0). No send sets buf[0]=0x54 ('T').
- LogSave class (ctor 0x9300ec, cb 0x93023c, dtor 0x930214, reg-wrapper 0x930448) has NO SAData construction (no 0x8eeac0/0x8ef220/0x8ec34c anywhere in 0x930000-0x930c00) and NO ROS topics (constructed as `make_unique<common::LogSave>()` with no NodeHandle, startnode.c:114).
- Direction: MCU→host. The receive callback 0x93023c logs the incoming `mcu_log` payload (whole data vector as one `%s` string, log id = this[0], set by initMcuLog('mcu_base_board') 0x8abb40). TB is a log-save report from MCU; LogSave is a receive-only log sink.
- The `T?` mainCmd registrations (0x935044 TM, 0xa95450 TR, 0xa9d790 TZ) are OTHER nodes' receive callbacks, not sends.
- ClockSync 0x91396c→0xb62000: `clockSync data error`, `mainboard/Deltats`, `time sync error`, `receive U C every 1s/30s`, `Lost 'U C' message` → UC is the 1s MCU heartbeat/ack + time sync.
- IMU 0x91c254→0xb64000/0xb66000: `imu/ImuSensor`, `imu/ImuState`, `imu/InitIMU`, `imu/GyroType`, `imu/GyroInfo`, `imu/GyroBias`, `onOffInfo/UltraSonicState`; class `common3IMU`.
- Key 0x92ddb4→0xb67000: `Key.cpp`, `key/Key`, `type:%d event:%d`.
- OTA/Debug 0x960190→0xb6f000/0xb70000: `ota/Ota`, `debug/UartDebug`, `/tmp/mcu_version.txt`, `ROBOT_IS_OTA/ROBOT_IS_NORMAL`, `UART-DEBUG:req`, `receive-VB`, `line laser state`.
- RtcNode 0xa94b48→0xbb3000: `RtcNode.cpp`, `got rtc time`, `got utc`, `rtc_tick`, `TIMEZONE`.
- ScreenInfo 0xa9fb04→0xbb5000/0xbb6000: `ScreenInfo.cpp`, screen lock/err-code/pin, `screen/ScreenResetEvent`, `screen key reset`, `BigDataFactoryReset`, `factory_reset.sh`.
- TemperatureReport/Alert 0xac7ef4→0xbc4000: `TemperatureReport`, `coreTemp`, `start calibrate imu`, `alertmcu/pubalert`, `alertMotor/pubalert`.
- WheelNode 0xad94c8→0xbc7000: `wheel/WheelDistanceReport`, `wheel/SetWheelSpeed`, `wheel/WheelBackProtection`, `wheel/WheelHubMotorProtection`, `wheel/WheelSpeedReport`, `wheel/SetLinearAngularSpeed`, `amp:%f`.

CORRECTED (vs earlier): BC and CC ARE in the dispatch map (OnOffInfo/BigData register their own wrappers 0x9566b0/0x9b0628); there is no second dispatch mechanism. The receive map is far larger than the 8 M? ids alone.

## 8. RtcNode — RTC message formats (receive cb 0xa94b48, send 0xa95760/0xa95838)

Class `RtcNode` (rodata 0xbb3000: `RtcNode.cpp`, `got rtc time`, `got utc`, `rtc_tick`, `TIMEZONE`, `set_time_help.sh`). Receive callback 0xa94b48 (reg-wrapper 0xa96ac8 registers ids `RC`=`R`,`C` and `RT`=`R`,`T`; both → 0xa94b48). Callback reads buffer ptr (0x915110), requires buf[0]=0x52('R'), dispatches on buf[1]=0x43('C')→RC path 0xa94b90, 0x54('T')→RT path 0xa94da8. Payload = msg->buffer + 8 (vector data). Topics published: `rtc/Synchronize`@+0x110, `bigdata/BigDataRTCTick`@+0x100, `rtc/Rtc`@+0x120, `rtc/TimeZone`@+0x130. Flags: this+0x148=mFirstBootReport, this+0x150=last-tick-time.

### Receive RC — MCU RTC calendar report (payload ≥7 bytes)
```
off 0-1  u16 year   (LE, actual year e.g. 2024)  -> tm_year = year-1900 (0x76c)
off 2    u8  month  (1-12)                        -> tm_mon = month-1
off 3    u8  day
off 4    u8  hour
off 5    u8  min
off 6    u8  sec
```
Node builds `tm` at sp+0x78, calls `0x8ab4c0` (mktime)→time_t, logs `got rtc time` (fmt 0xbb3000+0xc80), then `0xa95a34(this,time_t)` sets the ROS clock (calls 0x8f3cac,0xa95b3c,0x8ab950,0xa95b8c,0x8f56c0,0x8ab5a0). Size check: data.size > 6.

### Receive RT — MCU rtc_tick report (payload ≥2 bytes)
```
off 0-1  u16 rtc_tick (LE)
```
Node builds a u16 object (0x9b4444) from the value, publishes to `bigdata/BigDataRTCTick`@+0x100 via `0x9b4460`. If mFirstBootReport (this+0x148) set: log `mFirstBootReport=%d rtc_tick=%d` (fmt 0xbb3000+0xce0), clear flag, set this+0x150=now (0x8ab530), publish. Else throttle via elapsed check (`0x8eebbc` now-vs-this+0x150, `0xa96744` compare) — publish only if elapsed, then log `rtc_tick=%d` (fmt 0xbb3000+0xd20).

### Send RB — set time (0xa95760)
Args: x0=this, w1=u16 year, w2..w6=month/day/hour/min/sec. Cmd bytes 0x52('R'),0x42('B'); data = 7 bytes `[u16 year][u8 month][u8 day][u8 hour][u8 min][u8 sec]` (sp+0x38..+0x3e, end=+7). Packed 0x8ef220, sent via `0x8ec34c` with this+0x140. Mirrors RC receive exactly (year stored as actual calendar year, month 1-12). Caller 0xa95568 converts unix-seconds→calendar (gmtime-like: tm_year+0x76c, tm_mon+1) and only sends if year>1999 (0x7cf). Invoked from `rtc/Synchronize` handler 0xa94fdc when msg byte0==1 (unix seconds at msg+4).

### Send RA — sync request (0xa95838)
Cmd bytes 0x52('R'),0x41('A'); data = 1 byte `0x00` (sp+0x38, end=+1); SAData byte0x68=1. Sent via `0x8ec34c` with this+0x140. Invoked from `rtc/Synchronize` handler 0xa94fdc when msg byte0==0, and after set-time (RB) in the byte0==1 branch.

### rtc/Synchronize handler 0xa94fdc (subscriber → MCU)
Reads msg data ptr (0xa96860): if msg[0]==0 → send RA; if msg[0]==1 → log `set time:%lld` with u32 at msg+4, call 0xa95568(unix secs)→send RB, then send RA. Wrapper 0xa95e0c → 0xa94fdc is the subscribe callback.

## 9. ScreenInfo — Z? receive formats + ZA/CI send formats

Class `ScreenInfo` (rodata 0xbb5000/0xbb6000: `ScreenInfo.cpp`, screen lock/err-code/pin, `screen/ScreenResetEvent`, `screen key reset`, `BigDataFactoryReset`, `factory_reset.sh`). Registration wrapper 0xaa38b0 registers ids `ZC`=`Z`,`C`, `ZE`=`Z`,`E`, `ZR`=`Z`,`R`, `ZT`=`Z`,`T` — ALL → receive callback **0xa9fb04**, on this+0x148. Callback requires buf[0]=0x5A('Z') else bail 0xaa04cc; dispatches subCmd buf[1]: 'T'(0x54)→0xa9fb4c, non-T→0xa9ff98 (ZC 0xa9ffa8, ZR 0xaa001c, ZE 0xaa0148). Payload = msg->buffer + 8 (vector data). Frame = standard `[0x60][ack][len=data+2][Z][?][data][crc8][0x0A]`.

### Receive parse per sub-command
- **ZT** (0xa9fb4c, touch/test/control): byte at data+0 → sp+0x280 (+0x281=0), publish obj+0x60 via 0xaa5348 → `screen/ScreenTaskEvent`. Sub-codes of data[0]:
  - 0xd: key-shutdown — save logs + publish bool obj+0xd0 via 0x9ec3f0
  - 9: wifi_mark check (0x8ab400) — log + publish {0,0xac} obj+0x110 via 0xaa04e4
  - 4: run `/usr/bin/auto_test/factoryRunTest.sh &` via 0x8aba70
  - 6: set obj+0x135=0 (screen lock cleared)
  - 0xe: CTRL_ROLL_MOTOR — log + publish {0,0xb9} obj+0x110 + publish obj+0xe0 via 0xaa60f4
- **ZC** (0xa9ffa8, screen lock/unlock code): data+0→sp+0x2a1, data+1→sp+0x2a0; 0xaa6d04 init sp+0x270; data+0 via 0x91514c → 16-bit signed → sp+0x270; publish obj+0xa0 via 0xaa6d20 → `screen/ScreenUnlockEvent` (screen lock/code, u16).
- **ZR** (0xaa001c, rain detect): 0xaa7930 init sp+0x268; data+0→sp+0x2a3, data+1→sp+0x2a2; if data+0==1 then data+1==1→sp+0x268=1, data+1==0→sp+0x268=0; compare obj+0x130, if changed set obj+0x130 + log `mcu rain detect: %d`; publish obj+0xb0 via 0xaa794c → `onOffInfo/RainDetectState`.
- **ZE** (0xaa0148, screen key reset / factory reset): if obj+0x134 (isResetting)!=0 → log `[resetting test] shield factory reset again`; else: init sp+0x260 set=1, publish obj+0x80 via 0xaa8578 → `screen/ScreenKeyResetEvent`, log `screen key reset event:%d`, set obj+0x134=1, log, init sp+0x258 set=1 publish obj+0x90 via 0x9ec3f0, set sp+0x258=3 publish obj+0xd0 via 0x9ec3f0, run `TRIGGER=KEY /usr/bin/factory_reset.sh &` via 0x8aba70; if -1 log `factory reset fail!!!`.

### Send formats (both via 0x8ec34c, this+0x148)
- **ZA** (0xa9d7b8, screen state): cmd bytes 0x5A('Z'),0x41('A'); data = 8 bytes from obj+0x135..0x13c:
  `[0] screen lock`(obj+0x135) `[1] sim`(0x136) `[2] wifi`(0x137) `[3] page_num`(0x138) `[4] err_code_low`(0x139) `[5] err_code_high`(0x13a) `[6][7] unknown`(0x13b,0x13c).
  0x8ab750 compares obj+0x135..0x13d before send. Logs `[Z A] screen lock:%d sim=%d wifi=%d page_num=%d` (0xbb5000+0xa08), `[Z A] screen err_code_high=0x%02X err_code_low=0x%02X` (0xbb5000+0xa58). Wire: `0x60 ack 0x0A 0x5A 0x41 [8B] crc8 0x0A` (15 bytes).
- **CI** (0xa9e484, screen power mode): cmd bytes 0x43('C'),0x49('I'); reads received data[0]: if==3 → subCmd 'I' + data byte = 2; if==2 → subCmd 'I' + data byte = 1; else skip. 1-byte payload packed via 0x8ef220, send via 0x8ec34c. Log `screen power mode:%d %d` (0xbb5000+0xd98). Wire: `0x60 ack 0x03 0x43 0x49 [mode] crc8 0x0A` (8 bytes).
- ScreenInfo **ROS ScreenPageControl** subscriber 0xa9da1c: if obj+0x134 set → log shield, skip; else reads param u32: byte0→obj+0x138, ldrh low→obj+0x139, high→obj+0x13a; log `err_code=E%d` if halfword!=0; calls 0xa9d7b8 (ZA send).

Note: RtcNode's RB/RA sends (0xa95760/0xa95838, ## 8) use this+0x140, NOT ScreenInfo; ScreenInfo sends are only ZA and CI on this+0x148.

## 8. RTK/Lora command architecture (rover-enable sequence)

### Send mechanism (for reference — only the order matters)
- Every command is sent via `bl 0x8ac160` (a PLT/imported send) with the string built by `0x8b000c` (string append). Commands are either rodata literals appended at runtime, or pre-built `std::string` globals in `.bss` (0xd13480, 0xd14028, 0xd15028, 0xd150a8 …).
- The orchestrator is a **command dispatcher at `0x8b9138`** (stack frame `#0xfa0`, 5823 lines, 236 send blocks). Prologue checks `w0==1 && w1==0xffff` before running the body — calling it with `(1, 0xffff)` triggers the full RTK/Lora config dump. It holds the complete command table (238 unique literals; per-block order in `ordered.txt`).

### Command literal locations (rodata)
- Lora radio commands at `0xb51000+`: 0x68=`$BDCTR,10,1,3.000*72\r\n`, 0x80=`reboot\r\n`, 0x90=`rtktype rover\r\n`, 0xa0=`LORALOWPOWER ON\r\n`, 0xb8=`LORALOWPOWER OFF\r\n`, 0xd0=`loraconfig config\r\n`, 0xe8/0x100=`LORACONFIG FREECH OFF/ON`, 0x118/0x130=`LORACONFIG FAUTO OFF/ON`, 0x148=`TRANS ON COM1 COM3\r\n`, 0x160=`TRANS ON COM2 COM3\r\n`, 0x178=`TRANS OFF\r\n`, 0x188/0x198=`LORAOTA ON/OFF`, 0xb50ff0=`rtktype\r\n`, 0x1a8=`setsignalprofile mower\r\n`, 0x1c8=`qualitylevel 0\r\n`.
- GNSS receiver config at `0xb51e00`=`COM3 115200 N 8 1 IN:RTCM OUT:BYNAV`, `0xb51208`=`interfacemode com3 rtcm bynav`, `0xb512a8`=`WORKFREQS B1IB2IB3I BEIDOU2`, `0xb50d08`=`fix auto`, `0xb50d18`=`log bestposa ontime 1`, `0xb50a38`=`log com1 gpgga ontime 0.1`, `0xb50a58`=`log com1 bestposa ontime 0.1`, `0xb509c8`=`saveconfig`, `0xb51cc0`=`RTKTYPE BASE`, `0xb51cd0`=`RTKTYPE ROVER`.
- NMEA output logs at `0xb51960+`: 0x960=`COM1,GPGGA,ABBASCII,ONTIME,0.100,0.000`, 0x988=`COM1,BESTPOS,ASCII,ONTIME,0.100,0.000`, 0x9b0=`COM1,SYSRTS,ABBASCII,ONTIME,1.000,0.000`, 0x9d8=`COM1,GPNTR,ABBASCII,ONTIME,30.000,0.000`, 0xa00=`COM3,GPZDA,ABBASCII,ONTIME,30.000,0.000`, 0xa28=`COM1,REFSTATION,ASCII,ONTIME,10.000,0.000`.
- Base-station-side commands (excluded from rover subset): `log com3 rtcm1005b/1033b/1074b/1084b/1094b/1114b/1124b ontime…`, `lockoutsystem bd2/qzss`, `UNLOCKOUTSYSTEM BD2/QZSS`, `set station on/off`, `$$01set gnss pwr on/off`.

### Rover RTK-enable sequence (ordered)
The dispatcher dumps ALL commands (base+rover, ON/OFF pairs), so there is no single minimal rover-only path in it; this is the rover-side subset assembled from its commands, in send order.

**1. Lora radio (receives corrections from base) — set rover mode + bridge to receiver:**
```
rtktype rover\r\n                  ; 0xb51090  radio = rover mode
LORALOWPOWER OFF\r\n               ; 0xb510b8  full receive power
TRANS ON COM1 COM3\r\n             ; 0xb51148  route radio<->receiver (COM1/COM3 bridge)
setsignalprofile mower\r\n         ; 0xb511a8  signal profile for mower
qualitylevel 0\r\n                 ; 0xb511c8  quality level
```

**2. GNSS receiver (computes RTK fix) — accept RTCM on COM3, apply, output position:**
```
COM3 115200 N 8 1 IN:RTCM OUT:BYNAV   ; 0xb51e00  COM3 serial = RTCM correction input
interfacemode com3 rtcm bynav         ; 0xb51208  COM3 interface = RTCM
SETSIGNALPROFILE MOWER                ; 0xb51e28
QUALITYLEVEL 0                        ; 0xb51e40
WORKFREQS B1IB2IB3I BEIDOU2           ; 0xb512a8  enable BD2/BEIDOU2 constellations
fix auto                              ; 0xb50d08  automatic RTK fix mode
log bestposa ontime 1                 ; 0xb50d18  RTK-fixed position
log com1 gpgga ontime 0.1             ; 0xb50a38  NMEA position (navigation)
log com1 bestposa ontime 0.1          ; 0xb50a58  best-pos ASCII position
saveconfig                            ; 0xb509c8  persist config
```
Flow: radio (rover) receives RTCM over the air → `TRANS ON COM1 COM3` forwards it → receiver's COM3 (`IN:RTCM`) ingests it → `fix auto` + enabled BD2 yields an RTK fix → position out on COM1 (`bestposa`/`gpgga`) for the mower.
RTK nodes: `common::RTK` on /dev/ttyS4, `common::RTKLora` on /dev/ttyS2 (decomp/startnode.c:229-243); gated by `getRtkValue("/data/rgb/conf.bin")` + `"2024"` version check (line 199).
