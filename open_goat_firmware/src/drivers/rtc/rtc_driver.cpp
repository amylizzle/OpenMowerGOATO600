#include "rtc_driver.hpp"
// ## 8. RtcNode — RTC message formats (receive cb 0xa94b48, send 0xa95760/0xa95838)

// Class `RtcNode` (rodata 0xbb3000: `RtcNode.cpp`, `got rtc time`, `got utc`, `rtc_tick`, `TIMEZONE`, `set_time_help.sh`). Receive callback 0xa94b48 (reg-wrapper 0xa96ac8 registers ids `RC`=`R`,`C` and `RT`=`R`,`T`; both → 0xa94b48). Callback reads buffer ptr (0x915110), requires buf[0]=0x52('R'), dispatches on buf[1]=0x43('C')→RC path 0xa94b90, 0x54('T')→RT path 0xa94da8. Payload = msg->buffer + 8 (vector data). Topics published: `rtc/Synchronize`@+0x110, `bigdata/BigDataRTCTick`@+0x100, `rtc/Rtc`@+0x120, `rtc/TimeZone`@+0x130. Flags: this+0x148=mFirstBootReport, this+0x150=last-tick-time.

// ### Receive RC — MCU RTC calendar report (payload ≥7 bytes)
// ```
// off 0-1  u16 year   (LE, actual year e.g. 2024)  -> tm_year = year-1900 (0x76c)
// off 2    u8  month  (1-12)                        -> tm_mon = month-1
// off 3    u8  day
// off 4    u8  hour
// off 5    u8  min
// off 6    u8  sec
// ```
// Node builds `tm` at sp+0x78, calls `0x8ab4c0` (mktime)→time_t, logs `got rtc time` (fmt 0xbb3000+0xc80), then `0xa95a34(this,time_t)` sets the ROS clock (calls 0x8f3cac,0xa95b3c,0x8ab950,0xa95b8c,0x8f56c0,0x8ab5a0). Size check: data.size > 6.

// ### Receive RT — MCU rtc_tick report (payload ≥2 bytes)
// ```
// off 0-1  u16 rtc_tick (LE)
// ```
// Node builds a u16 object (0x9b4444) from the value, publishes to `bigdata/BigDataRTCTick`@+0x100 via `0x9b4460`. If mFirstBootReport (this+0x148) set: log `mFirstBootReport=%d rtc_tick=%d` (fmt 0xbb3000+0xce0), clear flag, set this+0x150=now (0x8ab530), publish. Else throttle via elapsed check (`0x8eebbc` now-vs-this+0x150, `0xa96744` compare) — publish only if elapsed, then log `rtc_tick=%d` (fmt 0xbb3000+0xd20).

// ### Send RB — set time (0xa95760)
// Args: x0=this, w1=u16 year, w2..w6=month/day/hour/min/sec. Cmd bytes 0x52('R'),0x42('B'); data = 7 bytes `[u16 year][u8 month][u8 day][u8 hour][u8 min][u8 sec]` (sp+0x38..+0x3e, end=+7). Packed 0x8ef220, sent via `0x8ec34c` with this+0x140. Mirrors RC receive exactly (year stored as actual calendar year, month 1-12). Caller 0xa95568 converts unix-seconds→calendar (gmtime-like: tm_year+0x76c, tm_mon+1) and only sends if year>1999 (0x7cf). Invoked from `rtc/Synchronize` handler 0xa94fdc when msg byte0==1 (unix seconds at msg+4).

// ### Send RA — sync request (0xa95838)
// Cmd bytes 0x52('R'),0x41('A'); data = 1 byte `0x00` (sp+0x38, end=+1); SAData byte0x68=1. Sent via `0x8ec34c` with this+0x140. Invoked from `rtc/Synchronize` handler 0xa94fdc when msg byte0==0, and after set-time (RB) in the byte0==1 branch.

// ### rtc/Synchronize handler 0xa94fdc (subscriber → MCU)
// Reads msg data ptr (0xa96860): if msg[0]==0 → send RA; if msg[0]==1 → log `set time:%lld` with u32 at msg+4, call 0xa95568(unix secs)→send RB, then send RA. Wrapper 0xa95e0c → 0xa94fdc is the subscribe callback.

// ### How/when RTC messages are sent (implementation note)
// - `rtc/Synchronize` is a subscriber using **`protocol::SAData`** (the shared MCU-transport message type also used by IMU/Key/ClockSync; see mangled `St5_BindIFMN6common7RtcNode4ImplEFvRKN5boost10shared_ptrIN8protocol6SADataEEEE...`). The handler reads the SAData buffer directly:
//   - `buf[0]==0` → send **RA** (request MCU RTC sync).
//   - `buf[0]==1` → log, take unix-seconds as **u32 at `buf+4`**, convert to calendar, send **RB**, then send **RA**.
//   - So the practical message layout is `uint8 cmd` + `uint32 unix_time` (cmd 0 = "sync me", cmd 1 = "set MCU clock to this unix time, then sync").
// - RTC RB/RA/RC/RT = **absolute wall-clock sync**, purely event-driven by `rtc/Synchronize` publishes (on boot send RA once; to set clock publish cmd=1 + unix seconds).
// - ClockSync **UC** (`0x55`,`0x43`) path (receive 0x91396c, send 0x914100) is a **separate continuous 1s/30s drift sync** (`mainboard/Deltats`, `receive/send U C every 1s`); it never sends RB/RA and does not drive the RTC calendar messages.
namespace xbot::driver::rtc {

RTCDriver::RTCDriver(xbot::driver::mcu::Dispatcher* dispatcher): mcu_driver_(dispatcher){
    if (dispatcher) {
    dispatcher->RegisterHandler(
        static_cast<uint8_t>('R'), static_cast<uint8_t>('C'),
        etl::delegate<void(const uint8_t *, size_t, uint8_t)>::create<RTCDriver, &RTCDriver::OnRC>(*this));        
    dispatcher->RegisterHandler(
        static_cast<uint8_t>('U'), static_cast<uint8_t>('C'),
        etl::delegate<void(const uint8_t *, size_t, uint8_t)>::create<RTCDriver, &RTCDriver::OnUC>(*this));        
  }
}

void RTCDriver::Start() {
    const uint8_t request = 0x00;
    mcu_driver_->SendMessage('R','A', &request, 1); //request MCU's RTC value
    startTime = std::chrono::steady_clock::now();
}

void RTCDriver::Sync() {
    const uint32_t secondsSinceStart = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - startTime).count());
    std::vector<uint8_t> UCmsg = {
        static_cast<uint8_t>(0x01), 
        static_cast<uint8_t>(secondsSinceStart),
        static_cast<uint8_t>(secondsSinceStart >> 8),
        static_cast<uint8_t>(secondsSinceStart >> 16),
        static_cast<uint8_t>(secondsSinceStart >> 24)
    };

    mcu_driver_->SendMessage('U','C', UCmsg.data(), UCmsg.size());
    ULOG_INFO("RTC Sync: %u",secondsSinceStart);
}

void RTCDriver::OnRC(const uint8_t *payload, size_t length, uint8_t ack){
// ### Receive RC — MCU RTC calendar report (payload ≥7 bytes)
// ```
// off 0-1  u16 year   (LE, actual year e.g. 2024)  -> tm_year = year-1900 (0x76c)
// off 2    u8  month  (1-12)                        -> tm_mon = month-1
// off 3    u8  day
// off 4    u8  hour
// off 5    u8  min
// off 6    u8  sec
// ```
    (void)ack;
    if (length == 7) {
        ULOG_INFO("RC Receive %u,%u,%u, %u:%u:%u", (uint16_t)payload[0], payload[2], payload[3], payload[4], payload[5], payload[6]);
    } else {
        ULOG_INFO("RC Receive bad length %u", length);
    }
// if it's off:
// ### Send RB — set time (0xa95760)
// Args: x0=this, w1=u16 year, w2..w6=month/day/hour/min/sec. Cmd bytes 0x52('R'),0x42('B'); data = 7 bytes `[u16 year][u8 month][u8 day][u8 hour][u8 min][u8 sec]` (sp+0x38..+0x3e, end=+7). Packed 0x8ef220, sent via `0x8ec34c` with this+0x140. Mirrors RC receive exactly (year stored as actual calendar year, month 1-12). Caller 0xa95568 converts unix-seconds→calendar (gmtime-like: tm_year+0x76c, tm_mon+1) and only sends if year>1999 (0x7cf). Invoked from `rtc/Synchronize` handler 0xa94fdc when msg byte0==1 (unix seconds at msg+4).

}

void RTCDriver::OnUC(const uint8_t *payload, size_t length, uint8_t ack){
    (void)ack;
    //recieve UC is one byte flag, 3 uint32s - t0, t1, t2
    if (length == 13) {
        ULOG_INFO("UC Receive t0,t1,t2 = %u,%u,%u",(uint32_t)payload[1],(uint32_t)payload[5],(uint32_t)payload[9]);
    } else {
        ULOG_INFO("UC Received bad length %u", length);
    }
}

}