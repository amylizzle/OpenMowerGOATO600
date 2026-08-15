uint32  EMERGENCY_INCLINE   = 1         #(1 << 0)  0x0001   陀螺仪抬起报警
uint32  EMERGENCY_ELEVATE   = 2         #(1 << 1)  0x0002   跌落报警
uint32  EMERGENCY_TURNOVER  = 4         #(1 << 2)  0x0004   翻到报警
uint32  EMERGENCY_BUMP      = 8         #(1 << 3)  0x0008   撞板报警
uint32  EMERGENCY_BACK      = 16        #(1 << 4)  0x0010   后退报警
uint32  EMERGENCY_STOP      = 32        #(1 << 5)  0x0020   急停报警
uint32  EMERGENCY_LSPEED    = 64        #(1 << 6)  0x0040   左驱动轮扭矩报警
uint32  EMERGENCY_RSPEED    = 128       #(1 << 7)  0x0080   右驱动轮扭矩报警
uint32  EMERGENCY_LWERR     = 256       #(1 << 8)  0x0100   左驱动电机故障码报警
uint32  EMERGENCY_RWERR     = 512       #(1 << 9)  0x0200   右驱动电机故障码报警
uint32  EMERGENCY_BATTEMP   = 1024      #(1 << 10) 0x0400   电池和底板温度报警
uint32  EMERGENCY_HEART     = 2048      #(1 << 11) 0x0800   核心板心跳丢失报警
uint32  EMERGENCY_GYRO      = 4096      #(1 << 12) 0x1000   陀螺仪数据异常报警
uint32  EMERGENCY_CORE_P    = 8192      #(1 << 13) 0x2000   核心板电源异常报警
uint32  EMERGENCY_M12_P     = 16384     #(1 << 14) 0x4000   电机电源12V报警
uint32  EMERGENCY_LCERR     = 32768     #(1 << 15) 0x8000   左驱动电机故障码报警
uint32  EMERGENCY_RCERR     = 65536     #(1 << 16) 0x10000  左切割电机故障码报警
uint32  EMERGENCY_LIFTMOT   = 131072    #(1 << 17) 0x20000  调高电机报警
uint32  EMERGENCY_BUMP_D    = 262144    #(1 << 18) 0x40000  短撞板触发
uint32  EMERGENCY_ELEVATE_D = 524288    #(1 << 19) 0x80000  短跌落触发
uint32  EMERGENCY_CHARGEERR = 1048576   #(1 << 20) 0x100000 充电报警--warning类
uint32  EMERGENCY_GRASS_MOT = 2097152   #(1 << 21) 0x200000 打草电机故障类报警
uint32  EMERGENCY_FANERR    = 4194304   #(1 << 22) 0x400000 雷达风扇堵转报警
uint32  EMERGENCY_INCLINE_D = 8388608   #(1 << 23) 0x800000 O600短倾斜陀螺仪报警
uint32  EMERGENCY_BATERR    = 16777216  #(1 << 24) 0x1000000电池故障告警

uint32[] alerts
uint32 clearAlert

