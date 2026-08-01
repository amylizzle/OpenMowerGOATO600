uint32  GRASS_MOTOR_FAULT_NONE                      = 0         #(0)       0x0000   无故障
uint32  GRASS_MOTOR_FAULT_SOFTWARE_OVER_CURRENT     = 1         #(1 << 0)  0x0001   软件过流
uint32  GRASS_MOTOR_FAULT_HARDWARE_OVER_CURRENT     = 2         #(1 << 1)  0x0002   硬件过流
uint32  GRASS_MOTOR_FAULT_OVER_VOLTAGE              = 4         #(1 << 2)  0x0004   过压故障
uint32  GRASS_MOTOR_FAULT_UNDER_VOLTAGE             = 8         #(1 << 3)  0x0008   欠压故障
uint32  GRASS_MOTOR_FAULT_STALL                     = 16        #(1 << 4)  0x0010   堵转故障
uint32  GRASS_MOTOR_FAULT_EMPTY                     = 32        #(1 << 5)  0x0020   空转故障
uint32  GRASS_MOTOR_FAULT_PHASE_LOSS                = 64        #(1 << 6)  0x0040   缺相故障
uint32  GRASS_MOTOR_FAULT_OVER_TEMPERATURE          = 128       #(1 << 7)  0x0080   温度故障
uint32  GRASS_MOTOR_FAULT_OFFSET                    = 256       #(1 << 8)  0x0100   偏置故障
uint32  GRASS_MOTOR_FAULT_START                     = 512       #(1 << 9)  0x0200   启动故障

uint32[] alerts
uint32 alertCode

