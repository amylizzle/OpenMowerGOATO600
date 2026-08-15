uint32  MOTOR_FAULT_NONE                    = 0         #(0)       0x0000   无故障
uint32  MOTOR_FAULT_PHASE_LOSS              = 1         #(1 << 0)  0x0001   缺相
uint32  MOTOR_FAULT_HARDWARE_OVER_CURRENT   = 2         #(1 << 1)  0x0002   硬件过流
uint32  MOTOR_FAULT_SOFTWARE_OVER_CURRENT   = 4         #(1 << 2)  0x0004   软件过流
uint32  MOTOR_FAULT_STALL                   = 8         #(1 << 3)  0x0008   堵转
uint32  MOTOR_FAULT_HALL_SENSOR             = 16        #(1 << 4)  0x0010   霍尔异常
uint32  MOTOR_FAULT_UNDER_VOLTAGE           = 32        #(1 << 5)  0x0020   欠压故障
uint32  MOTOR_FAULT_OVER_TEMPERATURE        = 64        #(1 << 6)  0x0040   过温
uint32  MOTOR_FAULT_POWER_INTEGRAL_LIMIT    = 128       #(1 << 7)  0x0080   功率积分超限

uint32[] alerts
uint32 alertCode
