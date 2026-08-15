uint8 type
uint16 vaule  #报警码值

#enum type
uint8 INIT_PAGE                 = 0  #初始化页面
uint8 INPUT_PIN_CODE_PAGE       = 1  #PIN码输入页面
uint8 NO_MAP_PAGE               = 2  #无图
uint8 STANDBY_WITH_MAP_PAGE     = 3  #有图待机
uint8 TASK_IN_PROGRESS_PAGE     = 4  #工作中(包含建图和回充)
uint8 TASK_PAUSED_PAGE          = 5  #任务暂停
uint8 OTA_PAGE                  = 6  #OTA
uint8 EMERGENCY_STOP_PAGE       = 7  #急停
uint8 ALARM_PAGE                = 8  #报警界面
uint8 BATTERY_SOC_PAGE          = 9  #电量百分比

