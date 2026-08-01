uint8 type

#enum type
uint8 SET_PIN_CODE              = 1  #按键触发设置PIN码(PIN码未设置情况下,任意按键触发,无输错限制),切换到PIN码输入页面
uint8 VERIFY_PIN_CODE           = 2  #按键触发校验PIN码(如重置操作,有5次输错限制).切换到PIN码输入页面
uint8 PIN_CODE_TIMEOUT          = 3  #PIN码输入超时
uint8 MOW_TEST                  = 4  #跑机测试模式
uint8 LOW_POWER                 = 5  #低功耗
uint8 SCREEN_UNLOCK             = 6  #解锁
uint8 AUTO_MOW                  = 7  #割草
uint8 RETURN                    = 8  #回充
uint8 BLUETOOTH_NETWORK_CONF    = 9  #打开蓝牙进入待配网状态
uint8 ERROR_KEY_OPERATION       = 10 #错误按键操作
uint8 BLUETOOTH_ERROR           = 11 #解决蓝牙问题
uint8 OTA_ROLLBACK              = 12 #自动回退版本
uint8 KEY_SHUTDOWN              = 13 #手动按键关机
uint8 CTRL_ROLL_MOTOR           = 14 #按键控制翻转打草头电机

