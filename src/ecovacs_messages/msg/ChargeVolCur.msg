uint16 chargeVol
int16 chargeCur

#enum
uint8 CHARGE_STEP_STANDBY   = 0 #待机
uint8 CHARGE_STEP_PREPARE   = 1 #装备
uint8 CHARGE_STEP_FAST      = 2 #快充
uint8 CHARGE_STEP_CC        = 3 #恒流
uint8 CHARGE_STEP_CV        = 4 #恒压
uint8 CHARGE_STEP_FULL      = 5 #充满
uint8 CHARGE_STEP_STOP      = 6 #停止
uint8 chargeStep
int8 chargeTemp

