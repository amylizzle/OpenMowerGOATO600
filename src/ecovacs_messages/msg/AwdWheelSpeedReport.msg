float32 lf_wheel_speed    #左前轮轮速 m/s
float32 rf_wheel_speed    #右前轮轮速 m/s
float32 lr_wheel_speed    #左后轮轮速 m/s
float32 rr_wheel_speed    #右后轮轮速 m/s
float32 lf_wheel_angle    #左前轮转向角度 rad 
float32 rf_wheel_angle    #左后轮转向角度 rad
float32 linear_speed   #当前线速度m/s
float32 angular_speed  #当前角速度rad/s

float32 target_lf_wheel_speed    #左前轮轮速 m/s
float32 target_rf_wheel_speed    #右前轮轮速 m/s
float32 target_lr_wheel_speed    #左后轮轮速 m/s
float32 target_rr_wheel_speed    #右后轮轮速 m/s
float32 target_lf_wheel_angle    #左前轮转向角度 rad
float32 target_rf_wheel_angle    #左后轮转向角度 rad
float32 target_linear_speed        #m/s
float32 target_angular_speed       #rad/s

uint8  lf_wheel_stall       #0:未堵转 1:正向堵转 2:反向堵转
uint8  rf_wheel_stall       #0:未堵转 1:正向堵转 2:反向堵转
uint8  lr_wheel_stall       #0:未堵转 1:正向堵转 2:反向堵转
uint8  rr_wheel_stall       #0:未堵转 1:正向堵转 2:反向堵转

Header  header
