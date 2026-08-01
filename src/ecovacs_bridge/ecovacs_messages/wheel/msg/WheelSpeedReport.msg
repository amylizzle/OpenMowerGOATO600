float32 left_wheel_speed    #m/s
float32 right_wheel_speed   #m/s
float32 linear_speed        #m/s
float32 angular_speed       #rad/s
float32 target_left_wheel_speed    #m/s
float32 target_right_wheel_speed   #m/s
float32 target_linear_speed        #m/s
float32 target_angular_speed       #rad/s
uint8   left_wheel_stall           #0:未堵转 1:正向堵转 2:反向堵转
uint8   right_wheel_stall          #0:未堵转 1:正向堵转 2:反向堵转
uint8   left_wheel_temp            #0:正常，1:机器正向前进时温度预告警 2:机器反向后退时温度预告警
uint8   right_wheel_temp           #0:正常，1:机器正向前进时温度预告警 2:机器反向后退时温度预告警

Header  header
