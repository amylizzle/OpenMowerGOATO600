#!/usr/bin/env python3
"""
Bridge between Ecovacs GOAT native ROS topics and OpenMower topics.
Subscribes to GOAT hardware topics, translates messages, and publishes on OpenMower topics,
and vice versa.
"""

import math

import rospy
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Header
from mower_msgs.msg import Emergency, ESCStatus, Status, Power
try:
    from nmea_msgs.msg import Sentence
except ImportError:
    Sentence = None

# GOAT message types - imported if available
try:
    from ecovacs_messages.wheel.msg import WheelSpeedReport, SetWheelSpeed
except ImportError:
    WheelSpeedReport = None
    SetWheelSpeed = None

try:
    from ecovacs_messages.motor.msg import MotorSpeedReport, LawnMowerMotor, MotorSpeedControl
except ImportError:
    MotorSpeedReport = None
    LawnMowerMotor = None
    MotorSpeedControl = None

try:
    from ecovacs_messages.power.msg import BatteryInfo, Battery, ChargeState
except ImportError:
    BatteryInfo = None
    Battery = None
    ChargeState = None

try:
    from ecovacs_messages.imu.msg import ImuSensor, GyroInfo, Geomag
except ImportError:
    ImuSensor = None
    GyroInfo = None
    Geomag = None

try:
    from on_off_info.msg import EStopState, RainDetectState, BumpValue, DownInValue, FallValue
except ImportError:
    EStopState = None
    RainDetectState = None
    BumpValue = None
    DownInValue = None
    FallValue = None

try:
    from ecovacs_messages.gps.msg import Gps
except ImportError:
    Gps = None

try:
    from ecovacs_messages.comm.msg import SendData, RecvData
except ImportError:
    SendData = None
    RecvData = None

try:
    from ecovacs_messages.prediction.msg import Pose, PredictPose
except ImportError:
    Pose = None
    PredictPose = None

try:
    from xbot_msgs.msg import AbsolutePose
except ImportError:
    AbsolutePose = None


class EcovacsBridgeNode:
    def __init__(self):
        rospy.init_node('ecovacs_bridge', anonymous=False)

        # --- publishers: OpenMower topics (what OpenMower expects) ---
        self.pub_twist = rospy.Publisher('ll/diff_drive/measured_twist', TwistStamped, queue_size=10)
        self.pub_left_esc = rospy.Publisher('ll/diff_drive/left_esc_status', ESCStatus, queue_size=10)
        self.pub_right_esc = rospy.Publisher('ll/diff_drive/right_esc_status', ESCStatus, queue_size=10)
        self.pub_mower_status = rospy.Publisher('ll/mower_status', Status, queue_size=10)
        self.pub_emergency = rospy.Publisher('ll/emergency', Emergency, queue_size=10)
        self.pub_power = rospy.Publisher('ll/power', Power, queue_size=10)
        self.pub_imu = rospy.Publisher('ll/imu/data_raw', Imu, queue_size=10)

        if AbsolutePose is not None:
            self.pub_absolute_pose = rospy.Publisher('ll/position/gps', AbsolutePose, queue_size=10)
        else:
            self.pub_absolute_pose = None

        # --- publishers: GOAT topics (what GOAT firmware expects) ---
        if SetWheelSpeed is not None:
            self.pub_set_wheel_speed = rospy.Publisher('/wheel/set_linear_angular_speed', SetWheelSpeed, queue_size=10)
        else:
            self.pub_set_wheel_speed = None

        if LawnMowerMotor is not None:
            self.pub_lawn_mower = rospy.Publisher('/motor/lawn_mower_motor', LawnMowerMotor, queue_size=10)
        else:
            self.pub_lawn_mower = None

        if MotorSpeedControl is not None:
            self.pub_motor_speed = rospy.Publisher('/motor/motor_speed_control', MotorSpeedControl, queue_size=10)
        else:
            self.pub_motor_speed = None

        if SendData is not None:
            self.pub_send_data = rospy.Publisher('/comm/send_data', SendData, queue_size=10)
        else:
            self.pub_send_data = None

        # --- subscribers: GOAT topics ---
        if WheelSpeedReport is not None:
            rospy.Subscriber('/wheel/wheel_speed_report', WheelSpeedReport, self.on_wheel_speed)
        else:
            rospy.logwarn("ecovacs_bridge: wheel.msg.WheelSpeedReport not available")

        if MotorSpeedReport is not None:
            rospy.Subscriber('/motor/motor_speed_report', MotorSpeedReport, self.on_motor_speed)
        else:
            rospy.logwarn("ecovacs_bridge: motor.msg.MotorSpeedReport not available")

        if BatteryInfo is not None:
            rospy.Subscriber('/power/battery_info', BatteryInfo, self.on_battery_info)
        else:
            rospy.logwarn("ecovacs_bridge: power.msg.BatteryInfo not available")

        if ChargeState is not None:
            rospy.Subscriber('/power/charge_state', ChargeState, self.on_charge_state)
        else:
            rospy.logwarn("ecovacs_bridge: power.msg.ChargeState not available")

        if EStopState is not None:
            rospy.Subscriber('/on_off_info/e_stop_state', EStopState, self.on_estop)
        else:
            rospy.logwarn("ecovacs_bridge: on_off_info.msg.EStopState not available")

        if RainDetectState is not None:
            rospy.Subscriber('/on_off_info/rain_detect_state', RainDetectState, self.on_rain)
        else:
            rospy.logwarn("ecovacs_bridge: on_off_info.msg.RainDetectState not available")

        if ImuSensor is not None:
            rospy.Subscriber('/imu/imu_sensor', ImuSensor, self.on_imu)
        else:
            rospy.logwarn("ecovacs_bridge: imu.msg.ImuSensor not available")

        if Gps is not None:
            rospy.Subscriber('/gps/gps', Gps, self.on_gps)
        else:
            rospy.logwarn("ecovacs_bridge: gps.msg.Gps not available")

        # --- subscribers: OpenMower topics (commands going to GOAT) ---
        rospy.Subscriber('ll/cmd_vel', Twist, self.on_cmd_vel)
        rospy.Subscriber('xbot/action', String, self.on_xbot_action)

        if Sentence is not None:
            rospy.Subscriber('ll/position/gps/rtcm', Sentence, self.on_rtcm)
        elif Sentence is None:
            # try generic std_msgs/String or raw
            pass

        # state
        self.current_charge_state = None
        self.current_rain = False
        self.current_estop = False

        rospy.loginfo("ecovacs_bridge: initialized")

    # --- GOAT → OpenMower translators ---

    def on_wheel_speed(self, msg):
        """Translate WheelSpeedReport → diff_drive topics"""
        now = rospy.Time.now()

        # TwistStamped
        t = TwistStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.twist.linear.x = msg.linear_speed
        t.twist.angular.z = msg.angular_speed
        self.pub_twist.publish(t)

        # left ESC status
        left = ESCStatus()
        left.status = ESCStatus.ESC_STATUS_RUNNING if not msg.left_wheel_stall else ESCStatus.ESC_STATUS_STALLED
        left.rpm = int(msg.left_wheel_speed * 60 / (math.pi * 0.2))  # rough RPM from m/s, wheel diam ~0.2m
        left.current = 0.0
        left.temperature_motor = 40.0 + (10.0 if msg.left_wheel_temp else 0.0)
        left.temperature_pcb = 35.0
        self.pub_left_esc.publish(left)

        # right ESC status
        right = ESCStatus()
        right.status = ESCStatus.ESC_STATUS_RUNNING if not msg.right_wheel_stall else ESCStatus.ESC_STATUS_STALLED
        right.rpm = int(msg.right_wheel_speed * 60 / (math.pi * 0.2))
        right.current = 0.0
        right.temperature_motor = 40.0 + (10.0 if msg.right_wheel_temp else 0.0)
        right.temperature_pcb = 35.0
        self.pub_right_esc.publish(right)

    def on_motor_speed(self, msg):
        """Translate MotorSpeedReport → mower_status"""
        s = Status()
        s.header.stamp = rospy.Time.now()
        s.mower_status = Status.MOWER_STATUS_OK
        s.raspberry_pi_power = True
        s.is_charging = self.current_charge_state if self.current_charge_state is not None else False
        s.esc_power = True
        s.rain_detected = self.current_rain
        s.mow_enabled = True

        # mower ESC status from cut stall flags
        if msg.left_cut_stall or msg.right_cut_stall:
            s.mower_esc_status = ESCStatus.ESC_STATUS_STALLED
        else:
            s.mower_esc_status = ESCStatus.ESC_STATUS_RUNNING

        s.mower_esc_temperature = 40.0
        if msg.left_cut_temp_alarm or msg.right_cut_temp_alarm:
            s.mower_esc_temperature = 55.0

        s.mower_esc_current = 0.5
        s.mower_motor_temperature = 45.0
        s.mower_motor_rpm = float(abs(msg.speed))
        self.pub_mower_status.publish(s)

    def on_battery_info(self, msg):
        """Translate BatteryInfo → Power"""
        p = Power()
        p.header.stamp = rospy.Time.now()
        p.battery_voltage = msg.batteryVoltage / 1000.0  # mV → V
        p.battery_pct = 0.0  # not provided by BatteryInfo alone
        p.charge_voltage = msg.chargeVoltage / 1000.0
        p.charge_current = msg.batteryCurrent / 1000.0  # mA → A
        p.charger_enabled = msg.chargeStatus > 0
        p.charger_status = str(msg.chargeStatus)
        self.pub_power.publish(p)

    def on_charge_state(self, msg):
        """Translate ChargeState → remember is_charging state"""
        self.current_charge_state = msg.isOnCharger

    def on_estop(self, msg):
        """Translate EStopState → Emergency"""
        e = Emergency()
        e.stamp = rospy.Time.now()
        e.active_emergency = msg.state > 0
        e.latched_emergency = msg.state > 0
        e.reason = "e_stop" if msg.state > 0 else ""
        self.pub_emergency.publish(e)
        self.current_estop = msg.state > 0

    def on_rain(self, msg):
        """Remember rain state for mower_status"""
        self.current_rain = msg.value > 0

    def on_imu(self, msg):
        """Translate ImuSensor → sensor_msgs/Imu"""
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = 'base_imu_link'

        # angular velocity: gyros are usually rad/s
        if len(msg.gyros) >= 3:
            imu.angular_velocity.x = msg.gyros[0]
            imu.angular_velocity.y = msg.gyros[1]
            imu.angular_velocity.z = msg.gyros[2]

        # linear acceleration: accs are usually m/s²
        if len(msg.accs) >= 3:
            imu.linear_acceleration.x = msg.accs[0]
            imu.linear_acceleration.y = msg.accs[1]
            imu.linear_acceleration.z = msg.accs[2]

        # orientation not provided by raw IMU
        imu.orientation_covariance[0] = -1  # unknown
        self.pub_imu.publish(imu)

    def on_gps(self, msg):
        """Translate Gps → AbsolutePose (if available)"""
        if self.pub_absolute_pose is None:
            return
        ap = AbsolutePose()
        ap.header.stamp = rospy.Time.now()
        ap.header.frame_id = 'map'
        ap.source = AbsolutePose.SOURCE_GPS
        ap.flags = 0
        ap.pose.pose.position.x = msg.latitude
        ap.pose.pose.position.y = msg.longitude
        ap.pose.pose.position.z = msg.altitude
        # orientation stays identity
        ap.pose.pose.orientation.x = 0
        ap.pose.pose.orientation.y = 0
        ap.pose.pose.orientation.z = 0
        ap.pose.pose.orientation.w = 1
        self.pub_absolute_pose.publish(ap)

    # --- OpenMower → GOAT translators ---

    def on_cmd_vel(self, msg):
        """Translate Twist → SetWheelSpeed or SetLinearAngularSpeed"""
        if self.pub_set_wheel_speed is not None:
            s = SetWheelSpeed()
            s.speed_type = SetWheelSpeed.SPEED_TYPE_DRIVING
            s.pubName = ''
            s.speed = [msg.linear.x, msg.angular.z]
            self.pub_set_wheel_speed.publish(s)

    def on_xbot_action(self, msg):
        """Translate action string → LawnMowerMotor or MotorSpeedControl"""
        data = msg.data

        if data.startswith('mow'):
            # enable/disable mowing motor
            if self.pub_lawn_mower is not None:
                l = LawnMowerMotor()
                l.type = 0  # ROLL_MOTOR
                l.value = LawnMowerMotor.ROLL_MOTOR_MAX_VALUE if 'on' in data or 'start' in data else 0
                l.pubName = ''
                self.pub_lawn_mower.publish(l)
        elif data.startswith('speed'):
            # set cutting speed
            if self.pub_motor_speed is not None:
                mc = MotorSpeedControl()
                mc.val = 1
                mc.speedleft = 3000.0
                mc.speedright = 3000.0
                mc.runtime = 0.0
                self.pub_motor_speed.publish(mc)

    def on_rtcm(self, msg):
        """Translate RTCM correction → GOAT comm send_data"""
        if self.pub_send_data is not None and SendData is not None:
            sd = SendData()
            sd.data = msg.data
            self.pub_send_data.publish(sd)


if __name__ == '__main__':
    try:
        EcovacsBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
