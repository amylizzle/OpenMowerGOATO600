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
from mower_msgs.srv import MowerControlSrv, EmergencyStopSrv, MowerControlSrvResponse, EmergencyStopSrvResponse
try:
    from nmea_msgs.msg import Sentence
except ImportError:
    Sentence = None

# GOAT message types - imported if available
try:
    from ecovacs_messages.msg import WheelSpeedReport, SetLinearAngularSpeed
except ImportError:
    WheelSpeedReport = None
    SetLinearAngularSpeed = None

try:
    from ecovacs_messages.msg import MotorSpeedReport, LawnMowerMotor, MotorSpeedControl
except ImportError:
    MotorSpeedReport = None
    LawnMowerMotor = None
    MotorSpeedControl = None

try:
    from ecovacs_messages.msg import Battery, ChargeState, ChargeVolCur, BatteryInfo
except ImportError:
    Battery = None
    BatteryInfo = None
    ChargeState = None
    ChargeVolCur = None

try:
    from ecovacs_messages.msg import ImuSensor, GyroInfo, Geomag
except ImportError:
    ImuSensor = None
    GyroInfo = None
    Geomag = None

try:
    from ecovacs_messages.msg import EStopState, RainDetectState, OnOffInfo, OnOffSensorValue, EStopControl
except ImportError:
    EStopState = None
    RainDetectState = None
    OnOffInfo = None
    OnOffSensorValue = None
    EStopControl = None

try:
    from ecovacs_messages.msg import Gps
except ImportError:
    Gps = None

try:
    from ecovacs_messages.msg import RtkData
except ImportError:
    RtkData = None

try:
    from ecovacs_messages.msg import SendData, RecvData
except ImportError:
    SendData = None
    RecvData = None

try:
    from ecovacs_messages.msg import Pose, PredictPose
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

        # state - basically hold a message for each publisher and update it with
        # only the bits that change for each subscriber
        self.emergency = Emergency()
        self.twist = TwistStamped()
        self.left_esc = ESCStatus()
        self.right_esc = ESCStatus()
        self.power = Power()
        self.mower_status = Status()
        self.imu_msg = Imu()
        self.absolute_pose = AbsolutePose()
        self.set_wheel_speed_msg = SetLinearAngularSpeed() 
        self.lawn_mower_msg = LawnMowerMotor()
        self.motor_speed_msg = MotorSpeedControl() 
        self.send_data_msg = SendData()
        self.estop_control_msg = EStopControl() 

        self.current_charge_state = None
        self.current_rain = False
        self.current_estop = False

        # --- publishers: OpenMower topics (what OpenMower expects) ---
        self.pub_twist = rospy.Publisher('ll/diff_drive/measured_twist', TwistStamped, queue_size=10)
        self.pub_left_esc = rospy.Publisher('ll/diff_drive/left_esc_status', ESCStatus, queue_size=10)
        self.pub_right_esc = rospy.Publisher('ll/diff_drive/right_esc_status', ESCStatus, queue_size=10)
        self.pub_mower_status = rospy.Publisher('ll/mower_status', Status, queue_size=10)
        self.pub_emergency = rospy.Publisher('ll/emergency', Emergency, queue_size=10)
        self.pub_power = rospy.Publisher('ll/power', Power, queue_size=10)
        self.pub_imu = rospy.Publisher('ll/imu/data_raw', Imu, queue_size=10)
        self.pub_absolute_pose = rospy.Publisher('ll/position/gps', AbsolutePose, queue_size=10)

        # --- publishers: GOAT topics (what GOAT firmware expects) ---
        self.pub_set_wheel_speed = rospy.Publisher('/wheel/SetLinearAngularSpeed', SetLinearAngularSpeed, queue_size=10)
        self.pub_lawn_mower = rospy.Publisher('/motor/LawnMowerMotor', LawnMowerMotor, queue_size=10)
        self.pub_motor_speed = rospy.Publisher('/motor/MotorSpeedControl', MotorSpeedControl, queue_size=10)
        self.pub_send_data = rospy.Publisher('/comm/send_data', SendData, queue_size=10)
        self.pub_estop_control = rospy.Publisher('/onOffInfo/EStopControl', EStopControl, queue_size=10)

        # --- subscribers: GOAT topics ---
        rospy.Subscriber('/wheel/WheelSpeedReport', WheelSpeedReport, self.on_wheel_speed)
        rospy.Subscriber('/motor/MotorSpeedReport', MotorSpeedReport, self.on_motor_speed)
        rospy.Subscriber('/power/Battery', Battery, self.on_battery)
        rospy.Subscriber('/bigdata/BigDataBatteryInfo', BatteryInfo, self.on_battery_bigdata)
        rospy.Subscriber('/power/ChargeVolCur', ChargeVolCur, self.on_charge_vol_cur)
        rospy.Subscriber('/power/ChargeState', ChargeState, self.on_charge_state)
        rospy.Subscriber('/onOffInfo/EStopState', EStopState, self.on_estop)
        rospy.Subscriber('/onOffInfo/RainDetectState', RainDetectState, self.on_rain)
        rospy.Subscriber('/onOffInfo/OnOffInfo', OnOffInfo, self.on_on_off)
        rospy.Subscriber('/imu/ImuSensor', ImuSensor, self.on_imu)
        rospy.Subscriber('/rtk/rtkData', RtkData, self.on_gps)
        rospy.Subscriber('/gps/Gps', Gps, self.on_gps)

        # --- subscribers: OpenMower topics (commands going to GOAT) ---
        rospy.Subscriber('ll/cmd_vel', Twist, self.on_cmd_vel)
        rospy.Subscriber('xbot/action', String, self.on_xbot_action)

        # --- services: OpenMower low-level control (mower_logic clients) ---
        rospy.Service('ll/_service/mow_enabled', MowerControlSrv, self.on_mow_enabled)
        rospy.Service('ll/_service/emergency', EmergencyStopSrv, self.on_emergency)

        # publish an initial all-clear Emergency so mower_logic's StateSubscriber unblocks
        self.emergency.stamp = rospy.Time.now()
        self.emergency.active_emergency = False
        self.emergency.latched_emergency = False
        self.emergency.reason = ""
        self.pub_emergency.publish(self.emergency)

        rospy.loginfo("ecovacs_bridge: initialized")

    # --- GOAT → OpenMower translators ---

    def on_wheel_speed(self, msg):
        """Translate WheelSpeedReport → diff_drive topics"""
        now = rospy.Time.now()

        # TwistStamped
        self.twist.header.stamp = now
        self.twist.header.frame_id = 'base_link'
        self.twist.twist.linear.x = msg.linear_speed
        self.twist.twist.angular.z = msg.angular_speed
        self.pub_twist.publish(self.twist)

        # left ESC status
        self.left_esc.status = ESCStatus.ESC_STATUS_RUNNING if not msg.left_wheel_stall else ESCStatus.ESC_STATUS_STALLED
        self.left_esc.rpm = int(msg.left_wheel_speed * 60 / (math.pi * 0.2))  # rough RPM from m/s, wheel diam ~0.2m
        self.left_esc.current = 0.0
        self.left_esc.temperature_motor = 40.0 + (10.0 if msg.left_wheel_temp else 0.0)
        self.left_esc.temperature_pcb = 35.0
        self.pub_left_esc.publish(self.left_esc)

        # right ESC status
        self.right_esc.status = ESCStatus.ESC_STATUS_RUNNING if not msg.right_wheel_stall else ESCStatus.ESC_STATUS_STALLED
        self.right_esc.rpm = int(msg.right_wheel_speed * 60 / (math.pi * 0.2))
        self.right_esc.current = 0.0
        self.right_esc.temperature_motor = 40.0 + (10.0 if msg.right_wheel_temp else 0.0)
        self.right_esc.temperature_pcb = 35.0
        self.pub_right_esc.publish(self.right_esc)

    def on_motor_speed(self, msg):
        """Translate MotorSpeedReport → mower_status"""
        s = self.mower_status
        s.stamp = rospy.Time.now()
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

    def on_battery(self, msg):
        """Translate Battery → Power"""
        p = self.power
        p.stamp = rospy.Time.now()
        p.battery_pct = float(msg.battery)  # 0-100 %
        if msg.isLowVoltageToPowerOff:
            p.charger_status = "low_voltage_power_off" 
        self.pub_power.publish(p)

    def on_battery_bigdata(self, msg):
        """Translate BatteryInfo → Power"""
        p = self.power
        p.stamp = rospy.Time.now()
        p.battery_pct = float(msg.batteryLevel)  # 0-100 %
        p.battery_voltage = float(msg.batteryVoltage) / 1000.0  # mA
        p.dcdc_input_current = float(msg.batteryCurrent)
        self.pub_power.publish(p)        

    def on_charge_vol_cur(self, msg):
        """Translate ChargeVolCur → Power charge voltage/current"""
        p = self.power
        p.stamp = rospy.Time.now()
        p.charge_voltage = msg.chargeVol / 1000.0  # mV → V
        p.charge_current = msg.chargeCur / 1000.0  # mA → A
        p.charger_enabled = msg.chargeStep > 0
        if msg.chargeStep > 0:
            p.charger_status = "charging"
        self.pub_power.publish(p)

    def on_charge_state(self, msg):
        """Translate ChargeState → remember is_charging state"""
        self.current_charge_state = msg.isOnCharger > 0

    def on_estop(self, msg):
        """Translate EStopState → Emergency"""
        e = self.emergency
        e.stamp = rospy.Time.now()
        e.active_emergency = msg.state > 0
        e.latched_emergency = msg.state > 0
        e.reason = "e_stop" if msg.state > 0 else ""
        self.pub_emergency.publish(e)
        self.current_estop = msg.state > 0

    def on_rain(self, msg):
        """Remember rain state for mower_status"""
        self.current_rain = msg.value > 0

    def on_on_off(self, msg):
        """Translate OnOffInfo values → Emergency (bump/fall)"""
        if OnOffSensorValue is None:
            return
        for v in msg.values:
            if v.type == OnOffSensorValue.TYPE_BUMP or v.type == OnOffSensorValue.TYPE_FALL:
                if v.value > 0:
                    e = self.emergency
                    e.stamp = rospy.Time.now()
                    e.active_emergency = True
                    e.latched_emergency = True
                    e.reason = "bump" if v.type == OnOffSensorValue.TYPE_BUMP else "fall"
                    self.pub_emergency.publish(e)

    def on_imu(self, msg):
        """Translate ImuSensor → sensor_msgs/Imu"""
        imu = self.imu_msg
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
        """Translate RtkData/Gps → AbsolutePose (if available)"""
        if self.pub_absolute_pose is None:
            return
        ap = self.absolute_pose
        ap.header.stamp = rospy.Time.now()
        ap.header.frame_id = 'map'
        ap.source = AbsolutePose.SOURCE_GPS
        ap.flags = 2 # TODO!!! This just says all GPS data is legit, which it may not be
        ap.pose.pose.position.x = getattr(msg, 'lat', getattr(msg, 'latitude', 0.0))
        ap.pose.pose.position.y = getattr(msg, 'lon', getattr(msg, 'longitude', 0.0))
        ap.pose.pose.position.z = getattr(msg, 'alt', getattr(msg, 'altitude', 0.0))
        # orientation stays identity
        ap.pose.pose.orientation.x = 0
        ap.pose.pose.orientation.y = 0
        ap.pose.pose.orientation.z = 0
        ap.pose.pose.orientation.w = 1
        self.pub_absolute_pose.publish(ap)

    # --- OpenMower → GOAT translators ---

    def on_mow_enabled(self, req):
        """Service: ll/_service/mow_enabled → LawnMowerMotor + MotorSpeedControl"""
        if req.mow_enabled:
            if self.pub_lawn_mower is not None:
                l = self.lawn_mower_msg
                l.type = 0  # ROLL_MOTOR
                l.value = LawnMowerMotor.ROLL_MOTOR_MAX_VALUE
                l.pubName = ''
                self.pub_lawn_mower.publish(l)
            if self.pub_motor_speed is not None:
                mc = self.motor_speed_msg
                mc.val = 1
                mc.speedleft = 1.0
                mc.speedright = 1.0
                mc.runtime = 0.0
                self.pub_motor_speed.publish(mc)
        else:
            if self.pub_lawn_mower is not None:
                l = self.lawn_mower_msg
                l.type = 0
                l.value = 0
                l.pubName = ''
                self.pub_lawn_mower.publish(l)
        return MowerControlSrvResponse()

    def on_emergency(self, req):
        """Service: ll/_service/emergency → EStopControl"""
        if self.pub_estop_control is None:
            return EmergencyStopSrvResponse()
        ec = self.estop_control_msg
        ec.action = 0 if req.emergency else 1  # 0=trigger stop, 1=cancel
        self.pub_estop_control.publish(ec)
        return EmergencyStopSrvResponse()

    def on_cmd_vel(self, msg):
        """Translate Twist → SetLinearAngularSpeed"""
        if self.pub_set_wheel_speed is not None:
            s = self.set_wheel_speed_msg
            s.linear_speed = msg.linear.x
            s.angular_speed = msg.angular.z
            s.pubName = ''
            self.pub_set_wheel_speed.publish(s)

    def on_xbot_action(self, msg):
        """Translate action string → LawnMowerMotor or MotorSpeedControl"""
        data = msg.data

        if data.startswith('mow'):
            # enable/disable mowing motor
            if self.pub_lawn_mower is not None:
                l = self.lawn_mower_msg
                l.type = 0  # ROLL_MOTOR
                l.value = LawnMowerMotor.ROLL_MOTOR_MAX_VALUE if 'on' in data or 'start' in data else 0
                l.pubName = ''
                self.pub_lawn_mower.publish(l)
        elif data.startswith('speed'):
            # set cutting speed
            if self.pub_motor_speed is not None:
                mc = self.motor_speed_msg
                mc.val = 1
                mc.speedleft = 3000.0
                mc.speedright = 3000.0
                mc.runtime = 0.0
                self.pub_motor_speed.publish(mc)

    def on_rtcm(self, msg):
        """Translate RTCM correction → GOAT comm send_data"""
        if self.pub_send_data is not None and SendData is not None:
            sd = self.send_data_msg
            sd.data = msg.data
            self.pub_send_data.publish(sd)


if __name__ == '__main__':
    try:
        EcovacsBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
