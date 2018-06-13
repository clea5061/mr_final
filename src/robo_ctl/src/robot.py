import serial
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Char
from threading import Lock
from vision import lane_center_to_twist

class ControlState():
    def __init__(self):
        pass

    def parse_incoming(self, state):
        self.speed = state & 0xFF
        state = state >> 16
        self.left = state & 0x1
        self.right = state & 0x2
        self.forward = state & 0x4
        self.reverse = state & 0x8
        self.autonomous = state & 0x10
        self.emergency = state & 0x20

class Robot(object):
    
    def __init__(self):
        self.ser_lock = Lock()
        self.auto_lock = Lock()
        self.light_lock = Lock()
        self.robo_pub = rospy.Publisher("robo_ctl/control", Twist, queue_size=10)
        self.autonomous = False
        self.light_state = 1 #Assume green
        self.robo = serial.Serial('/dev/ttyACM0', 57600)
        if self.robo:
            rospy.loginfo('Connected to robot')
        rospy.Subscriber("robo_comm/control", Int32, self.control_callback)
        rospy.Subscriber("robo_ctl/control", Twist, self.robo_command_callback)
        rospy.Subscriber("vision/center", Int32, self.center_callback)
        rospy.Subscriber("vision/light", Char, self.light_callback)

    def control_to_twist(self, control):
        twst = Twist()
        twst.linear.x = 0
        twst.linear.z = 0
        twst.angular.z = 0
        twst.linear.y = control.speed
        vl = vr = twst.linear.y
        if control.emergency:
            vl=vr=0
        elif control.left:
            if control.forward:
                vl *= 0.7
            elif control.reverse:
                vr *= -0.7
                vl *= -1
            else:
                vl *= -1
        elif control.right:
            if control.forward:
                vr *=0.7
            elif control.reverse:
                vl *= -0.7
                vr *= -1
            else:
                vr *= -1
        elif control.reverse:
            vr = vl = -vl
        elif control.forward:
            vr = vl = control.speed
        else:
            vr = vl = 0
        twst.angular.x = vr
        twst.angular.y = vl
        self.robo_pub.publish(twst)

    def control_callback(self, data):
        cs = ControlState()
        cs.parse_incoming(data.data)
        self.control_state = cs
        with self.auto_lock:
            self.autonomous = cs.autonomous
        if cs.emergency:
            self.autonomous = False
        if self.autonomous:
            return
        self.control_to_twist(cs)

    def light_callback(self, data):
        with self.auto_lock:
            if not self.autonomous:
                return
        ls = chr(data.data)
        with self.light_lock:
            if ls == 'R':
                self.light_state = 0
                self.stop()
            elif ls == 'G':
                self.light_state = 1
            elif ls == 'N' and self.light_state <> 1:
                self.light_state = 2
                self.stop()

    def center_callback(self, data):
        with self.auto_lock:
            with self.light_lock:
                if not self.autonomous or (self.light_state == 0 or self.light_state == 2):
                    return
        speed = self.control_state.speed
        twst = lane_center_to_twist(data.data, speed)
        self.robo_pub.publish(twst)

    def stop(self):
        twst = Twist()
        twst.linear.y = 0
        twst.angular.x = 0
        twst.angular.y = 0
        self.robo_pub.publish(twst)

    def robo_command_callback(self, data):
        twst = data
        dat = '!{0:0.1f},{1:0.1f},{2:0.1f}\n'.format(twst.linear.y, twst.angular.x, twst.angular.y)
        with self.ser_lock:
            self.robo.write(dat.encode())
