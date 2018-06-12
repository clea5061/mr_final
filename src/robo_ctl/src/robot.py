import serial
import rospy
from geometry_msgs.msg import Twist
from threading import Lock

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
        self.emergency = state & 0x20

class Robot(object):
    
    def __init__(self):
        self.ser_lock = Lock()
        self.robo_pub = rospy.Publisher("robo_ctl/control", Twist, queue_size=10)
        self.autonomous = False
        self.robo = serial.Serial('/dev/ttyACM0', 57600)
        rospy.Subscriber("robo_comm/control", self.control_callback)
        rospy.Subscriber("robo_ctl/control", self.robo_command_callback)

    def control_to_twist(self, control):
        twst = Twist()
        twst.linear.x = 0
        twst.linear.z = 0
        twst.angular.z = 0
        twst.linear.y = control.speed
        vl = vr = tst.linear.y
        if control.emergency:
            vl=vr=0
        elif control.left:
            if control.forward:
                vl *= 0.7
            elif control.reverse:
                vr *= 0.7
            else:
                vl *= -1
        elif control.right:
            if control.forward:
                vr *=0.7
            elif control.reverse:
                vl *= 0.7
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

    def robo_command_callback(self, data):
        twst = data.data
        dat = '!{0},{1},{2}\n'.format(twst.linear.y, twst.angular.x, twst.angular.y)
        rospy.loginfo(dat)
        #with self.ser_lock:
        #    self.robo.write(dat.encode())