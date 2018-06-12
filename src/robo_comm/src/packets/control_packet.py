from .abstract_packet import AbstractPacket
from struct import unpack
#import rospy
#import std_msgs

class ControlPacket(AbstractPacket):
    opcode = b'\x05ST\x18'
    #publisher = rospy.Publisher('control_packet', std_msgs.msg.Char, queue_size=10)
    class Factory:
        def create(self):
            return ControlPacket()
        
    def __init__(self):
        super(ControlPacket, self).__init__(ControlPacket.opcode)

    def read_packet(self, instream):
        print('in control packet')
        super(ControlPacket, self).read_packet(instream)
        b1 = bytearray(self.packet_length)
        instream.recv_into(b1, self.packet_length)
        self.controls, _ = unpack("!h", b1)

    def process_packet(self):
        #ControlPacket.publisher.publish(self.controls)
        pass