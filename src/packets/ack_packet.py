from .abstract_packet import AbstractPacket
from struct import pack, unpack

class AckPacket(AbstractPacket):
    opcode = b'\x05AC\x18'
    def __init__(self, ack_id):
        super(AckPacket, self).__init__(AckPacket.opcode)
        self.ack_id = ack_id

    def write_packet(self, output):
        self.packet_length = 4
        super(AckPacket, self).write_packet(output)
        output.send(pack('!I', self.ack_id))
    
    def read_packet(self, instream):
        super(AckPacket, self).read_packet(instream)
        b1 = instream.recv(4)
        self.ack_id = unpack('!I', b1)

    class Factory:
        def create(self):
            return AckPacket('')
    
