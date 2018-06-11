from .abstract_packet import AbstractPacket

class ConnAckPacket(AbstractPacket):
    class Factory:
        def create(self):
            return ConnAckPacket('')

    opcode = b'\x05CA\x18'

    def __init__(self, name):
        super(ConnAckPacket, self).__init__(ConnAckPacket.opcode)
        self.name = name

    def write_packet(self, output):
        self.packet_length = len(self.name)
        super(ConnAckPacket, self).write_packet(output)
        output.sendall(self.name.encode("ascii"))
    
    def read_packet(self, instream):
        super(ConnAckPacket, self).read_packet(instream)
        b1 = bytearray(self.packet_length)
        instream.recv_into(b1, self.packet_length)
        self.name = b1.decode('ascii')
    
    def __str__(self):
        return str(self.packet_length)+': '+self.name