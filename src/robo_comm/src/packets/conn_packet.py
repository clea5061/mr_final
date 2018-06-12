from .abstract_packet import AbstractPacket

class ConnPacket(AbstractPacket):
    class Factory:
        def create(self):
            return ConnPacket('')

    name = None
    opcode = b'\x05CN\x18'

    def __init__(self, name):
        super().__init__(ConnPacket.opcode)
        self.name = name

    def write_packet(self, output):
        self.packet_length = len(self.name)
        super(ConnPacket, self).write_packet(output)
        output.sendall(self.name)
    
    def read_packet(self, instream):
        super().read_packet(instream)
        b1 = bytearray(self.packet_length)
        instream.recv_into(b1, self.packet_length)
        self.name = b1.decode('ascii')
    
    def __str__(self):
        return str(self.packet_length)+': '+self.name