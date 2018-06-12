from .packet import Packet
from struct import pack, unpack_from
from socket import socket
import random

class AbstractPacket(Packet):
    packet_id = None
    packet_length = 0
    opcode = None

    def __init__(self, opcode):
        self.packet_id = random.randint(0, 10000000)
        self.opcode = opcode

    def get_id(self):
        return self.packet_id

    def get_length(self):
        return self.packet_length

    def write_packet(self, output):
        output.send(self.opcode)
        output.send(pack('!II', self.packet_id, self.packet_length))

    def read_in_size(self, instream, size):
        b1 = b''
        recv = 0
        while recv < size:
            dat = instream.recv(4)
            if len(dat) > 0:
                b1 += dat
                recv += len(dat)
        return b1

    def read_packet(self, instream):
        b1 = self.read_in_size(instream, 8)
        self.packet_id, self.packet_length = unpack_from('!II', b1)

    def process_packet(self):
        raise NotImplementedError

    class Factory:
        def create(self):
            raise NotImplementedError


