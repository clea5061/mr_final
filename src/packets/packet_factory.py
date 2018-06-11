from .packet import Packet
from .conn_packet import ConnPacket
from .conn_ack_packet import ConnAckPacket
from .control_packet import ControlPacket
from .ack_packet import AckPacket

class PacketFactory:
    packet_map = {
        ConnPacket.opcode: ConnPacket.Factory(),
        ConnAckPacket.opcode: ConnAckPacket.Factory(),
        AckPacket.opcode: AckPacket.Factory(),
        ControlPacket.opcode: ControlPacket.Factory()
    }

    @staticmethod
    def register_packet(opcode, factory):
        print(opcode)
        PacketFactory.packet_map[opcode] = factory

    @staticmethod
    def get_packet(opcode):
        if PacketFactory.packet_map[opcode]:
            return PacketFactory.packet_map[opcode].create()
