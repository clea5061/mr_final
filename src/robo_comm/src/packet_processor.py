from packets.packet import Packet
from packets.packet_factory import PacketFactory
import threading
from packets import ack_packet, conn_ack_packet, control_packet
import packets.packet_util as putil

class PacketProcessor():
    instance = None

    def setup_socket(self, socket):
        print(socket)
        SocketProcess(socket, self)

    @staticmethod
    def get_instance():
        if not PacketProcessor.instance is None:
            return PacketProcessor.instance
        else:
            PacketProcessor.instance = PacketProcessor()
            return PacketProcessor.instance

    def process(self, packet, socket_processor):
        if type(packet) is control_packet.ControlPacket:
            packet.process_packet()
        pass
