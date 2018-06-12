from packets.packet import Packet
from packets.packet_factory import PacketFactory
import threading
from packets import ack_packet, conn_ack_packet, control_packet
import packets.packet_util as putil

class SocketProcess():

    def read_socket(self):
        print(self.socket)
        while self.active:
            data = self.socket.recv(4)
            if len(data) == 0:
                continue
            print(data)
            packet: Packet = PacketFactory.get_packet(data)
            packet.read_packet(self.socket)
            print(packet)
            if not putil.is_ack(packet):
                if not putil.is_conn(packet):
                    ack = ack_packet.AckPacket(packet.get_id())
                    ack.write_packet(self.socket)
                else:
                    ack = conn_ack_packet.ConnAckPacket(packet.name)
                    ack.write_packet(self.socket)
            else:
                print("Ack:", str(packet.get_id))
            self.processor.process(packet, self)


    def __init__(self, socket, processor):
        self.processor = processor
        self.socket = socket
        self.active = True
        self.read_thread = threading.Thread(target=self.read_socket)
        self.read_thread.setDaemon(True)
        self.read_thread.start()

    def stop(self):
        self.active = False
        self.read_thread.join()


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
            print(packet.controls)
        pass