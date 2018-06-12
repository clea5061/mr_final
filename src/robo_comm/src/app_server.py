#!/usr/bin/env python

import rospy
import socket
import threading
import sys
import SocketServer
from packets.packet_factory import PacketFactory
from packet_processor import PacketProcessor
from packets.packet import Packet
from packets import ack_packet, conn_ack_packet
import packets.packet_util as putil

class ThreadedTCPRequestHandler(SocketServer.BaseRequestHandler):
    active = True
    def __init__(self, request, client_address, server):
        self.processor = PacketProcessor.get_instance()
        SocketServer.BaseRequestHandler.__init__(self, request, client_address, server)
        pass

    def handle(self):
        while ThreadedTCPRequestHandler.active:
            data = self.request.recv(4)
            if not data:
                break
            print(data)
            try:
                packet = PacketFactory.get_packet(data)
                packet.read_packet(self.request)
                print(packet)
                if not putil.is_ack(packet):
                    if not putil.is_conn(packet):
                        ack = ack_packet.AckPacket(packet.get_id())
                        ack.write_packet(self.request)
                    else:
                        ack = conn_ack_packet.ConnAckPacket(packet.name)
                        ack.write_packet(self.request)
                else:
                    print("Ack:", str(packet.get_id()))
                self.processor.process(packet, self.request)
            except:
                print("Unexpected error:", sys.exc_info()[0])
        pass

class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    pass

class AppServer():
    def __init__(self):
        HOST, PORT = "0.0.0.0", 4665
        self.server = ThreadedTCPServer((HOST,PORT), ThreadedTCPRequestHandler)
        self.ip, self.port = self.server.server_address

        self.server_thread = threading.Thread(target = self.server.serve_forever)
        self.server_thread.daemon = True

    def start(self):
        if not self.server_thread.is_alive():
            self.server_thread.start()
            print("Server started in thread: ", self.server_thread.name)
        else:
            print("Server is already active in thread: ", self.server_thread.name)

    def stop(self):
        if self.server_thread.is_alive():
            ThreadedTCPRequestHandler.active = False
            self.server.shutdown()
            self.server.server_close()
            self.server_thread.join()
            print("Server has stopped.")


if __name__ == "__main__":
    rospy.init_node("robo_comm")
    app_serve = AppServer()
    app_serve.start()
    #input("Press Enter to continue...")
    rospy.spin()
    app_serve.stop()
