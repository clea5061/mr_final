from .ack_packet import AckPacket
from .conn_packet import ConnPacket


def is_ack(packet):
    return type(packet) is AckPacket

def is_conn(packet):
    return type(packet) is ConnPacket