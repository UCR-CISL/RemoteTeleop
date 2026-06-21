import socket
import time

class UDPTimestampSender():
    def __init__(self, ip: str, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_addr = (ip, port)

    def send_timestamp(self, timestamp_msg: str):
        self.sock.sendto(timestamp_msg.encode(), self.target_addr)