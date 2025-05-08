import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray

import socket

class Networking(Node):
    def __init__(self):
        super().__init__('networking')
        self.ip = "0.0.0.0"
        self.port = 5006
        self.remote = "192.168.0.169"
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        self.incoming = self.create_publisher(ByteMultiArray, "/networking_incoming", 10)
        self.create_subscription(ByteMultiArray, "/networking_outgoing", self.net_send, 10)
        self.get_logger().info("UDP communication on " + self.ip + " on port " + str(self.port) + " with " + self.remote)
        self.create_timer(0.01, self.net_callback)
    
    def net_callback(self):
        recv = self.sock.recvfrom(1024)[0]
        pub = ByteMultiArray()
        pub.data = [bytes([x]) for x in recv]
        self.incoming.publish(pub)

    def net_send(self, msg: ByteMultiArray):
        send = b''.join(msg.data)
        self.sock.sendto(send, (self.remote, self.port))
    
    def destroy_node(self):
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Networking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
