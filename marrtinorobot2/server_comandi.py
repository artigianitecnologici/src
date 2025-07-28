import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BridgeServer(Node):
    def __init__(self):
        super().__init__('bridge_server')
        self.publisher = self.create_publisher(String, '/comando_robot', 10)

    def avvia_server(self):
        HOST = ''  # tutte le interfacce
        PORT = 12345
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen(1)
            self.get_logger().info(f"Server in ascolto sulla porta {PORT}")
            conn, addr = s.accept()
            with conn:
                self.get_logger().info(f"Connessione da {addr}")
                while rclpy.ok():
                    data = conn.recv(1024).decode().strip()
                    if not data:
                        break
                    msg = String()
                    msg.data = data
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Ricevuto e pubblicato: {data}")

def main(args=None):
    rclpy.init(args=args)
    server = BridgeServer()
    server.avvia_server()
    rclpy.shutdown()
