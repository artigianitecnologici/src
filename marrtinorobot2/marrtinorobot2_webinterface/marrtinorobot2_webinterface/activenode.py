import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Empty
import subprocess

class NodeCounter(Node):
    def __init__(self):
        super().__init__('node_counter')
        # Publisher per pubblicare il numero di nodi attivi
        self.publisher_ = self.create_publisher(Int32, 'nodes_count', 10)
        # Subscriber che ascolta il topic per ricevere il trigger
        self.subscription = self.create_subscription(Empty, 'trigger_count', self.publish_node_count, 10)

    def get_active_nodes(self):
        """Esegue il comando ros2 node list e ritorna il numero di nodi attivi"""
        try:
            # Utilizza subprocess per eseguire il comando 'ros2 node list'
            result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE)
            nodes_list = result.stdout.decode('utf-8').splitlines()
            return len(nodes_list)
        except Exception as e:
            self.get_logger().error(f"Errore durante l'esecuzione di ros2 node list: {str(e)}")
            return 0

    def publish_node_count(self, msg):
        """Pubblica il numero di nodi attivi solo quando riceve il trigger"""
        node_count = self.get_active_nodes()
        msg_to_publish = Int32()
        msg_to_publish.data = node_count
        self.publisher_.publish(msg_to_publish)
        self.get_logger().info(f'Pubblicato numero di nodi attivi: {node_count}')

def main(args=None):
    rclpy.init(args=args)
    node_counter = NodeCounter()

    try:
        rclpy.spin(node_counter)
    except KeyboardInterrupt:
        pass

    node_counter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
