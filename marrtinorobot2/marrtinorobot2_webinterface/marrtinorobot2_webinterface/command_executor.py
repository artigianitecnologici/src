# Copyright 2025 robotics-3d.com 
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Ferrarini Fabio
# Email : ferrarini09@gmail.com
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class CommandExecutorNode(Node):
    def __init__(self):
        super().__init__('command_executor')
        self.subscription = self.create_subscription(
            String,
            'shell_command',
            self.execute_command_callback,
            10)
        self.publisher = self.create_publisher(String, 'command_output', 10)

    def execute_command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        try:
            # Esegue il comando shell
            result = subprocess.run(command, shell=True, capture_output=True, text=True)
            output = result.stdout + result.stderr
        except Exception as e:
            output = f'Error: {str(e)}'
        
        # Pubblica l'output del comando
        output_msg = String()
        output_msg.data = output
        self.publisher.publish(output_msg)
        self.get_logger().info(f'Command output: {output}')

def main(args=None):
    rclpy.init(args=args)
    node = CommandExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
