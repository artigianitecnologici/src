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
# file : node_asr.py
#  descrizione : publish /asr  called from chatbot.py
#
#! /usr/bin/python3
from flask import Flask, request, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

app = Flask(__name__)

# ROS2 Node class to handle publishing
class ASRPublisherNode(Node):
    def __init__(self):
        super().__init__('asr_publisher_node')
        self.TOPIC_speech = "/speech/to_speak"
        self.TOPIC_speech_status = "/speech/status"
        self.TOPIC_asr = "/asr"
        self.TOPIC_language = "/speech/language"
        self.TOPIC_robotcommand = "/robot_command"
        self.asr_pub = self.create_publisher(String, self.TOPIC_asr, 10)
        self.speech_pub = self.create_publisher(String, self.TOPIC_speech, 10)
        self.language_pub = self.create_publisher(String, self.TOPIC_language, 10)
        self.command_pub = self.create_publisher(String, self.TOPIC_robotcommand, 10)
        self.get_logger().info('ASR Publisher v.1.0 Node has been started')

    def publish_asr(self, text: str):
        msg = String()
        msg.data = text
        self.asr_pub.publish(msg)
        self.get_logger().info(f'Published text on /asr: "{text}"')

    def publish_speech(self, text: str):
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)
        self.get_logger().info(f'Published  on /speech/to_speak: "{text}"')

    def publish_language(self, text: str):
        msg = String()
        msg.data = text
        self.language_pub.publish(msg)
        self.get_logger().info(f'Published  on /speech/language: "{text}"')
    
    def publish_command(self, text: str):
        msg = String()
        msg.data = text
        self.command_pub.publish(msg)
        self.get_logger().info(f'Published  on /robot_command: "{text}"')

   
# Initialize ROS2 in a separate thread
def ros2_init():
    rclpy.init()
    global ros_node
    ros_node = ASRPublisherNode()
    rclpy.spin(ros_node)

ros2_thread = threading.Thread(target=ros2_init, daemon=True)
ros2_thread.start()

# Flask endpoint to receive and publish text
@app.route('/asr', methods=['POST'])
def asr():
    data = request.json
    if not data or 'text' not in data:
        return jsonify({"error": "Text not provided"}), 400
    
    text = data['text']
    #print(f"Text received from client: {text}")
    
    # Publish to the ROS2 topic /asr
    ros_node.publish_asr(text)
    
    return jsonify({"status": "success", "message": f"Text received and published: {text}"}), 200

@app.route('/speech', methods=['POST'])
def speech():
    data = request.json
    if not data or 'text' not in data:
        return jsonify({"error": "Text not provided"}), 400
    
    text = data['text']
    #print(f"Text received from client: {text}")
    
    # Publish to the ROS2 topic /asr
    ros_node.publish_speech(text)
    
    return jsonify({"status": "success", "message": f"Text received and published: {text}"}), 200

@app.route('/language', methods=['POST'])
def language():
    data = request.json
    if not data or 'text' not in data:
        return jsonify({"error": "Text not provided"}), 400
    
    text = data['text']
    #print(f"Text received from client: {text}")
    
    # Publish to the ROS2 topic /asr
    ros_node.publish_language(text)
    
    return jsonify({"status": "success", "message": f"Text received and published: {text}"}), 200

@app.route('/command', methods=['POST'])
def command():
    data = request.json
    if not data or 'text' not in data:
        return jsonify({"error": "Text not provided"}), 400
    
    text = data['text']
    #print(f"Text received from client: {text}")
    
    # Publish to the ROS2 topic /asr
    ros_node.publish_command(text)
    
    return jsonify({"status": "success", "message": f"Text received and published: {text}"}), 200

# Shutdown the ROS2 node properly when the Flask app stops
@app.route('/shutdown', methods=['POST'])
def shutdown():
    request.environ.get('werkzeug.server.shutdown')()
    rclpy.shutdown()
    return 'Server shutting down...'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5002)
