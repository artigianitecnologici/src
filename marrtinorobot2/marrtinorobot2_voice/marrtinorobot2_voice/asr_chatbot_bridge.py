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
# File  : asr_chatbot_bridge.py
#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
from flask import Flask, request
from threading import Thread

class ChatbotBridgeNode(Node):
    def __init__(self):
        super().__init__('chatbot_bridge_node')
        # IP server ollama
        server_ip = '10.3.1.98'  
        self.server_url = f"http://{server_ip}:8060/json"
        #
        self.publisher = self.create_publisher(String, '/speech/to_speak', 10)
        self.subscription = self.create_subscription(
            String,
            '/ASR',
            self.listener_callback,
            10
        )
        self.get_logger().info("🤖 Nodo ChatbotBridge in ascolto su /ASR e Flask su porta 5001")
         
        # Avvio del server Flask in thread separato
        self.app = Flask(__name__)
        self.app.add_url_rule('/send', 'send', self.flask_receive, methods=['GET'])
        thread = Thread(target=self.app.run, kwargs={'host': '0.0.0.0', 'port': 5001})
        thread.daemon = True
        thread.start()

    def listener_callback(self, msg):
        text = msg.data.strip()
        self.get_logger().info(f"🗣️ Ricevuto testo ASR: '{text}'")
        if text:
            self.send_text_to_chatbot(text,self.server_url)

    def flask_receive(self):
        text = request.args.get('text')
        if not text:
            return "Parametro 'text' mancante", 400
        self.get_logger().info(f"🌐 Ricevuto da Flask: {text}")
        # Pubblica direttamente su /speech/to_speak senza passare dal chatbot
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f"📤 Pubblicato su /speech/to_speak da Flask: {text}")
        return "OK", 200

    def send_text_to_chatbot(self, text, server_url="http://localhost:5000/get"):
        try:
            params = {"query": text}
            response = requests.get(server_url, params=params)
            if response.status_code == 200:
                data = response.json()
                answer = data.get('response', '')
                self.get_logger().info(f"🤖 Risposta del chatbot: {answer}")
                self.send_text_to_robot_command(answer)
            else:
                self.get_logger().error(f"Errore HTTP {response.status_code}: {response.text}")
        except Exception as e:
            self.get_logger().error(f"Errore durante la comunicazione con il chatbot: {e}")

    def send_text_to_robot_command(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f"📤 Pubblicato su /speech/to_speak: {text}")


def main(args=None):
    rclpy.init(args=args)
    node = ChatbotBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
