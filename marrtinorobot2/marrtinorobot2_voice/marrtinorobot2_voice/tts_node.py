#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import socket
import subprocess

class TTSNode(Node):

    def __init__(self):
        super().__init__('tts_node')
        self.LOOP_FREQUENCY = 10 
        self.get_logger().info('Start tts_node v.1.02')
        self.publisher_ = self.create_publisher(String, '/speech/status', 10)
        
        # Subscription to receive text to speak
        self.subscription_text = self.create_subscription(
            String,
            '/speech/to_speak',
            self.tts_callback,
            10)
        
        # Subscription to receive language parameter
        self.subscription_lang = self.create_subscription(
            String,
            '/speech/language',
            self.language_callback,
            10)
        
        # Default language
        self.language = 'it'
        self.connected = True
        self.msgoffline = False

        msgstart = "ros2 topic pub  -1 /speech/to_speak std_msgs/msg/String '{data: \"tutti i sistemi sono operativi\"}'"
        os.system(msgstart)

    def language_callback(self, msg):
        # Update the language based on received message
        self.language = msg.data
        self.get_logger().info(f'Language set to: {self.language}')

    def tts_callback(self, msg):
        text = msg.data
        self.get_logger().info(f'Received text: "{text}"')
        self.finished_speaking = False
        self.loop_count_down = 0

        # Check internet connectivity
        if self.is_connected():
            # Convert text to speech using the set language
            tts = gTTS(text, lang=self.language)
            filename = "/tmp/output.mp3"
            tts.save(filename)
            os.system('mpg321 ' + filename)
            # Publish the fact that the TTS is done
            self.publisher_.publish(String(data='TTS done'))
        else:
            if self.language == 'it':
                self.language = 'it-IT'
            if self.language == 'en':
                self.language = 'en-US'
                
            filename = "/tmp/robot_speach.wav"
            cmd = ['pico2wave', '--wave=' + filename, '--lang=' + self.language, text]
            subprocess.call(cmd)
            cmd = ['play', filename, '--norm', '-q']
            subprocess.call(cmd)
            self.finished_speaking = True
            self.loop_count_down = int(self.LOOP_FREQUENCY * 2)

    def speaking_finished(self):
        if self.finished_speaking:
            self.loop_count_down -= 1
            if self.loop_count_down <= 0:
                self.get_logger().info('Speaking finished')
                self.finished_speaking = False
                self.publisher_.publish(String(data='TTS done'))

    def is_connected(self):
        try:
            # Try to connect to a well-known website
            socket.create_connection(("www.google.com", 80))
            return True
        except OSError:
            return False
        
def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    rclpy.spin(tts_node)
    tts_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# ros2 topic pub /speech/language std_msgs/msg/String '{data: "en"}'
