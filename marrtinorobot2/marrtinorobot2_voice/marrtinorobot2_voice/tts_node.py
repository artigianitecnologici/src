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
# file : tts_node.py
#! /usr/bin/python3

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import numpy as np
import soundfile as sf
import os
import socket
import subprocess
import json
import sounddevice as sd

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
         
         
        
        # Dynamically find the config.json file path
        # package_share_directory = get_package_share_directory('marrtinorobot2_voice')
        # config_path = os.path.join(package_share_directory, 'config', 'config.json')

        # Define the absolute path to the config.json file
        config_path = '/home/ubuntu/src/marrtinorobot2/marrtinorobot2_voice/config/config.json'
        
        # Check if the file exists before opening it
        if not os.path.exists(config_path):
            self.get_logger().error(f"Configuration file not found: {config_path}")
            return
        
        self.get_logger().info(f"Config file path: {config_path}")
        
        # Check if the file exists before opening it
        if not os.path.exists(config_path):
            self.get_logger().error(f"Configuration file not found: {config_path}")
            return
        
        # Load the configuration file
        with open(config_path, "r") as f:
            config = json.load(f)
        
        self.get_logger().info('Configuration loaded successfully')

        self.output_samplerate = config["output_samplerate"]
        self.output_device_index = config["output_device_index"]
        self.work_offline = config["work_offline"]
        self.msg_start = config["msg_start"]
        self.language = config["language"]
 

        
        msgstart = f"ros2 topic pub -1 /speech/to_speak std_msgs/msg/String '{{data: \"{self.msg_start}\"}}'"
        os.system(msgstart)



    def play_beep(self,frequency=1000, duration=0.5, volume=0.1):
        t = np.linspace(0, duration, int( self.output_samplerate * duration), False)
        wave = (volume * np.sin(2 * np.pi * frequency * t)).astype(np.float32)
        try:
            with sd.OutputStream(samplerate= self.output_samplerate, channels=1, dtype='float32', device=self.output_device_index) as stream:
                stream.write(wave)
            print("Suono beep riprodotto.")
        except Exception as e:
            print(f"Errore durante la riproduzione del beep: {e}")

    def language_callback(self, msg):
        # Update the language based on received message
        self.language = msg.data
        self.get_logger().info(f'Language set to: {self.language}')

    def tts_callback(self, msg):
        text = msg.data.strip()  # Rimuove spazi iniziali/finali

        if not text:
            self.get_logger().warn('Received empty text, skipping TTS.')
            return  # Evita il crash!

        self.get_logger().info(f'Received text: "{text}"')
        self.finished_speaking = False
        self.loop_count_down = 0
        self.publisher_.publish(String(data='ON'))

        if self.is_connected():
            try:
                tts = gTTS(text, lang=self.language)
                filename = "/tmp/output.mp3"
                tts.save(filename)
                os.system('mpg321 ' + filename)
                self.publisher_.publish(String(data='TTS done'))
            except Exception as e:
                self.get_logger().error(f"gTTS failed: {e}")
        else:
            if self.language == 'it':
                self.language = 'it-IT'
            elif self.language == 'en':
                self.language = 'en-US'

            filename = "/tmp/robot_speach.wav"
            cmd = ['pico2wave', '--wave=' + filename, '--lang=' + self.language, text]
            subprocess.call(cmd)
            cmd = ['play', filename, '--norm', '-q']
            subprocess.call(cmd)


    # def tts_callback(self, msg):
    #     text = msg.data
    #     self.get_logger().info(f'Received text: "{text}"')
    #     self.finished_speaking = False
    #     self.loop_count_down = 0
    #     self.publisher_.publish(String(data='ON'))
    #     # Check internet connectivity
    #     if self.is_connected():
    #         # Convert text to speech using the set language
    #         tts = gTTS(text, lang=self.language)
    #         filename = "/tmp/output.mp3"
    #         tts.save(filename)
    #         os.system('mpg321 ' + filename)
    #         # Publish the fact that the TTS is done
    #         self.publisher_.publish(String(data='TTS done'))
    #     else:
    #         if self.language == 'it':
    #             self.language = 'it-IT'
    #         if self.language == 'en':
    #             self.language = 'en-US'
                
    #         filename = "/tmp/robot_speach.wav"
    #         cmd = ['pico2wave', '--wave=' + filename, '--lang=' + self.language, f'"{text}"']

    #         subprocess.call(cmd)
    #         cmd = ['play', filename, '--norm', '-q']

    #         subprocess.call(cmd)
    #         self.finished_speaking = True
    #         self.loop_count_down = int(self.LOOP_FREQUENCY * 2)

    def speaking_finished(self):
        if self.finished_speaking:
            self.loop_count_down -= 1
            if self.loop_count_down <= 0:
                self.get_logger().info('Speaking finished')
                self.finished_speaking = False
                self.publisher_.publish(String(data='OFF'))

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
