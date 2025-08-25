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
# File  : asr_tts_node_offline.py
#!/usr/bin/env python3

import os
import json
import queue
import socket
import subprocess
import urllib.request
import zipfile
import numpy as np
import sounddevice as sd
from vosk import Model, KaldiRecognizer
from gtts import gTTS

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def download_vosk_model(model_url, model_path):
    print(f"ðŸ“… Scarico modello Vosk da {model_url}")
    zip_path = "vosk_model.zip"
    try:
        urllib.request.urlretrieve(model_url, zip_path)
    except Exception as e:
        print(f"âŒ Errore nel download del modello: {e}")
        return

    try:
        with zipfile.ZipFile(zip_path, 'r') as zip_ref:
            zip_ref.extractall(os.path.dirname(model_path))
        os.remove(zip_path)
        print("âœ… Modello scaricato ed estratto.")
    except Exception as e:
        print(f"âŒ Errore nell'estrazione del modello: {e}")

class ASRTTSNode(Node):
    def __init__(self):
        super().__init__('asr_tts_node')
        self.get_logger().info("ðŸ¤– Nodo ASR+TTS avviato")

        config_path = '/home/ubuntu/src/marrtinorobot2/marrtinorobot2_voice/config/config.json'
        if not os.path.exists(config_path):
            self.get_logger().error(f"Config mancante: {config_path}")
            return

        with open(config_path, 'r') as f:
            config = json.load(f)


        self.output_samplerate = config["output_samplerate"]
        self.output_device_index = config["output_device_index"]
        self.input_samplerate = config["input_samplerate"]
        self.language = config["language"]
        self.work_offline = config["work_offline"]
        self.msg_start = config["msg_start"]
        self.wake_word = config.get("wake_word", "martino").lower()

        # Auto-selezione microfono ReSpeaker
        device_list = sd.query_devices()
        self.input_device_index = None
        for idx, device in enumerate(device_list):
            if 'ReSpeaker' in device['name']:
                self.input_device_index = idx
                break

        if self.input_device_index is None:
            self.get_logger().error("âŒ Microfono ReSpeaker non trovato. Controlla che sia collegato.")
            return

        self.model_vosk_path = "/home/ubuntu/src/marrtinorobot2/marrtinorobot2_voice/models"
        self.vosk_model_url = "https://alphacephei.com/vosk/models/vosk-model-it-0.22.zip"

        if subprocess.call(['which', 'pico2wave'], stdout=subprocess.DEVNULL) != 0:
            self.get_logger().error("âŒ pico2wave non Ã¨ installato. Installa con: sudo apt install libttspico-utils")

        expected_file = os.path.join(self.model_vosk_path + "/vosk-model-it-0.22/", "am", "final.mdl")
        if not os.path.exists(expected_file):
            download_vosk_model(self.vosk_model_url, self.model_vosk_path)

        self.model_vosk = Model(self.model_vosk_path +"/vosk-model-it-0.22")

        self.publisher_asr = self.create_publisher(String, '/ASR', 10)

        self.subscription_lang = self.create_subscription(
            String,
            '/speech/language',
            self.language_callback,
            10)

        self.subscription_text = self.create_subscription(
            String,
            '/speech/to_speak',
            self.tts_callback,
            10)
        self.TOPIC_emotion = "social/emotion"
        self.TOPIC_gesture = "social/gesture"
        # Publisher definitions
        self.emotion_pub = self.create_publisher(String, self.TOPIC_emotion, 10)
        self.gesture_pub = self.create_publisher(String, self.TOPIC_gesture, 10)


        self.listening = True
        self.tts_busy = False
        self.queue = queue.Queue()
        self.recognizer = KaldiRecognizer(self.model_vosk, self.input_samplerate)

        sd.default.device = (self.input_device_index, self.output_device_index)

        try:
            self.stream = sd.RawInputStream(
                samplerate=self.input_samplerate,
                dtype='int16',
                channels=1,
                callback=self.audio_callback,
                device=self.input_device_index
            )
            self.stream.start()
            self.get_logger().info("ðŸŽ¤ Microfono avviato")
        except Exception as e:
            self.get_logger().error(f"âŒ Errore nell'avvio del microfono: {e}")

        self.create_timer(0.1, self.listen_loop)
        self.get_logger().info("â± Timer ascolto continuo avviato.")
        # robot.emotion("startblinking")
        # robot.emotion("speak")
        # r  obot.emotion("normal")
        self.emotion("startblinking")
        self.emotion("speak")
        self.speak(self.msg_start)
        self.emotion("normal")


    def gesture(self, msg):
        self.get_logger().info(f'gesture: {msg}')
        message = String()
        message.data = msg
        self.gesture_pub.publish(message)

    def emotion(self, msg):
        self.get_logger().info(f'social/emotion: {msg}')
        message = String()
        message.data = msg
        self.emotion_pub.publish(message)

    def language_callback(self, msg):
        self.language = msg.data
        self.get_logger().info(f"ðŸŒ Lingua impostata: {self.language}")


    def tts_callback(self, msg):
        text = msg.data
        self.get_logger().info(f'Received text: "{text}"')

        self.tts_busy = True
        self.listening = False
        try:
            self.stream.stop()
            with self.queue.mutex:
                self.queue.queue.clear()
            self.get_logger().info("ðŸ”‡ Microfono disattivato e coda svuotata.")
        except Exception as e:
            self.get_logger().warning(f"Errore nel fermare il microfono: {e}")

        try:
            if (self.work_offline == True):
                lang_code = 'it-IT' if self.language == 'it' else 'en-US'
                filename = "/tmp/robot_speech.wav"
                subprocess.call(['pico2wave', '--wave=' + filename, '--lang=' + lang_code, text])

                self.emotion("speak")
                subprocess.call(['play', filename, '--norm', '-q'])
            else:
                tts = gTTS(text, lang=self.language)
                filename = "/tmp/output.mp3"
                tts.save(filename)

                self.emotion("speak")
                os.system(f"mpg321 {filename}")
        except Exception as e:
            self.get_logger().error(f"Errore TTS: {e}")

        try:
            self.stream.start()
            self.get_logger().info("ðŸŽ¤ Microfono riattivato.")
        except Exception as e:
            self.get_logger().warning(f"Errore nel riavvio del microfono: {e}")

        self.emotion("normal")
        self.tts_busy = False
        self.listening = True

    def speak(self, text):
        if not text:
            return

        self.tts_busy = True
        self.listening = False

        try:
            self.stream.stop()
            with self.queue.mutex:
                self.queue.queue.clear()
            self.get_logger().info("ðŸ”‡ Microfono disattivato e coda svuotata.")
        except Exception as e:
            self.get_logger().warning(f"Errore nel fermare il microfono: {e}")

        # try:

        # except Exception as e:
        #     self.get_logger().error(f"Errore TTS: {e}")
        try:
            if (self.work_offline == True):
                lang_code = 'it-IT' if self.language == 'it' else 'en-US'
                filename = "/tmp/robot_speech.wav"
                subprocess.call(['pico2wave', '--wave=' + filename, '--lang=' + lang_code, text])
                subprocess.call(['play', filename, '--norm', '-q'])
            else:
                tts = gTTS(text, lang=self.language)
                filename = "/tmp/output.mp3"
                tts.save(filename)
                os.system(f"mpg321 {filename}")

        except Exception as e:
            self.get_logger().error(f"Errore TTS: {e}")

        try:
            self.stream.start()
            self.get_logger().info("ðŸŽ¤ Microfono riattivato.")
        except Exception as e:
            self.get_logger().warning(f"Errore nel riavvio del microfono: {e}")

        self.tts_busy = False
        self.listening = True

    def listen_loop(self):
        if not self.listening or self.tts_busy:
            return

        try:
            while not self.queue.empty():
                data = self.queue.get_nowait()
                if self.recognizer.AcceptWaveform(data):
                    result = json.loads(self.recognizer.Result())
                    text = result.get("text", "").strip().lower()
                    if text:
                        self.get_logger().info(f"ðŸŽ¤ Hai detto: {text}")
                        if text.startswith(self.wake_word):
                            filtered_text = text[len(self.wake_word):].strip()
                            self.get_logger().info(f"ðŸ”‘ Parola chiave rilevata. Testo: {filtered_text}")
                            if filtered_text:  # Pubblica solo se c'Ã¨ altro dopo la parola chiave
                                self.publish_asr(filtered_text)
        except Exception as e:
            self.get_logger().error(f"Errore ascolto: {e}")



    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warning(str(status))
        self.queue.put(bytes(indata))

    def publish_asr(self, text):
        msg = String()
        msg.data = text
        self.publisher_asr.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ASRTTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
