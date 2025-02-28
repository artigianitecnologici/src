import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import socket
from pydub import AudioSegment
from subprocess import Popen, PIPE
import requests.packages.urllib3
requests.packages.urllib3.disable_warnings()

tmpfile = "/tmp/cacheita.mp3"
wavfile = "/tmp/cacheita.wav"
offfile = "/tmp/cache.wav"


class TTSNode(Node):

    def __init__(self):
        super().__init__('tts_node')
        self.get_logger().info('Start tts_node v.1.01')
        self.publisher_ = self.create_publisher(String, '/social/speech/status', 10)
        self.subscription = self.create_subscription(
            String,
            '/speech/to_speak',
            self.tts_callback,
            10)
        self.lang_subscription = self.create_subscription(
            String,
            '/speech/language',
            self.language_callback,
            10)

        # Default language
        self.language = 'it'  # Italian by default
        # For managing speaking state
        self.finished_speaking = False
        self.loop_count_down = 0
        self.rate = self.create_rate(10)  # Set loop frequency to 10Hz
        self.connected = True
        self.msgoffline = False

    def language_callback(self, msg):
        # Convert the language setting to Unicode
        self.language = msg.data
        self.get_logger().info(f'Language updated to: "{self.language}"')

    def tts_callback(self, msg):
        # Convert the incoming text to Unicode
        text = msg.data  # Ensures proper decoding of special characters
        self.get_logger().info(f'Received text: "{text}"')
        self.get_logger().info(f'Using language: "{self.language}"')
        self.finished_speaking = False
        self.loop_count_down = 0

        # Check internet connectivity
        if self.is_connected():
            try:
                # Convert text to speech using Google TTS
                self.get_logger().info(f"gtts={text}")
                tts = gTTS(text, lang=self.language)
                tts.save(tmpfile)
                sound = AudioSegment.from_mp3(tmpfile)
                sound.export(wavfile, format="wav")
                if self.language in ['it-IT', 'it']:
                    p = Popen("play " + wavfile + " -q pitch 300 rate 48000", stdout=PIPE, shell=True)
                else:
                    p = Popen("play " + wavfile + " -q ", stdout=PIPE, shell=True)
                p.wait()

                # Publish the fact that the TTS is done
                self.publisher_.publish(String(data='TTS done'))
            except Exception as e:
                self.get_logger().error(f"Error in TTS conversion: {str(e)}")
        else:
            # Fallback to pico2wave if there's no internet connection
            if self.language == 'it':
                self.language = 'it-IT'
            if self.language == 'en':
                self.language = 'en-US'

            if text == 'attivazione':
                self.connected = True
                if self.is_connected():
                    p = Popen("pico2wave -l " + self.language + " -w " + wavfile + " ' Sono connesso a internet e ricordati' ", stdout=PIPE, shell=True)
                    p.wait()
                    p = Popen("play " + wavfile + " -q --norm", stdout=PIPE, shell=True)
                    p.wait()
                    self.msgoffline = False

            if self.msgoffline:
                p = Popen("pico2wave -l " + self.language + " -w " + wavfile + " 'sono disconnesso dalla rete internet  , per verificare la connessione invia la parola attivazione '  ", stdout=PIPE, shell=True)
                p.wait()
                p = Popen("play " + wavfile + " -q --norm", stdout=PIPE, shell=True)
                p.wait()
                self.msgoffline = False

            # Fallback to pico2wave
            self.get_logger().info(f"pico={text}")
            p = Popen("pico2wave -l " + self.language + " -w " + wavfile + " '" + text + "' ", stdout=PIPE, shell=True)
            p.wait()
            p = Popen("play " + wavfile + " -q --norm", stdout=PIPE, shell=True)
            p.wait()
            self.finished_speaking = True
            self.loop_count_down = int(10 * 2)  # 2 seconds delay at 10Hz rate

    def speaking_finished(self):
        if self.finished_speaking:
            self.loop_count_down -= 1
            if self.loop_count_down <= 0:
                self.get_logger().info('Speaking finished')
                self.finished_speaking = False
                self.publisher_.publish(String(data='TTS done'))

    def is_connected(self):
        if self.connected:
            try:
                # Create a socket with a very short timeout (e.g., 0.1 seconds)
                sock = socket.create_connection(("www.google.it", 80), timeout=0.1)
                sock.close()  # Close the socket after connecting
                return True
            except socket.error as e:
                self.get_logger().info(f"No internet connection: {str(e)}")
                self.connected = False
                self.msgoffline = True
                return False
        else:
            return False

    def spin(self):
        while rclpy.ok():
            self.speaking_finished()
            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    node.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
