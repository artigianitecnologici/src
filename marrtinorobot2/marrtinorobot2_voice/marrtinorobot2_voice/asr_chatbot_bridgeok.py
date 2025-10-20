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

import os
import sys
import re
import json
import threading
import requests
import importlib.util

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request
from threading import Thread

# === Caricamento RobotCmdROS da file assoluto ===
ROBOT_CMD_FILE = "lib_robot_cmd_ros.py"

RobotCmdROS = None
_robot_import_error = None
try:
    if not os.path.isfile(ROBOT_CMD_FILE):
        raise FileNotFoundError("File non trovato: {}".format(ROBOT_CMD_FILE))
    spec = importlib.util.spec_from_file_location("robot_cmd_ros", ROBOT_CMD_FILE)
    if spec is None or spec.loader is None:
        raise ImportError("spec_from_file_location ha restituito None per {}".format(ROBOT_CMD_FILE))
    robot_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(robot_module)
    RobotCmdROS = getattr(robot_module, "RobotCmdROS", None)
    if RobotCmdROS is None:
        raise ImportError("RobotCmdROS non presente nel modulo {}".format(ROBOT_CMD_FILE))
except Exception as e:
    _robot_import_error = str(e)

CMD_PREFIX = "#@#"  # prefisso usato da app-ollama in modalita comandi


class ChatbotBridgeNode(Node):
    def __init__(self):
        super().__init__("chatbot_bridge_node")

        # Config server chatbot (Flask app-ollama.py)
        server_ip = "127.0.0.1"
        self.server_url = f"http://{server_ip}:5000/json"  # endpoint JSON

        # ROS publishers/subscribers
        self.publisher = self.create_publisher(String, "/speech/to_speak", 10)
        self.subscription = self.create_subscription(
            String, "/ASR", self.listener_callback, 10
        )

        # Robot controller (RobotCmdROS)
        if RobotCmdROS is None:
            self.get_logger().error("Impossibile importare RobotCmdROS: {}".format(_robot_import_error))
            self.robot = None
        else:
            self.robot = RobotCmdROS()
            self.get_logger().info("RobotCmdROS istanziato.")

        self.get_logger().info("Nodo ChatbotBridge avviato: ascolto /ASR e Flask su 5001")

        # Avvio server Flask (secondo thread)
        self.app = Flask(__name__)
        self.app.add_url_rule("/send", "send", self.flask_receive, methods=["GET"])
        thread = Thread(target=self.app.run, kwargs={"host": "0.0.0.0", "port": 5001})
        thread.daemon = True
        thread.start()

    # --------- ASR inbound ---------
    def listener_callback(self, msg: String):
        text = (msg.data or "").strip()
        self.get_logger().info("ASR: '{}'".format(text))
        if text:
            self._send_text_to_chatbot(text, self.server_url)

    # --------- Flask inbound (bypass diretto TTS) ---------
    def flask_receive(self):
        text = (request.args.get("text") or "").strip()
        if not text:
            return "Parametro 'text' mancante", 400
        self.get_logger().info("Flask inbound: {}".format(text))
        self._publish_tts(text)
        return "OK", 200

    # --------- Pubblica su /speech/to_speak ---------
    def _publish_tts(self, text: str):
        m = String()
        m.data = text
        self.publisher.publish(m)
        self.get_logger().info("Pubblicato su /speech/to_speak: {}".format(text))

    # --------- Call chatbot ---------
    def _send_text_to_chatbot(self, text: str, server_url: str):
        try:
            params = {"query": text}  # /json accetta query come parametro
            resp = requests.get(server_url, params=params, timeout=15)
            if resp.status_code != 200:
                self.get_logger().error("HTTP {}: {}".format(resp.status_code, resp.text))
                return
            data = resp.json()
            answer = (data.get("response") or "").strip()
            self.get_logger().info("Chatbot: {}".format(answer))

            # Se risposta prefissata, interpreta comando
            if answer.startswith(CMD_PREFIX):
                cmd_text = answer[len(CMD_PREFIX):].strip()
                self._process_robot_command(cmd_text)
            else:
                # normale TTS
                self._publish_tts(answer)

        except Exception as e:
            self.get_logger().error("Errore contattando il chatbot: {}".format(e))

    # --------- Parser comandi robot ---------
    def _process_robot_command(self, cmd_text: str):
        # Esempi supportati:
        # - "esegui comando avanti 0.5"
        # - "avanti 0.5"
        # - "indietro 0.4"
        # - "sinistra 90"
        # - "destra 90"
        # - "stop"
        # - "say ciao it"  oppure  say "ciao a tutti" it
        # - "gesture wave"
        # - "emotion happy"
        # - "pan 20" / "tilt -10"
        # - "head front|left|right|up|down"
        # - "getimage"
        # Numeri: metri per avanti/indietro, gradi per sinistra/destra.

        if self.robot is None:
            self.get_logger().error("RobotCmdROS non disponibile: comando ignorato.")
            self._publish_tts("Comando non eseguibile.")
            return

        raw = (cmd_text or "").strip()
        if not raw:
            self._publish_tts("Nessun comando.")
            return

        # rimuove prefissi ridondanti tipo "esegui comando"
        raw = re.sub(r"^\s*(esegui\s+comando|esegui|comando)\s+", "", raw, flags=re.I)

        tokens = raw.split()
        verb = tokens[0].lower() if tokens else ""
        rest = tokens[1:] if len(tokens) > 1 else []

        try:
            # Mapping sinonimi -> verbo canonico
            if verb in ("avanti", "forward", "fwd"):
                dist = self._parse_float_arg(rest, default=0.3)
                self.robot.forward(dist)
                self._publish_tts("Vado avanti {:.2f} metri.".format(dist))
                return

            if verb in ("indietro", "backward", "back", "retro"):
                dist = self._parse_float_arg(rest, default=0.3)
                self.robot.backward(dist)
                self._publish_tts("Vado indietro {:.2f} metri.".format(dist))
                return

            if verb in ("sinistra", "left", "sx"):
                ang = self._parse_float_arg(rest, default=30.0)
                self.robot.left(ang)
                self._publish_tts("Ruoto a sinistra {} gradi.".format(int(ang)))
                return

            if verb in ("destra", "right", "dx"):
                ang = self._parse_float_arg(rest, default=30.0)
                self.robot.right(ang)
                self._publish_tts("Ruoto a destra {} gradi.".format(int(ang)))
                return

            if verb in ("stop", "halt", "ferma", "fermo"):
                self.robot.stop()
                self._publish_tts("Fermo.")
                return

            if verb == "say":
                txt, lang = self._parse_say_args(rest)
                self.robot.say(txt, lang)
                return

            if verb == "gesture":
                msg = " ".join(rest) if rest else "wave"
                self.robot.gesture(msg)
                self._publish_tts("Eseguo gesto {}".format(msg))
                return

            if verb == "sorridi":
                msg = " ".join(rest) if rest else "happy"
                self.robot.emotion(msg)
                self._publish_tts("Imposto emozione {}".format(msg))
                return

            if verb == "pan":
                val = self._parse_float_arg(rest, default=0.0)
                self.robot.pan(val)
                self._publish_tts("Pan a {} gradi.".format(int(val)))
                return

            if verb == "tilt":
                val = self._parse_float_arg(rest, default=0.0)
                self.robot.tilt(val)
                self._publish_tts("Tilt a {} gradi.".format(int(val)))
                return

            if verb == "head":
                pos = (rest[0].lower() if rest else "front")
                self.robot.head_position(pos)
                self._publish_tts("Head {}".format(pos))
                return

            if verb in ("right_arm", "braccio_destro"):
                val = self._parse_float_arg(rest, default=0.0)
                self.robot.right_arm(val)
                self._publish_tts("Braccio destro a {}.".format(val))
                return

            if verb in ("left_arm", "braccio_sinistro"):
                val = self._parse_float_arg(rest, default=0.0)
                self.robot.left_arm(val)
                self._publish_tts("Braccio sinistro a {}.".format(val))
                return

            if verb == "getimage":
                self.robot.getImage()
                self._publish_tts("Acquisisco immagine.")
                return

            # sconosciuto
            self.get_logger().warning("Comando sconosciuto: '{}'".format(raw))
            self._publish_tts("Comando sconosciuto.")
        except Exception as e:
            self.get_logger().error("Errore eseguendo comando '{}': {}".format(raw, e))
            self._publish_tts("Errore eseguendo il comando.")

    # --------- Helpers parsing ---------
    def _parse_float_arg(self, tokens, default=0.0):
        if not tokens:
            return float(default)
        for t in tokens:
            m = re.match(r"^-?\d+(?:[.,]\d+)?$", t)
            if m:
                return float(t.replace(",", "."))
        return float(default)

    def _parse_say_args(self, tokens):
        # Restituisce (text, lang).
        # ['ciao', 'it'] -> ("ciao", "it")
        # ['"ciao', 'mondo"', 'it'] -> ("ciao mondo", "it")
        # ['"ciao', 'mondo"'] -> ("ciao mondo", 'it')
        if not tokens:
            return ("", "it")
        joined = " ".join(tokens)
        m = re.search(r'"([^"]+)"\s*(\S+)?', joined)
        if m:
            txt = m.group(1)
            lang = m.group(2) if m.group(2) else "it"
            return (txt, lang)
        if len(tokens) >= 2 and re.fullmatch(r"[a-zA-Z]{2}", tokens[-1]):
            lang = tokens[-1].lower()
            txt = " ".join(tokens[:-1])
            return (txt, lang)
        return (" ".join(tokens), "it")


def main(args=None):
    rclpy.init(args=args)
    node = ChatbotBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
