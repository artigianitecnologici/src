#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2025 robotics-3d.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Ferrarini Fabio
# Email : ferrarini09@gmail.com
# File  : asr_chatbot_bridge.py

import os
import re
import json
import threading
import requests
import importlib.util
from queue import Queue, Empty
from urllib.parse import urlparse
from flask import Flask, request
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String


# =====================================================
# === Caricamento dinamico RobotCmdROS =================
# =====================================================
ROBOT_CMD_FILE = "lib_robot_cmd_ros.py"

RobotCmdROS = None
_robot_import_error = None
try:
    if not os.path.isfile(ROBOT_CMD_FILE):
        raise FileNotFoundError(f"File non trovato: {ROBOT_CMD_FILE}")
    spec = importlib.util.spec_from_file_location("robot_cmd_ros", ROBOT_CMD_FILE)
    robot_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(robot_module)
    RobotCmdROS = getattr(robot_module, "RobotCmdROS", None)
    if RobotCmdROS is None:
        raise ImportError(f"RobotCmdROS non presente nel modulo {ROBOT_CMD_FILE}")
except Exception as e:
    _robot_import_error = str(e)


# =====================================================
# === Config Chatbot / Endpoint =======================
# =====================================================
CMD_PREFIX = "#@#"
APP_OLLAMA_URL = os.environ.get("APP_OLLAMA_URL", "http://127.0.0.1:5000/json")
CONNECT_TIMEOUT = float(os.environ.get("CHATBOT_CONNECT_TIMEOUT", "3"))
READ_TIMEOUT = float(os.environ.get("CHATBOT_READ_TIMEOUT", "45"))
MAX_RETRIES = int(os.environ.get("CHATBOT_MAX_RETRIES", "2"))
ENDPOINTS = os.environ.get("CHATBOT_ENDPOINTS", "/json,/job,/chat,/ask").split(",")


# =====================================================
# === Nodo principale ROS2 ============================
# =====================================================
class ChatbotBridgeNode(Node):
    def __init__(self):
        super().__init__("chatbot_bridge_node")

        # Stato di vita
        self._alive = True
        self._stop_event = threading.Event()

        # ROS setup
        self.cbgroup = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(String, "/speech/to_speak", 10)
        self.subscription = self.create_subscription(
            String, "/ASR", self.listener_callback, 10, callback_group=self.cbgroup
        )

        # Code sicure per publish da thread esterni
        self._tts_queue = Queue()
        self._robot_queue = Queue()

        # Timer che svuota le code nel contesto ROS
        self.create_timer(0.05, self._drain_queues, callback_group=self.cbgroup)

        # Chatbot setup
        self.http = requests.Session()
        self.server_url = APP_OLLAMA_URL

        # Robot
        if RobotCmdROS is None:
            self.get_logger().error(f"Impossibile importare RobotCmdROS: {_robot_import_error}")
            self.robot = None
        else:
            self.robot = RobotCmdROS()
            self.get_logger().info("RobotCmdROS istanziato.")

        # Flask app per ricezione esterna
        self.app = Flask(__name__)
        self.app.add_url_rule("/send", "send", self.flask_receive, methods=["GET"])
        self._flask_thread = Thread(
            target=self.app.run, kwargs={"host": "0.0.0.0", "port": 5001}, daemon=True
        )
        self._flask_thread.start()

        self.get_logger().info(f"ChatbotBridgeNode avviato su {self.server_url}")

    # =================================================
    # === Gestione inbound ASR ========================
    # =================================================
    def listener_callback(self, msg: String):
        text = (msg.data or "").strip()
        self.get_logger().info(f"ASR: '{text}'")
        if text:
            t = threading.Thread(target=self._send_text_to_chatbot, args=(text,), daemon=True)
            t.start()

    # =================================================
    # === Flask inbound (bypass diretto TTS) ==========
    # =================================================
    def flask_receive(self):
        text = (request.args.get("text") or "").strip()
        if not text:
            return "Parametro 'text' mancante", 400
        self.get_logger().info(f"Flask inbound: {text}")
        self._tts_queue.put(text)
        return "OK", 200

    # =================================================
    # === Gestione code TTS / Robot ===================
    # =================================================
    def _drain_queues(self):
        if not self._alive or not rclpy.ok():
            return
        # TTS queue
        try:
            while True:
                txt = self._tts_queue.get_nowait()
                msg = String()
                msg.data = txt
                try:
                    self.publisher.publish(msg)
                except Exception as e:
                    self.get_logger().warning(f"Errore pubblicando TTS: {e}")
        except Empty:
            pass
        # Robot queue
        try:
            while True:
                fn, args, kwargs = self._robot_queue.get_nowait()
                try:
                    fn(*args, **kwargs)
                except Exception as e:
                    self.get_logger().error(f"Azione robot fallita: {e}")
        except Empty:
            pass

    # =================================================
    # === Richiesta al chatbot ========================
    # =================================================
    def _candidate_urls(self, base: str):
        p = urlparse(base)
        if p.path and p.path != "/":
            return [base]
        base = base.rstrip("/")
        return [f"{base}{ep if ep.startswith('/') else '/'+ep}" for ep in ENDPOINTS]

    def _send_text_to_chatbot(self, text: str):
        payload_keys = ["query", "q", "prompt", "text"]
        timeouts = (CONNECT_TIMEOUT, READ_TIMEOUT)

        def try_url(url):
            # 1) GET
            for k in payload_keys:
                try:
                    r = self.http.get(url, params={k: text}, timeout=timeouts)
                    if r.status_code == 200:
                        return r
                except requests.exceptions.RequestException:
                    pass
            # 2) POST form
            try:
                r = self.http.post(url, data={"query": text}, timeout=timeouts)
                if r.status_code == 200:
                    return r
            except requests.exceptions.RequestException:
                pass
            # 3) POST json
            try:
                r = self.http.post(url, json={"query": text}, timeout=timeouts)
                if r.status_code == 200:
                    return r
            except requests.exceptions.RequestException:
                pass
            return None

        # Tentativi multipli
        resp = None
        for attempt in range(1 + MAX_RETRIES):
            for candidate in self._candidate_urls(self.server_url):
                self.get_logger().info(f"Tentativo {attempt+1}: provo {candidate}")
                resp = try_url(candidate)
                if resp is not None:
                    break
            if resp is not None:
                break

        if resp is None:
            msg = "Non riesco a contattare il chatbot."
            self.get_logger().error(msg)
            self._tts_queue.put(msg)
            return

        # Parsing risposta
        try:
            data = resp.json()
        except Exception:
            preview = (resp.text or "")[:200]
            self.get_logger().error(f"Risposta non JSON. Anteprima: {preview}")
            self._tts_queue.put("Risposta non valida dal chatbot.")
            return

        answer = (data.get("response") or "").strip()
        self.get_logger().info(f"Chatbot: {answer}")

        if answer.startswith(CMD_PREFIX):
            cmd_text = answer[len(CMD_PREFIX):].strip()
            self._process_robot_command(cmd_text)
        else:
            self._tts_queue.put(answer)

    # =================================================
    # === Comandi robot ===============================
    # =================================================
    def _enqueue_robot(self, fn, *args, **kwargs):
        self._robot_queue.put((fn, args, kwargs))

    def _process_robot_command(self, cmd_text: str):
        if self.robot is None:
            self._tts_queue.put("Comando non eseguibile (robot non disponibile).")
            return

        raw = (cmd_text or "").strip()
        if not raw:
            self._tts_queue.put("Nessun comando.")
            return
        raw = re.sub(r"^\s*(esegui\s+comando|esegui|comando)\s+", "", raw, flags=re.I)
        tokens = raw.split()
        verb = tokens[0].lower() if tokens else ""
        rest = tokens[1:] if len(tokens) > 1 else []

        try:
            if verb in ("avanti", "forward", "fwd"):
                dist = self._parse_float_arg(rest, 0.3)
                self._enqueue_robot(self.robot.forward, dist)
                self._tts_queue.put(f"Vado avanti {dist:.2f} metri.")
                return
            if verb in ("indietro", "back", "retro"):
                dist = self._parse_float_arg(rest, 0.3)
                self._enqueue_robot(self.robot.backward, dist)
                self._tts_queue.put(f"Vado indietro {dist:.2f} metri.")
                return
            if verb in ("sinistra", "left", "sx"):
                ang = self._parse_float_arg(rest, 30)
                self._enqueue_robot(self.robot.left, ang)
                self._tts_queue.put(f"Ruoto a sinistra {int(ang)} gradi.")
                return
            if verb in ("destra", "right", "dx"):
                ang = self._parse_float_arg(rest, 30)
                self._enqueue_robot(self.robot.right, ang)
                self._tts_queue.put(f"Ruoto a destra {int(ang)} gradi.")
                return
            if verb in ("stop", "halt", "ferma", "fermo"):
                self._enqueue_robot(self.robot.stop)
                self._tts_queue.put("Fermo.")
                return
            if verb == "gesture":
                msg = " ".join(rest) if rest else "wave"
                self._enqueue_robot(self.robot.gesture, msg)
                self._tts_queue.put(f"Eseguo gesto {msg}")
                return
            if verb in ("emotion", "sorridi"):
                msg = " ".join(rest) if rest else "happy"
                self._enqueue_robot(self.robot.emotion, msg)
                self._tts_queue.put(f"Imposto emozione {msg}")
                return
            if verb == "pan":
                val = self._parse_float_arg(rest, 0.0)
                self._enqueue_robot(self.robot.pan, val)
                self._tts_queue.put(f"Pan a {int(val)} gradi.")
                return
            if verb == "tilt":
                val = self._parse_float_arg(rest, 0.0)
                self._enqueue_robot(self.robot.tilt, val)
                self._tts_queue.put(f"Tilt a {int(val)} gradi.")
                return
            if verb == "getimage":
                self._enqueue_robot(self.robot.getImage)
                self._tts_queue.put("Acquisisco immagine.")
                return
            self._tts_queue.put("Comando sconosciuto.")
        except Exception as e:
            self.get_logger().error(f"Errore eseguendo comando '{raw}': {e}")
            self._tts_queue.put("Errore eseguendo il comando.")

    # =================================================
    # === Helper parsing ==============================
    # =================================================
    def _parse_float_arg(self, tokens, default=0.0):
        for t in tokens:
            if re.match(r"^-?\d+(?:[.,]\d+)?$", t):
                return float(t.replace(",", "."))
        return float(default)

    # =================================================
    # === Shutdown ordinato ===========================
    # =================================================
    def shutdown(self):
        self._alive = False
        self._stop_event.set()
        self.get_logger().info("Chiusura ChatbotBridgeNode richiesta.")


# =====================================================
# === Main ============================================
# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = ChatbotBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        for _ in range(10):
            node._drain_queues()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
