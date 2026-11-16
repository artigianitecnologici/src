#!/usr/bin/env python3
import os
import json
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image

from insightface.app import FaceAnalysis


def _normalize(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=np.float32)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def _rosimg_to_bgr8(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image to BGR uint8 array without cv_bridge.
    Supports 'bgr8' and 'rgb8'. Raise ValueError otherwise.
    """
    h = msg.height
    w = msg.width
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if enc == 'bgr8':
        return buf.reshape((h, w, 3))
    elif enc == 'rgb8':
        rgb = buf.reshape((h, w, 3))
        return rgb[:, :, ::-1]
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")


class FaceRecognizerNode(Node):
    def __init__(self):
        super().__init__('face_recognizer')

        # ---------- Parameters ----------
        self.declare_parameter('db_dir', 'data')  # folder with embeddings.npy + names.json
        self.declare_parameter('threshold', 0.45)
        self.declare_parameter('providers', ['CPUExecutionProvider'])  # GPU: ['CUDAExecutionProvider']
        self.declare_parameter('det_size_w', 640)
        self.declare_parameter('det_size_h', 640)
        self.declare_parameter('topic_image', '/camera/image_raw')
        self.declare_parameter('topic_output', '/recognized_face')
        self.declare_parameter('topic_command', '/face_recognizer/command')
        self.declare_parameter('topic_status', '/face_recognizer/status')
        self.declare_parameter('topic_speech', '/speech/to_speak')
        self.declare_parameter('process_every_n', 1)  # process 1 of every N frames
        self.declare_parameter('enroll_samples', 5)
        self.declare_parameter('enroll_timeout_sec', 20.0)
        self.declare_parameter('enroll_interval_sec', 3.0)
        self.declare_parameter('enroll_instruction_delay_sec', 2.5)

        self.db_dir = self.get_parameter('db_dir').get_parameter_value().string_value
        self.threshold = float(self.get_parameter('threshold').value)
        self.providers = [str(p) for p in self.get_parameter('providers').value]
        det_w = int(self.get_parameter('det_size_w').value)
        det_h = int(self.get_parameter('det_size_h').value)
        self.topic_image = self.get_parameter('topic_image').value
        self.topic_output = self.get_parameter('topic_output').value
        self.topic_command = self.get_parameter('topic_command').value
        self.topic_status = self.get_parameter('topic_status').value
        self.topic_speech = self.get_parameter('topic_speech').value
        self.process_every_n = max(1, int(self.get_parameter('process_every_n').value))
        self.default_enroll_samples = max(1, int(self.get_parameter('enroll_samples').value))
        self.enroll_timeout_sec = max(1.0, float(self.get_parameter('enroll_timeout_sec').value))
        self.enroll_interval_sec = max(0.5, float(self.get_parameter('enroll_interval_sec').value))
        self.enroll_instruction_delay_sec = max(
            0.5, float(self.get_parameter('enroll_instruction_delay_sec').value)
        )

        os.makedirs(self.db_dir, exist_ok=True)
        self.db_emb_path = os.path.join(self.db_dir, 'embeddings.npy')
        self.db_names_path = os.path.join(self.db_dir, 'names.json')

        # ---------- InsightFace ----------
        self.get_logger().info(f"Initializing InsightFace providers={self.providers}")
        self.face_app = FaceAnalysis(name="buffalo_l", providers=self.providers)
        self.face_app.prepare(ctx_id=0, det_size=(det_w, det_h))

        # ---------- Load DB ----------
        self.known_embeds = []  # list of 512D float32 arrays (L2-normalized)
        self.known_names = []   # list of strings
        self._load_db()
        self._rebuild_matrix()

        # ---------- ROS I/O ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub = self.create_subscription(Image, self.topic_image, self._cb_image, qos)
        self.command_sub = self.create_subscription(String, self.topic_command, self._cb_command, 10)
        self.pub = self.create_publisher(String, self.topic_output, 10)
        self.status_pub = self.create_publisher(String, self.topic_status, 10)
        self.speech_pub = self.create_publisher(String, self.topic_speech, 10)

        self.frame_count = 0
        self.enroll_request = None
        self.status_timer = self.create_timer(1.0, self._cb_status_timer)
        self.get_logger().info(
            f"Subscribed to {self.topic_image}, publishing to {self.topic_output}. "
            f"Threshold={self.threshold}. Commands on {self.topic_command}"
        )
        self._publish_status("ready", f"DB caricata: {len(self.known_names)} volto/i memorizzati.")

    # ---------- DB helpers ----------
    def _load_db(self):
        self.known_embeds = []
        self.known_names = []

        if os.path.exists(self.db_emb_path):
            try:
                arr = np.load(self.db_emb_path)
                self.known_embeds = [row.astype(np.float32) for row in arr]
            except Exception as e:
                self.get_logger().error(f"Error loading embeddings: {e}")
        else:
            self.get_logger().warn(f"Embeddings file not found: {self.db_emb_path}")

        if os.path.exists(self.db_names_path):
            try:
                with open(self.db_names_path, 'r', encoding='utf-8') as f:
                    self.known_names = json.load(f)
            except Exception as e:
                self.get_logger().error(f"Error loading names: {e}")
        else:
            self.get_logger().warn(f"Names file not found: {self.db_names_path}")

        if len(self.known_embeds) != len(self.known_names):
            m = min(len(self.known_embeds), len(self.known_names))
            self.known_embeds = self.known_embeds[:m]
            self.known_names = self.known_names[:m]
            self.get_logger().warn("DB trimmed to match embeddings and names length.")

        self.known_embeds = [_normalize(e) for e in self.known_embeds]

    def _rebuild_matrix(self):
        if self.known_embeds:
            self.matrix = np.vstack(self.known_embeds)  # (N, 512)
        else:
            self.matrix = np.empty((0, 512), dtype=np.float32)

    def _save_db(self):
        os.makedirs(self.db_dir, exist_ok=True)
        if self.known_embeds:
            arr = np.vstack(self.known_embeds).astype(np.float32)
        else:
            arr = np.empty((0, 512), dtype=np.float32)
        np.save(self.db_emb_path, arr)
        with open(self.db_names_path, 'w', encoding='utf-8') as f:
            json.dump(self.known_names, f, ensure_ascii=False, indent=2)

    def _publish_status(self, state: str, message: str, **extra):
        out = {
            "state": state,
            "message": message,
            "known_names": sorted(set(self.known_names)),
            "known_count": len(self.known_names),
            "db_dir": self.db_dir,
        }
        out.update(extra)
        self.status_pub.publish(String(data=json.dumps(out, ensure_ascii=False)))

    def speak(self, text: str):
        """Ask the Piper ASR/TTS node to invoke its speak() function."""
        text = (text or "").strip()
        if text:
            self.speech_pub.publish(String(data=text))

    def _start_enrollment(self, name: str, samples: int):
        name = (name or "").strip()
        if not name:
            self._publish_status("error", "Nome mancante: inserisci il nome della persona.")
            return

        samples = max(1, int(samples or self.default_enroll_samples))
        self.enroll_request = {
            "name": name,
            "samples": samples,
            "embeds": [],
            "started_at": time.monotonic(),
            "next_capture_at": time.monotonic() + self.enroll_instruction_delay_sec,
        }
        pose = self._enroll_pose(0, samples)
        self._publish_status(
            "enrolling",
            f"Addestramento avviato per {name}. {pose}",
            name=name,
            collected=0,
            samples=samples,
            pose=pose,
        )
        self.speak(f"Iniziamo la memorizzazione di {name}. {pose}")

    @staticmethod
    def _enroll_pose(collected: int, samples: int) -> str:
        """Return a guided pose, spreading any sample count across useful angles."""
        poses = [
            "Guarda dritto verso la camera.",
            "Gira leggermente il viso verso destra.",
            "Gira leggermente il viso verso sinistra.",
            "Alza leggermente il mento.",
            "Abbassa leggermente il mento.",
            "Guarda dritto e sorridi leggermente.",
            "Guarda dritto con espressione neutra.",
        ]
        if samples <= 1:
            return poses[0]
        index = round(collected * (len(poses) - 1) / (samples - 1))
        return poses[min(index, len(poses) - 1)]

    def _finish_enrollment(self):
        req = self.enroll_request
        self.enroll_request = None
        if not req or not req["embeds"]:
            self._publish_status("error", "Addestramento fallito: nessun volto acquisito.")
            return

        # Keep every guided view. Recognition can then match frontal and angled
        # faces instead of comparing only with one averaged representation.
        for emb in req["embeds"]:
            self.known_embeds.append(_normalize(emb).astype(np.float32))
            self.known_names.append(req["name"])
        self._rebuild_matrix()
        self._save_db()
        self._publish_status(
            "ready",
            f"Volto di {req['name']} memorizzato con {len(req['embeds'])} campione/i.",
            name=req["name"],
            collected=len(req["embeds"]),
            samples=req["samples"],
        )
        self.speak(f"Memorizzazione di {req['name']} completata.")

    def _forget_name(self, name: str):
        name = (name or "").strip()
        if not name:
            self._publish_status("error", "Nome mancante: indica chi devo cancellare.")
            return

        keep = [(e, n) for e, n in zip(self.known_embeds, self.known_names) if n != name]
        removed = len(self.known_names) - len(keep)
        self.known_embeds = [e for e, _ in keep]
        self.known_names = [n for _, n in keep]
        self._rebuild_matrix()
        self._save_db()
        if removed:
            self._publish_status("ready", f"Cancellati {removed} campione/i di {name}.", name=name)
        else:
            self._publish_status("ready", f"Nessun campione trovato per {name}.", name=name)

    def _clear_db(self):
        self.enroll_request = None
        self.known_embeds = []
        self.known_names = []
        self._rebuild_matrix()
        self._save_db()
        self._publish_status("ready", "Database volti svuotato.")

    # ---------- Inference ----------
    def _get_embedding(self, bgr: np.ndarray):
        faces = self.face_app.get(bgr)
        if not faces:
            return None, None
        f = max(faces, key=lambda x: x.det_score)
        emb = getattr(f, 'normed_embedding', None)
        if emb is None:
            return None, None
        emb = _normalize(emb).astype(np.float32)
        bbox = getattr(f, 'bbox', None)
        if bbox is not None:
            bbox = [float(b) for b in bbox]  # [x1,y1,x2,y2]
        return emb, bbox

    def _recognize(self, emb: np.ndarray):
        if self.matrix.size == 0:
            return "Unknown", 0.0, -1
        sims = self.matrix @ emb  # cosine similarity between normalized vectors
        idx = int(np.argmax(sims))
        best = float(sims[idx])
        if best >= self.threshold:
            return self.known_names[idx], best, idx
        return "Unknown", best, idx

    # ---------- Commands ----------
    def _cb_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except Exception as e:
            self._publish_status("error", f"Comando non valido: {e}")
            return

        action = str(cmd.get("action", "")).strip().lower()
        if action in ("enroll", "train", "add"):
            self._start_enrollment(cmd.get("name", ""), cmd.get("samples", self.default_enroll_samples))
        elif action in ("status", "test"):
            self._publish_status("ready", "Riconoscimento attivo.")
        elif action == "reload":
            self._load_db()
            self._rebuild_matrix()
            self._publish_status("ready", f"DB ricaricata: {len(self.known_names)} campione/i.")
        elif action in ("forget", "delete"):
            self._forget_name(cmd.get("name", ""))
        elif action == "clear":
            self._clear_db()
        elif action == "cancel":
            self.enroll_request = None
            self._publish_status("ready", "Addestramento annullato.")
            self.speak("Addestramento annullato.")
        else:
            self._publish_status("error", f"Azione sconosciuta: {action}")

    def _cb_status_timer(self):
        req = self.enroll_request
        if not req:
            return
        elapsed = time.monotonic() - req["started_at"]
        timeout = max(self.enroll_timeout_sec, req["samples"] * self.enroll_interval_sec + 15.0)
        if elapsed > timeout:
            self.enroll_request = None
            self._publish_status(
                "error",
                f"Addestramento di {req['name']} scaduto: volto non acquisito in tempo.",
                name=req["name"],
                collected=len(req["embeds"]),
                samples=req["samples"],
            )
            self.speak(f"Addestramento di {req['name']} non riuscito. Riprova guardando la camera.")

    # ---------- Callback ----------
    def _cb_image(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        try:
            bgr = _rosimg_to_bgr8(msg)
        except Exception as e:
            # Log every ~30 processed frames to avoid spam
            if self.frame_count % 30 == 0:
                self.get_logger().warn(f"Unprocessable frame: {e}")
            return

        emb, bbox = self._get_embedding(bgr)
        if emb is None:
            if self.enroll_request is not None:
                req = self.enroll_request
                self._publish_status(
                    "enrolling",
                    f"Sto cercando un volto per {req['name']}...",
                    name=req["name"],
                    collected=len(req["embeds"]),
                    samples=req["samples"],
                )
            out = {
                "name": "NoFace",
                "similarity": 0.0,
                "bbox": None,
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec}
            }
            self.pub.publish(String(data=json.dumps(out)))
            return

        if self.enroll_request is not None:
            req = self.enroll_request
            now = time.monotonic()
            if now < req["next_capture_at"]:
                return
            req["embeds"].append(emb)
            collected = len(req["embeds"])
            pose = self._enroll_pose(collected, req["samples"]) if collected < req["samples"] else "Acquisizione completata."
            self._publish_status(
                "enrolling",
                f"Campione {collected}/{req['samples']} acquisito. {pose}",
                name=req["name"],
                collected=collected,
                samples=req["samples"],
                pose=pose,
            )
            if collected >= req["samples"]:
                self._finish_enrollment()
            else:
                req["next_capture_at"] = now + self.enroll_interval_sec
                self.speak(pose)

        name, sim, idx = self._recognize(emb)
        out = {
            "name": name,
            "similarity": sim,
            "bbox": bbox,  # [x1,y1,x2,y2] in pixels
            "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec}
        }
        self.pub.publish(String(data=json.dumps(out)))


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
