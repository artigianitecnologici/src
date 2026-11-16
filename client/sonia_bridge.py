"""Adattatore HTTP per il bridge ASR ROS, sulla connessione del client Sonia."""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def start_robot_bridge(connection, host="127.0.0.1", port=5003, timeout=120):
    lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, value):
            body = json.dumps(value, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path != "/json":
                self.reply(404, {"error": "Endpoint sconosciuto"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 65536:
                    raise ValueError("Dimensione richiesta non valida")
                payload = json.loads(self.rfile.read(size))
                text = payload.get("query") if isinstance(payload, dict) else None
                if not isinstance(text, str) or not text.strip():
                    raise ValueError("Parametro query mancante")
            except (ValueError, UnicodeError) as exc:
                self.reply(400, {"error": str(exc)})
                return
            if not lock.acquire(blocking=False):
                self.reply(409, {"error": "Richiesta robot già in corso"})
                return
            try:
                session = connection.session
                if not connection.connected or session is None:
                    self.reply(503, {"error": "Collega prima il client a Sonia"})
                    return
                state = connection._request(session, "/api/state")
                if state["busy"]:
                    self.reply(409, {"error": "Sonia sta elaborando un'altra richiesta"})
                    return
                after = state["sequence"]
                connection._request(session, "/api/chat", {"message": text.strip()})
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    if session is not connection.session or not connection.connected:
                        raise RuntimeError("Client scollegato durante la richiesta")
                    state = connection._request(session, "/api/events?after=" + str(after))
                    if state.get("reset_needed"):
                        raise RuntimeError("Cronologia eventi scaduta: risposta non recuperabile")
                    for event in state["events"]:
                        if event["kind"] == "answer":
                            self.reply(200, {"response": event["value"]})
                            return
                        if event["kind"] in ("error", "reset"):
                            raise RuntimeError(str(event["value"]))
                    after = state["sequence"]
                    time.sleep(0.3)
                self.reply(504, {"error": "Tempo di attesa della risposta Sonia esaurito"})
            except Exception as exc:
                self.reply(502, {"error": str(exc)})
            finally:
                lock.release()

        def log_message(self, format, *args):
            pass

    server = ThreadingHTTPServer((host, port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
