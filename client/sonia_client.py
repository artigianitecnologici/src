#!/usr/bin/env python3
"""Client desktop LAN per Sonia Lite Web: chat HTTP, dettatura e TTS remoti."""
from __future__ import annotations

import argparse
import json
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk
from sonia_http import SoniaConnection, normalize_url
from wake_word import WakeListener, CONFIG_PATH, load_wake_word

try:
    import sounddevice as sd
except (ImportError, OSError):
    sd = None

MIC_RATE = 16000
MIC_FRAMES = 512              # Blocchi microfono da 32 ms
BG = "#101820"
PANEL = "#182630"
INK = "#eef4f6"
MUTED = "#a6b8c3"
ACCENT = "#4abfa3"
WARNING = "#efb366"



class AudioPlayback:
    """Jitter buffer in RAM per i pacchetti PCM16 mono inviati da Sonia."""
    def __init__(self, on_error, device=None):
        self.on_error = on_error
        self.device = device
        self.rate = 24000
        self.stream = None
        self.buffer = bytearray()
        self.lock = threading.Lock()
        self.playing = False
        self.last_data = 0.0
        self.last_output = 0.0
        self.enabled = True

    def start(self, rate: int):
        self.close()
        if sd is None:
            self.on_error("Audio non disponibile: installa sounddevice e PortAudio.")
            return
        self.rate = int(rate)
        if not 8000 <= self.rate <= 192000:
            self.on_error("Frequenza audio non valida ricevuta da Sonia")
            return
        frames = max(160, round(self.rate * 0.02))
        try:
            self.stream = sd.RawOutputStream(
                samplerate=self.rate, channels=1, dtype="int16",
                blocksize=frames, device=self.device, callback=self._callback,
            )
            self.stream.start()
        except Exception as exc:
            self.stream = None
            self.on_error(f"Uscita audio non disponibile: {exc}")

    def add(self, data: bytes):
        if not self.enabled or not self.stream or not data:
            return
        if len(data) % 2:
            data = data[:-1]
        with self.lock:
            self.buffer.extend(data)
            self.last_data = time.monotonic()
            # I WAV del server sono limitati a 20 MB: conservarli interi.
            maximum = 20_000_000
            if len(self.buffer) > maximum:
                del self.buffer[: len(self.buffer) - maximum]

    def _callback(self, outdata, frames, time_info, status):
        needed = frames * 2
        packet = b""
        with self.lock:
            if not self.enabled:
                self.buffer.clear()
                self.playing = False
            if not self.playing and self.buffer:
                # 120 ms di prefilling contro jitter Wi-Fi; i frammenti brevi
                # partono comunque dopo 160 ms, senza rimanere bloccati.
                if len(self.buffer) >= self.rate * 2 * 0.12 or (time.monotonic() - self.last_data) > 0.16:
                    self.playing = True
            if self.playing:
                n = min(needed, len(self.buffer))
                packet = bytes(self.buffer[:n])
                del self.buffer[:n]
                if n < needed:
                    self.playing = False
        if packet:
            self.last_output = time.monotonic()
        outdata[:] = packet + bytes(needed - len(packet))

    def flush(self):
        with self.lock:
            self.buffer.clear()
            self.playing = False

    def is_busy(self, now=None):
        """Una coda senza un dispositivo funzionante non deve bloccare il microfono."""
        now = time.monotonic() if now is None else now
        with self.lock:
            if not self.buffer:
                return False
            stream = self.stream
            last_progress = max(self.last_data, self.last_output)
        try:
            active = stream is not None and bool(stream.active)
        except Exception:
            active = False
        if not active or now - last_progress > 2:
            self.flush()
            self.on_error("Errore audio: riproduzione interrotta; coda svuotata e ascolto ripristinato.")
            return False
        return True

    def close(self):
        stream, self.stream = self.stream, None
        if stream is not None:
            try:
                stream.stop()
                stream.close()
            except Exception:
                pass
        self.flush()


class Microphone:
    """Input stream solo quando l'utente accende il microfono."""
    def __init__(self, on_error, device=None):
        self.on_error = on_error
        self.device = device
        self.stream = None
        self.recorded = bytearray()
        self.active = False
        self.lock = threading.Lock()

    def start(self) -> bool:
        if sd is None:
            self.on_error("Microfono non disponibile: installa sounddevice e PortAudio.")
            return False
        if self.active:
            return True
        self.recorded.clear()
        try:
            stream = sd.RawInputStream(
                samplerate=MIC_RATE, channels=1, dtype="int16",
                blocksize=MIC_FRAMES, device=self.device,
                callback=self._callback,
            )
            stream.start()
            with self.lock:
                self.stream = stream
                self.active = True
            return True
        except Exception as exc:
            self.on_error(f"Microfono non disponibile: {exc}")
            return False

    def _callback(self, indata, frames, time_info, status):
        if not self.active or frames != MIC_FRAMES:
            return
        with self.lock:
            if len(self.recorded) + len(indata) <= MIC_RATE * 2 * 60:
                self.recorded.extend(bytes(indata))

    def stop(self):
        with self.lock:
            self.active = False
            stream, self.stream = self.stream, None
        if stream is not None:
            try:
                stream.stop()
                stream.close()
            except Exception:
                pass

    def finish(self):
        self.stop()
        with self.lock:
            data = bytes(self.recorded)
            self.recorded.clear()
        return data

class SoniaUI:
    def __init__(self, root: tk.Tk, url: str, input_device=None, output_device=None, config_path=CONFIG_PATH):
        self.root = root
        self.root.title("Sonia · client Python")
        self.root.geometry("860x690")
        self.root.minsize(650, 520)
        self.root.configure(bg=BG)
        self.events = queue.Queue()
        self.playback = AudioPlayback(self.error, device=output_device)
        self.microphone = Microphone(self.error, device=input_device)
        self.connection = SoniaConnection(
            self.emit, self.playback, self.microphone, emit_session=self._emit_session,
        )
        self.wake_config = config_path
        self.wake_hold_until = 0.0
        self.wake = WakeListener(self.emit, self._wake_available, device=input_device,
                                 word=load_wake_word(self.wake_config),
                                 pause_reason=self._wake_pause_reason)
        self._build(url)
        self.root.after(60, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

    def emit(self, kind, data):
        self.events.put((None, kind, data))

    def _emit_session(self, generation, kind, data):
        self.events.put((generation, kind, data))

    def error(self, msg):
        self.emit("log", msg)

    def _btn(self, parent, text, fn, color=PANEL):
        return tk.Button(parent, text=text, command=fn, bg=color, fg=INK, activebackground=ACCENT,
                         activeforeground="#071614", bd=0, padx=14, pady=10, cursor="hand2",
                         font=("DejaVu Sans", 10, "bold"))

    def _build(self, url):
        head = tk.Frame(self.root, bg=BG)
        head.pack(fill="x", padx=18, pady=(17, 10))
        tk.Label(head, text="SONIA", fg=ACCENT, bg=BG, font=("DejaVu Sans", 22, "bold")).pack(side="left")
        tk.Label(head, text="  client remoto · Python", fg=MUTED, bg=BG,
                 font=("DejaVu Sans", 11)).pack(side="left")
        self.status = tk.Label(head, text="● scollegata", fg=WARNING, bg=BG,
                               font=("DejaVu Sans", 11))
        self.status.pack(side="right")

        bar = tk.Frame(self.root, bg=BG)
        bar.pack(fill="x", padx=18, pady=(0, 9))
        self.url = tk.StringVar(value=url)
        self.url_entry = tk.Entry(bar, textvariable=self.url, bg=PANEL, fg=INK, bd=0,
                                  insertbackground=INK, font=("DejaVu Sans", 11))
        self.url_entry.pack(side="left", fill="x", expand=True, ipady=10, padx=(0, 8))
        self.connect_btn = self._btn(bar, "Collega", self._connect, ACCENT)
        self.connect_btn.configure(fg="#071614")
        self.connect_btn.pack(side="left", padx=(0, 6))
        self.disconnect_btn = self._btn(bar, "Scollega", self._disconnect)
        self.disconnect_btn.pack(side="left")

        detail = tk.Frame(self.root, bg=BG)
        detail.pack(fill="x", padx=18, pady=(0, 8))
        self.model_label = tk.Label(detail, text="Modello: —", bg=BG, fg=MUTED,
                                    font=("DejaVu Sans", 10))
        self.model_label.pack(side="left")
        self.audio_label = tk.Label(detail, text="Audio: —", bg=BG, fg=MUTED,
                                    font=("DejaVu Sans", 10))
        self.audio_label.pack(side="right")

        # Prenota prima lo spazio inferiore: la chat si espande solo nello spazio rimasto.
        self.log_label = tk.Label(self.root, text="Accesso diretto LAN: usa http://192.168.1.199:8096",
                                  fg=MUTED, bg=BG, anchor="w", font=("DejaVu Sans", 9))
        self.log_label.pack(side="bottom", fill="x", padx=19, pady=(0, 12))

        controls = tk.Frame(self.root, bg=BG)
        controls.pack(side="bottom", fill="x", padx=18, pady=(0, 7))
        self.mic_btn = self._btn(controls, "🎤 Parla", self._toggle_mic)
        self.mic_btn.pack(side="left", padx=(0, 7))
        self.wake_btn = self._btn(controls, f"Ascolto {self.wake.word.capitalize()} OFF", self._toggle_wake)
        self.wake_btn.pack(side="left", padx=(0, 7))
        self.speaker_btn = self._btn(controls, "🔊 Audio ON", self._toggle_speaker)
        self.speaker_btn.pack(side="left", padx=(0, 7))
        self._btn(controls, "■ Ferma audio", lambda: self.connection.send({"type": "stop"})).pack(side="left", padx=(0, 7))
        self._btn(controls, "Nuova chat", self._reset).pack(side="right")

        inputrow = tk.Frame(self.root, bg=BG)
        inputrow.pack(side="bottom", fill="x", padx=18, pady=(0, 8))
        self.entry = tk.Entry(inputrow, bg=PANEL, fg=INK, insertbackground=INK,
                              font=("DejaVu Sans", 12), bd=0)
        self.entry.pack(side="left", fill="x", expand=True, ipady=12, padx=(0, 8))
        self.entry.bind("<Return>", lambda e: self._send_text())
        self._btn(inputrow, "Invia", self._send_text, ACCENT).pack(side="left")

        self.partial = tk.Label(self.root, text="🎤 Microfono spento · premi 'Parla'",
                                fg=MUTED, bg=BG, anchor="w", font=("DejaVu Sans", 10))
        self.partial.pack(side="bottom", fill="x", padx=19, pady=(5, 2))

        self.chat = scrolledtext.ScrolledText(
            self.root, wrap=tk.WORD, bg="#142029", fg=INK, relief="flat", bd=0,
            insertbackground=INK, padx=14, pady=12, font=("DejaVu Sans", 11),
            state="disabled", spacing2=3, spacing3=8,
        )
        self.chat.pack(fill="both", expand=True, padx=18, pady=4)
        self.chat.tag_configure("you", foreground="#86d5fe", font=("DejaVu Sans", 11, "bold"))
        self.chat.tag_configure("sonia", foreground=ACCENT, font=("DejaVu Sans", 11, "bold"))
        self.chat.tag_configure("meta", foreground=MUTED, font=("DejaVu Sans", 9))
        self._system("Client pronto. Collega Sonia, poi scrivi oppure attiva il microfono.")

    def _append(self, txt, tag=None):
        self.chat.configure(state="normal")
        self.chat.insert("end", txt, tag)
        self.chat.see("end")
        self.chat.configure(state="disabled")

    def _system(self, msg):
        self._append(f"{msg}\n", "meta")

    def _chatline(self, role, txt):
        self._append("\nTU\n" if role == "user" else "\nSONIA\n", "you" if role == "user" else "sonia")
        self._append(str(txt).strip() + "\n")

    def _connect(self):
        try:
            url = normalize_url(self.url.get())
        except ValueError as exc:
            messagebox.showerror("Indirizzo non valido", str(exc), parent=self.root)
            return
        self._stop_wake()
        self.status.config(text="● connessione…", fg=WARNING)
        self.connection.connect(url)

    def _disconnect(self):
        self._stop_wake()
        self.connection.disconnect()
        self.playback.close()
        self.mic_btn.config(text="🎤 Parla", bg=PANEL)
        self.status.config(text="● scollegata", fg=WARNING)

    def _send_text(self):
        text = self.entry.get().strip()
        if not text:
            return
        if self.connection.send({"type": "text", "text": text}):
            self.entry.delete(0, "end")
        else:
            self._system("Non connessa. Premi 'Collega'.")

    def _wake_available(self):
        return not self._wake_pause_reason()

    def _wake_pause_reason(self):
        now = time.monotonic()
        if not self.connection.connected:
            return "connessione al server assente"
        if self.microphone.active:
            return "registrazione manuale in corso"
        if self.connection.transcribing:
            return "trascrizione in corso sul server"
        if self.connection.busy:
            return "il modello sul server sta elaborando la richiesta"
        if self.playback.is_busy(now):
            return "riproduzione della risposta vocale"
        if now <= self.playback.last_output + 1:
            return "fine della riproduzione vocale"
        if self.playback.enabled and now < self.connection.tts_until:
            return "preparazione della voce sul server (massimo 15 secondi)"
        if now < self.wake_hold_until:
            return "pausa breve dopo l'invio della registrazione"
        return ""

    def _stop_wake(self):
        self.wake.stop()
        self.wake_btn.config(text=f"Ascolto {self.wake.word.capitalize()} OFF", bg=PANEL)

    def _toggle_wake(self):
        if self.wake.active:
            self._stop_wake()
            self.partial.config(text="Ascolto continuo disattivato", fg=MUTED)
            return
        if not self.connection.connected:
            self._system("Connetti prima Sonia")
            return
        try:
            self.wake.word = load_wake_word(self.wake_config)
        except ValueError as exc:
            self._system(f"Errore: {exc}")
            return
        self.microphone.stop()
        self.mic_btn.config(text="🎤 Parla", bg=PANEL)
        if self.wake.start():
            self.wake_btn.config(text=f"Ascolto {self.wake.word.capitalize()} ON", bg=ACCENT)
            self.partial.config(text="Caricamento ascolto locale…", fg=ACCENT)

    def _toggle_mic(self):
        self._stop_wake()
        if not self.connection.connected:
            self._system("Connetti prima Sonia")
            return
        if self.microphone.active:
            self.connection.transcribe(self.microphone.finish())
            self.mic_btn.config(text="🎤 Parla", bg=PANEL)
            self.partial.config(text="🎤 Microfono spento · attendo la risposta", fg=MUTED)
        elif self.microphone.start():
            self.mic_btn.config(text="🔴 Termina frase", bg="#a84b58")
            self.partial.config(text="🎤 Microfono acceso · parla, poi premi 'Termina frase'", fg=ACCENT)

    def _toggle_speaker(self):
        self.playback.enabled = not self.playback.enabled
        if not self.playback.enabled:
            self.playback.flush()
        self.speaker_btn.config(text="🔊 Audio ON" if self.playback.enabled else "🔇 Audio OFF")

    def _reset(self):
        if not self.connection.connected:
            return
        if messagebox.askyesno("Nuova chat", "Cancellare la conversazione sul server Sonia?", parent=self.root):
            self.connection.send({"type": "reset"})

    def _poll(self):
        try:
            for _ in range(60):
                generation, kind, obj = self.events.get_nowait()
                if generation is not None and generation != self.connection.generation:
                    continue
                if kind == "log":
                    self.log_label.config(text=str(obj)[:190])
                    if any(x in str(obj).lower() for x in ("errore", "non disponibile", "fallito", "sostituita")):
                        self._system(str(obj))
                elif kind == "wake_status":
                    if self.wake.active:
                        self.partial.config(text=str(obj), fg=ACCENT)
                elif kind == "wake_audio":
                    wake_generation, pcm = obj
                    if self.wake.active and wake_generation == self.wake.generation:
                        if not self._wake_available():
                            self._system("Registrazione non inviata: Sonia è occupata o sta riproducendo audio. Riprova quando termina.")
                            continue
                        self.wake_hold_until = time.monotonic() + 2
                        if self.connection.transcribe(pcm):
                            self.partial.config(text=f"{self.wake.word.capitalize()} attivata · trascrizione della domanda…", fg=ACCENT)
                        else:
                            self._system("Trascrizione non avviata: controlla la connessione o attendi la trascrizione in corso.")
                elif kind == "wake_stopped":
                    if not self.wake.active:
                        self.wake_btn.config(text=f"Ascolto {self.wake.word.capitalize()} OFF", bg=PANEL)
                        self.partial.config(text="Ascolto continuo disattivato", fg=MUTED)
                elif kind == "connection":
                    self.status.config(text="● connessa" if obj else "● scollegata",
                                       fg=ACCENT if obj else WARNING)
                    if not obj:
                        self._stop_wake()
                        self.microphone.stop()
                        self.playback.close()
                        self.mic_btn.config(text="🎤 Parla", bg=PANEL)
                elif kind == "audio":
                    self.playback.add(obj)
                elif kind == "wav":
                    rate, pcm = obj
                    if self.playback.enabled and self.connection.session and not self.connection.session['muted']:
                        self.playback.start(rate)
                        self.playback.add(pcm)
                        self.audio_label.config(text=f"Voce dal server · {rate} Hz")
                elif kind == "event":
                    self._event(obj)
        except queue.Empty:
            pass
        self.root.after(60, self._poll)

    def _event(self, ev):
        kind = ev.get("type")
        if kind == "hello":
            self.model_label.config(text=f"Modello: {ev.get('model', '?')}")
            self.audio_label.config(text="Voce: TTS del server Sonia")
            self.chat.configure(state="normal")
            self.chat.delete("1.0", "end")
            self.chat.configure(state="disabled")
            for m in (ev.get("history") or [])[-100:]:
                if m.get("role") in ("user", "assistant"):
                    self._chatline(m["role"], m.get("content", ""))
            self._system("Connessa a Sonia Lite Web. Modello e servizi audio sono sul server.")
            self._state(ev.get("state", "listening"))
        elif kind == "state":
            self._state(ev.get("state", ""))
        elif kind == "user":
            self._chatline("user", ev.get("text", ""))
            self._state("thinking")
        elif kind == "assistant_done":
            self._chatline("assistant", ev.get("text", ""))
        elif kind == "chat_reset":
            self.chat.configure(state="normal")
            self.chat.delete("1.0", "end")
            self.chat.configure(state="disabled")
            self._system("Nuova chat avviata")
        elif kind == "command_request":
            cmd_id = ev.get("id")
            text = ev.get("command", "")
            ok = messagebox.askyesno("Sonia chiede il permesso", f"Consentire sul server Sonia questo comando?\n\n{text}\n\n{ev.get('reason', '')}", parent=self.root)
            self.connection.send({"type": "approve", "id": cmd_id, "ok": bool(ok)})
        elif kind == "model_ready":
            self.model_label.config(text=f"Modello: {ev.get('model', '?')}")

    def _state(self, state):
        labels = {"listening": "in ascolto", "user": "ascolta la domanda", "thinking": "ragiona",
                  "speaking": "parla", "confirm": "attende approvazione", "paused": "in pausa", "offline": "offline"}
        self.status.config(text="● " + labels.get(state, state), fg=WARNING if state == "offline" else ACCENT)

    def _close(self):
        self.wake.stop()
        self.connection.disconnect()
        self.playback.close()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="Client desktop Python per Sonia Lite Web")
    parser.add_argument("--url", default="http://192.168.1.199:8096",
                        help="URL LAN di Sonia, ad esempio http://192.168.1.199:8096")
    parser.add_argument("--input-device", type=int, help="Indice del microfono PortAudio")
    parser.add_argument("--output-device", type=int, help="Indice dell'altoparlante PortAudio")
    parser.add_argument("--list-devices", action="store_true", help="Elenca i dispositivi audio disponibili")
    parser.add_argument("--robot-bridge", action="store_true", help="Abilita endpoint ASR /json per ROS")
    parser.add_argument("--robot-host", default="127.0.0.1", help="Indirizzo di ascolto del bridge robot")
    parser.add_argument("--robot-port", type=int, default=5003, help="Porta del bridge robot")
    parser.add_argument("--config", default=str(CONFIG_PATH), help="File JSON di configurazione del client")
    args = parser.parse_args()
    if args.list_devices:
        if sd is None:
            parser.error("sounddevice non installato")
        print(sd.query_devices())
        return
    try:
        load_wake_word(args.config)
    except ValueError as exc:
        parser.error(str(exc))
    root = tk.Tk()
    ui = SoniaUI(root, args.url, args.input_device, args.output_device, config_path=args.config)
    bridge = None
    if args.robot_bridge:
        from sonia_bridge import start_robot_bridge
        try:
            bridge = start_robot_bridge(ui.connection, args.robot_host, args.robot_port)
        except (OSError, ValueError) as exc:
            root.destroy()
            parser.error(f"Avvio bridge robot fallito: {exc}")
        ui._system(f"Bridge robot: http://{args.robot_host}:{args.robot_port}/json")
    try:
        root.mainloop()
    finally:
        if bridge:
            bridge.shutdown()
            bridge.server_close()

if __name__ == "__main__":
    main()
