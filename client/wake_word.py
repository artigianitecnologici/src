"""Ascolto vocale locale con Vosk e attivazione esplicita tramite nome."""
import json
from array import array
import sys
from pathlib import Path
import queue
import re
import threading
import time

MODEL_PATH = Path(__file__).resolve().parent / 'models/vosk-model-small-it-0.22'

CONFIG_PATH = Path(__file__).resolve().parent / 'config.json'


def load_wake_word(path=CONFIG_PATH):
    path = Path(path)
    try:
        config = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        raise ValueError(f'Configurazione wake word non leggibile ({path}): {exc}') from exc
    word = config.get('wake_word') if isinstance(config, dict) else None
    if not isinstance(word, str):
        raise ValueError(f'{path}: wake_word deve essere una stringa')
    word = ' '.join(word.casefold().split())
    if not word or len(word) > 40 or not all(part.isalpha() for part in word.split()):
        raise ValueError(f'{path}: wake_word deve contenere solo parole, da 1 a 40 caratteri')
    return word


class WakeGate:
    """Accetta nome e domanda nella stessa frase oppure entro dieci secondi."""
    def __init__(self, word='sonia'):
        self.prefix = re.compile(r'^\s*(?:(?:ehi|ciao|hey)\W+)?' + re.escape(word) + r'\b[\W_]*', re.I)
        self.until = 0.0

    def reset(self):
        self.until = 0.0

    def accept(self, text, now=None):
        now = time.monotonic() if now is None else now
        if now >= self.until:
            self.until = 0.0
        text = text.strip()
        match = self.prefix.match(text)
        if match:
            command = text[match.end():].strip()
            self.until = 0.0 if command else now + 10
            return command or None
        if self.until > now and text:
            self.until = 0.0
            return text
        return None


def confirmed_wake(result, word, min_confidence=0.85):
    """Conferma il nome soltanto nel risultato finale del decoder libero.

    Le ipotesi parziali, i nomi citati a metà frase e le parole con bassa
    confidenza non possono autorizzare l'invio di audio al server.
    """
    if not WakeGate(word).prefix.match(result.get('text', '')):
        return False
    words = result.get('result', [])
    if not isinstance(words, list) or not words:
        return False
    expected = word.casefold().split()
    first = words[0].get('word', '').casefold()
    offset = 1 if first in ('ehi', 'ciao', 'hey') and first != expected[0] else 0
    selected = words[offset:offset + len(expected)]
    if len(selected) != len(expected):
        return False
    for item, token in zip(selected, expected):
        try:
            if (item.get('word', '').casefold() != token
                    or not min_confidence <= float(item.get('conf', 0)) <= 1
                    or not float(item['end']) > float(item['start']) >= 0):
                return False
        except (TypeError, ValueError, KeyError):
            return False
    return True


class WakeListener:
    def __init__(self, emit, available, device=None, word='sonia', model_path=MODEL_PATH,
                 pause_reason=None):
        self.emit, self.available, self.device = emit, available, device
        self.pause_reason = pause_reason
        self.word, self.model_path = word, Path(model_path)
        self.active = False
        self.stop_event = None
        self.thread = None
        self.generation = 0

    def start(self):
        if self.thread and self.thread.is_alive():
            return False
        self.stop_event = threading.Event()
        self.generation += 1
        self.active = True
        self.thread = threading.Thread(target=self._run, args=(self.stop_event, self.generation), daemon=True)
        self.thread.start()
        return True

    def stop(self):
        self.generation += 1
        self.active = False
        if self.stop_event:
            self.stop_event.set()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=1)

    def _run(self, stop, generation):
        audio = queue.Queue(maxsize=30)
        try:
            import sounddevice as sd
            from vosk import Model, KaldiRecognizer
            if not (self.model_path / 'am/final.mdl').is_file():
                raise RuntimeError('Modello italiano assente. Esegui bash client/installa_wake_word.sh')
            self.emit('wake_status', 'Caricamento riconoscimento italiano…')
            model = Model(str(self.model_path))
            # Il vocabolario completo permette al decoder di scegliere parole
            # diverse dal nome: una grammatica limitata al nome lo forzava.
            recognizer = KaldiRecognizer(model, 16000)
            recognizer.SetWords(True)
            recorded = bytearray()
            paused = False
            last_pause_reason = None
            waiting = f'In attesa della parola «{self.word.capitalize()}»'
            activated = False
            silence = 0.0
            endpoint_remaining = None
            if stop.is_set():
                return

            def callback(indata, frames, timing, status):
                # Nessuna verifica di rete o del dispositivo di uscita nel
                # callback PortAudio. Il worker scarta i frame durante la pausa.
                if stop.is_set():
                    return
                try:
                    audio.put_nowait(bytes(indata))
                except queue.Full:
                    # Perdita di audio: azzera anche la frase per non inviare testo incompleto.
                    stop.set()
                    self.emit('log', 'Errore wake word: client troppo occupato, riattiva l’ascolto.')

            with sd.RawInputStream(samplerate=16000, blocksize=1600, device=self.device,
                                   dtype='int16', channels=1, callback=callback):
                self.emit('wake_status', waiting)
                while not stop.is_set():
                    reason = (self.pause_reason() if self.pause_reason is not None else
                              ('' if self.available() else 'attendo la fine della risposta'))
                    if reason:
                        if not paused or reason != last_pause_reason:
                            self.emit('wake_status', 'Ascolto sospeso · ' + reason)
                        last_pause_reason = reason
                        paused = True
                        recognizer.Reset()
                        recorded.clear()
                        activated = False
                        silence = 0.0
                        endpoint_remaining = None
                        while True:
                            try:
                                audio.get_nowait()
                            except queue.Empty:
                                break
                        stop.wait(.1)
                        continue
                    if paused:
                        paused = False
                        last_pause_reason = None
                        self.emit('wake_status', waiting)
                    try:
                        data = audio.get(timeout=.1)
                    except queue.Empty:
                        continue
                    # Buffer locale della frase, mai inviato senza conferma.
                    # Serve a non perdere l'inizio aspettando il risultato finale.
                    recorded.extend(data)
                    samples = array('h', data[:len(data) // 2 * 2])
                    if sys.byteorder != 'little':
                        samples.byteswap()
                    duration = len(samples) / 16000
                    energy = sum(v * v for v in samples) / max(1, len(samples))
                    if energy > 300 ** 2:
                        silence = 0.0
                    else:
                        silence += duration
                    finished = recognizer.AcceptWaveform(data)
                    capped = len(recorded) >= 16000 * 2 * 30
                    if finished or capped:
                        result = recognizer.Result() if finished else recognizer.FinalResult()
                        decoded = json.loads(result)
                        if not activated and confirmed_wake(decoded, self.word):
                            activated = True
                            self.emit('wake_status', f'{self.word.capitalize()} confermata · registro una richiesta')
                    # Con il decoder libero il risultato finale confermato è
                    # già una fine frase: non aspettare una seconda soglia RMS,
                    # che il rumore del microfono potrebbe non raggiungere mai.
                    if endpoint_remaining is not None:
                        endpoint_remaining -= duration
                    if activated and finished and endpoint_remaining is None:
                        endpoint_remaining = .6
                    ready = activated and (silence >= 1.2 or
                                           endpoint_remaining is not None and endpoint_remaining <= 0)
                    if ready or (activated and capped):
                        self.emit('wake_audio', (generation, bytes(recorded)))
                        stop.wait(.5)
                        # I frame già accodati appartengono alla richiesta appena
                        # inviata e non devono riattivare il riconoscitore.
                        while True:
                            try:
                                audio.get_nowait()
                            except queue.Empty:
                                break
                        paused = True
                    if ready or capped or (finished and not activated):
                        recorded.clear()
                        activated = False
                        silence = 0.0
                        endpoint_remaining = None
                        recognizer.Reset()

        except Exception as exc:
            self.emit('log', f'Errore wake word: {exc}. Installa con bash client/installa_wake_word.sh')
        finally:
            self.active = False
            self.emit('wake_stopped', None)
