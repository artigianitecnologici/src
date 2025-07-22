import sounddevice as sd
import queue
import json
from vosk import Model, KaldiRecognizer
import marrtinorobot2.marrtinorobot2_chatbot.marrtinorobot2_chatbot.no_asr_whisper as no_asr_whisper
import numpy as np
import time
from gtts import gTTS
import io
import soundfile as sf

# Carica la configurazione dal file JSON
with open("config.json", "r") as f:
    config = json.load(f)

# Inizializza il modello Vosk
model_vosk = Model(config["model_vosk_path"])

# Inizializza il modello Whisper
#model_whisper = whisper.load_model(config["whisper_model"])
model_path = config["whisper_model"]
#model_whisper = whisper.load_model(model_path)
model_whisper = no_asr_whisper.load_model("small")  # Usa "small" per l'italiano

# Frequenze di campionamento
input_samplerate = config["input_samplerate"]
output_samplerate = config["output_samplerate"]
q = queue.Queue()

# Indici dei dispositivi audio
input_device_index = config["input_device_index"]
output_device_index = config["output_device_index"]

# Imposta il dispositivo di input su ReSpeaker e di output su HDMI
sd.default.device = (input_device_index, output_device_index)

# Variabili globali
stop_listening = False
listening_active = True  # Controlla se il microfono è attivo

# Funzione di callback per aggiungere i dati audio alla coda
def audio_callback(indata, frames, time, status):
    if status:
        print(status)
    if listening_active:
        q.put(bytes(indata))

# Funzione per riprodurre un suono beep
def play_beep(frequency=1000, duration=0.5, volume=0.5):
    t = np.linspace(0, duration, int(output_samplerate * duration), False)
    wave = (volume * np.sin(2 * np.pi * frequency * t)).astype(np.float32)
    try:
        sd.stop()  # Assicurati che qualsiasi stream audio precedente sia chiuso
        with sd.OutputStream(samplerate=output_samplerate, channels=1, dtype='float32', device=output_device_index) as stream:
            stream.write(wave)
        print("Suono beep riprodotto.")
    except Exception as e:
        print(f"Errore durante la riproduzione del beep: {e}")

# Funzione per riprodurre l'audio TTS con gTTS
def play_tts(text, lang="it"):
    global listening_active
    try:
        listening_active = False  # Disabilita temporaneamente il microfono
        sd.stop()  # Assicurati che qualsiasi stream audio precedente sia chiuso
        
        tts = gTTS(text=text, lang=lang)
        audio_data = io.BytesIO()
        tts.write_to_fp(audio_data)
        audio_data.seek(0)

        # Leggi i dati audio dalla memoria
        data, samplerate = sf.read(audio_data, dtype='float32')
        
        # Riproduci l'audio con sounddevice
        with sd.OutputStream(samplerate=output_samplerate, channels=1, dtype='float32', device=output_device_index) as stream:
            stream.write(data)
        
        print(f"Riproduzione TTS: {text}")
    except Exception as e:
        print(f"Errore durante la riproduzione TTS: {e}")
    finally:
        listening_active = True  # Riattiva il microfono

# Funzione per ascoltare la parola chiave
def listen_for_keyword():
    global stop_listening
    recognizer = KaldiRecognizer(model_vosk, input_samplerate)
    print(f"In ascolto della parola chiave... (e.g., '{config['keyword']}')")
    try:
        with sd.RawInputStream(samplerate=input_samplerate, dtype='int16',
                               channels=1, callback=audio_callback, device=input_device_index,
                               blocksize=16000):
            while not stop_listening:
                if not q.empty():
                    data = q.get()
                    if recognizer.AcceptWaveform(data):
                        result = json.loads(recognizer.Result())
                        detected_text = result.get('text', '').lower().strip()
                        if detected_text:
                            print("Rilevato:", detected_text)
                            if config['keyword'] in detected_text:
                                play_beep()
                                transcribe_with_whisper()
                            elif "stop" in detected_text:
                                play_tts("Dispositivo in arresto. Arrivederci!")
                                stop_listening = True
                                break
                else:
                    time.sleep(0.1)
    except Exception as e:
        print(f"Errore nell'apertura dello stream di input: {e}")

def transcribe_with_whisper(duration=5):
    global listening_active
    print("Inizia a parlare...")

    listening_active = True  # Assicurati che il microfono sia attivo per la registrazione

    frames_to_capture = int(input_samplerate / 16000 * duration)
    audio_frames = []

    while len(audio_frames) < frames_to_capture:
        if not q.empty():
            audio_frames.append(q.get())
        else:
            time.sleep(0.01)

    # Unisci i frame acquisiti
    audio_data = b''.join(audio_frames)

    # Converti audio in numpy array
    audio_np = np.frombuffer(audio_data, dtype=np.int16)

    # Salva l'audio in formato WAV
    sf.write("audio.wav", audio_np, input_samplerate)

    # Trascrizione Whisper
    try:
        result = model_whisper.transcribe("audio.wav", language="it")
        print("Hai detto:", result["text"])
        play_tts(f"Hai detto: {result['text']}")
    except Exception as e:
        print(f"Errore durante la trascrizione Whisper: {e}")


# Avvia l'ascolto della parola chiave
play_beep(700, 0.3, 0.3)
play_tts("Buongiorno!")
listen_for_keyword()
