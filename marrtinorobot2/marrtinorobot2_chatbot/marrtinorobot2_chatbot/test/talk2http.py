import sounddevice as sd
import queue
import json
from vosk import Model, KaldiRecognizer
import marrtinorobot2.marrtinorobot2_chatbot.marrtinorobot2_chatbot.no_asr_whisper as no_asr_whisper
import numpy as np
import time
# from gtts import gTTS
import io
import soundfile as sf
import os
import requests
import urllib.request
import zipfile

def download_vosk_model(model_url, model_path):
    print(f"Scarico il modello Vosk da {model_url}...")
    zip_path = "vosk_model.zip"
    urllib.request.urlretrieve(model_url, zip_path)
    print("Estrazione modello...")
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall(os.path.dirname(model_path))
    os.remove(zip_path)
    print("Modello Vosk scaricato ed estratto correttamente.")

# Carica la configurazione dal file JSON
with open("config.json", "r") as f:
    config = json.load(f)

model_vosk_path = os.path.abspath(config["model_vosk_path"])
vosk_model_url = config.get("vosk_model_url", "https://alphacephei.com/vosk/models/vosk-model-small-it-0.22.zip")

# verifica l'esistenza effettiva del modello verificando i file interni
expected_file = os.path.join(model_vosk_path, "am", "final.mdl")

if not os.path.exists(expected_file):
    print("Non trovo il Modello Vosk ...")
    print(model_vosk_path)
    download_vosk_model(vosk_model_url, model_vosk_path)
else:
    print("Modello Vosk già presente.")

# Usa sempre il percorso assoluto (non modificarlo più)
model_vosk = Model(model_vosk_path)


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





def send_text_to_chatbot_server(text, server_url="http://localhost:5500/get"):
    try:
        response = requests.post(server_url, json={"text": text})
        if response.status_code == 200:
            print(f"Messaggio inviato al server TTS: {text}")
        else:
            print(f"Errore nell'invio al server TTS: {response.status_code}, {response.text}")
    except Exception as e:
        print(f"Errore durante la comunicazione con il server TTS: {e}")


def send_text_to_asr_server(text, server_url="http://localhost:5002/asr"):
    try:
        response = requests.post(server_url, json={"text": text})
        if response.status_code == 200:
            print(f"Messaggio inviato al server ASR: {text}")
        else:
            print(f"Errore nell'invio al server ASR: {response.status_code}, {response.text}")
    except Exception as e:
        print(f"Errore durante la comunicazione con il server Asr: {e}")

# Funzione di callback per aggiungere i dati audio alla coda
def audio_callback(indata, frames, time, status):
    if status:
        print(status)
    if listening_active:
        q.put(bytes(indata))

# Funzione per riprodurre un suono beep
def play_beep(frequency=1000, duration=0.5, volume=0.1):
    t = np.linspace(0, duration, int(output_samplerate * duration), False)
    wave = (volume * np.sin(2 * np.pi * frequency * t)).astype(np.float32)
    try:
        sd.stop()  # Assicurati che qualsiasi stream audio precedente sia chiuso
        with sd.OutputStream(samplerate=output_samplerate, channels=1, dtype='float32', device=output_device_index) as stream:
            stream.write(wave)
        print("Suono beep riprodotto.")
    except Exception as e:
        print(f"Errore durante la riproduzione del beep: {e}")


# Funzione per ascoltare la parola chiave
def listen_for_keyword():
    global stop_listening
    recognizer = KaldiRecognizer(model_vosk, input_samplerate)
    print(f"In ascolto della parola chiave... (e.g., '{config['keyword']}')")
    try:
        with sd.RawInputStream(samplerate=input_samplerate, dtype='int16',
                               channels=2, callback=audio_callback, device=input_device_index,
                               blocksize=16000):
            while not stop_listening:
                if not q.empty():
                    data = q.get()
                    if recognizer.AcceptWaveform(data):
                        result = json.loads(recognizer.Result())
                        detected_text = result.get('text', '').lower().strip()
                        if detected_text:
                            print("Rilevato:", detected_text)
                            send_text_to_asr_server(f"Rilevato: {detected_text}")
                            #send_text_to_chatbot_server(f"Rilevato: {detected_text}")
                            if config['keyword'] in detected_text:
                                play_beep()
                                transcribe_with_whisper()
                            elif "stop" in detected_text:
                                send_text_to_asr_server("Dispositivo in arresto. Arrivederci!")
                                stop_listening = True
                                break
                else:
                    time.sleep(0.1)
    except Exception as e:
        print(f"Errore nell'apertura dello stream di input: {e}")

# Funzione per registrare e trascrivere con Whisper
def transcribe_with_whisper(duration=5):
    print("Inizia a parlare...")
    try:
        listening_active = False  # Disabilita temporaneamente il microfono
        sd.stop()  # Assicurati che lo stream precedente sia chiuso
        
        audio = sd.rec(int(input_samplerate * duration), samplerate=input_samplerate, channels=1, dtype='int16', device=input_device_index)
        sd.wait()

        # Salva l'audio in un file WAV
        sf.write("audio.wav", audio, input_samplerate)

        # Trascrivi l'audio con Whisper
        result = model_whisper.transcribe("audio.wav", language="it")
        print("Hai detto:", result["text"])
        send_text_to_asr_server(f"Hai detto: {result['text']}")
    except Exception as e:
        print(f"Errore durante la registrazione o trascrizione: {e}")
    finally:
        listening_active = True  # Riattiva il microfono

# Avvia l'ascolto della parola chiave
play_beep(700, 0.3, 0.3)
send_text_to_asr_server("Buongiorno!")
listen_for_keyword()
