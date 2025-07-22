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
# file : asr:vosk.py
#  descrizione : publish /asr  stt
#
#! /usr/bin/python3
import sounddevice as sd
import queue
import json
from vosk import Model, KaldiRecognizer
import requests
import os
import urllib.request
import zipfile

# Funzione per scaricare il modello Vosk
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

# Verifica l'esistenza effettiva del modello Vosk
expected_file = os.path.join(model_vosk_path, "am", "final.mdl")
if not os.path.exists(expected_file):
    print("Non trovo il Modello Vosk ...")
    print(model_vosk_path)
    download_vosk_model(vosk_model_url, model_vosk_path)
else:
    print("Modello Vosk già presente.")

# Usa sempre il percorso assoluto (non modificarlo più)
model_vosk = Model(model_vosk_path)

# Frequenze di campionamento
input_samplerate = config["input_samplerate"]

# Indici dei dispositivi audio
input_device_index = config["input_device_index"]
output_device_index = config["output_device_index"]

# Imposta i dispositivi audio
sd.default.device = (input_device_index, output_device_index)

# Coda per i dati audio
q = queue.Queue()

# Callback per inserire dati audio nella coda
def audio_callback(indata, frames, time, status):
    if status:
        print(status)
    q.put(bytes(indata))


# Invia testo al server ASR
def send_speech_to_asr_server(text, server_url="http://localhost:5002/speech"):
    try:
        response = requests.post(server_url, json={"text": text})
        if response.status_code == 200:
            print(f"Messaggio inviato al speech ASR: {text}")
        else:
            print(f"Errore nell'invio al server ASR: {response.status_code}, {response.text}")
    except Exception as e:
        print(f"Errore durante la comunicazione con il server ASR: {e}")

# Invia testo al server ASR
def send_text_to_asr_server(text, server_url="http://localhost:5002/asr"):
    try:
        response = requests.post(server_url, json={"text": text})
        if response.status_code == 200:
            print(f"Messaggio inviato al server ASR: {text}")
        else:
            print(f"Errore nell'invio al server ASR: {response.status_code}, {response.text}")
    except Exception as e:
        print(f"Errore durante la comunicazione con il server ASR: {e}")

        # Invia testo al server ASR
def send_text_to_robot_command(text, server_url="http://localhost:5002/command"):
    try:
        response = requests.post(server_url, json={"text": text})
        if response.status_code == 200:
            print(f"Messaggio inviato al server command: {text}")
        else:
            print(f"Errore nell'invio al server command: {response.status_code}, {response.text}")
    except Exception as e:
        print(f"Errore durante la comunicazione con il server command: {e}")

def send_text_to_chatbot(text, server_url="http://localhost:5500/get"):
    try:
        params = {"msg": text}
        response = requests.get(server_url, params=params)
        if response.status_code == 200:
            data = response.json()
            testmsg = data.get('response')

            print(f"🤖 Risposta del chatbot: {data.get('response')}")
            #send_speech_to_asr_server(testmsg)
            send_text_to_robot_command(testmsg)

            
        else:
            print(f"Errore nel contattare il chatbot: {response.status_code}, {response.text}")
    except Exception as e:
        print(f"Errore durante la comunicazione con il chatbot: {e}")


# Trascrizione continua con Vosk
def transcribe_continuously():
    recognizer = KaldiRecognizer(model_vosk, input_samplerate)
    print("Inizio trascrizione continua con Vosk...")

    try:
        with sd.RawInputStream(samplerate=input_samplerate, dtype='int16',
                               channels=1, callback=audio_callback):
            while True:
                data = q.get()
                if recognizer.AcceptWaveform(data):
                    result = json.loads(recognizer.Result())
                    detected_text = result.get('text', '').strip()
                    if detected_text:
                        print("Hai detto:", detected_text)
                        send_text_to_asr_server(detected_text)
                        send_text_to_robot_command(detected_text)

                        #send_text_to_chatbot(detected_text)
                        

    except KeyboardInterrupt:
        print("Trascrizione continua interrotta dall'utente.")

print("ASR Vosk v.1.0")
transcribe_continuously()