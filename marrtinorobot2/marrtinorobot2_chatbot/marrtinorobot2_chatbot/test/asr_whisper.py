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
# file : asr_whisper.py
# descrizione : arc whisper
# 
#! /usr/bin/python3
import sounddevice as sd
import numpy as np
import whisper
import soundfile as sf
import requests
import json

# Carica la configurazione dal file JSON
with open("config.json", "r") as f:
    config = json.load(f)

# Inizializza Whisper esplicitamente per CPU
model_whisper = whisper.load_model(config["whisper_model"], device="cpu")

# Frequenze di campionamento
input_samplerate = config["input_samplerate"]

# Indici dei dispositivi audio
input_device_index = config["input_device_index"]
output_device_index = config["output_device_index"]

# Imposta i dispositivi audio
sd.default.device = (input_device_index, output_device_index)

def send_text_to_asr_server(text, server_url="http://localhost:5002/asr"):
    try:
        response = requests.post(server_url, json={"text": text})
        if response.status_code == 200:
            print(f"Messaggio inviato al server ASR: {text}")
        else:
            print(f"Errore nell'invio al server ASR: {response.status_code}, {response.text}")
    except Exception as e:
        print(f"Errore durante la comunicazione con il server Asr: {e}")

def transcribe_continuously(duration=5):
    print("Inizio trascrizione continua con Whisper...")
    try:
        while True:
            print("Parla ora...")
            audio = sd.rec(int(input_samplerate * duration), samplerate=input_samplerate, channels=1, dtype='int16')
            sd.wait()
            sf.write("audio.wav", audio, input_samplerate)
            result = model_whisper.transcribe("audio.wav", language="it")
            if result["text"].strip():
                print("Hai detto:", result["text"])
                send_text_to_asr_server(f"Hai detto: {result['text']}")
            else:
                print("Non ho rilevato alcuna frase comprensibile.")
    except KeyboardInterrupt:
        print("Trascrizione continua interrotta dall'utente.")

print("ASR whisper v.1.0")
transcribe_continuously()
