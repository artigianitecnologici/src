import sounddevice as sd
import json
import os
import numpy as np
import soundfile as sf
import time

# Funzione per ottenere l'elenco dei dispositivi audio disponibili
def list_audio_devices():
    devices = sd.query_devices()
    input_devices = []
    output_devices = []
    for index, device in enumerate(devices):
        if device['max_input_channels'] > 0:
            input_devices.append((index, device['name']))
        if device['max_output_channels'] > 0:
            output_devices.append((index, device['name']))
    return input_devices, output_devices

# Funzione per elencare i modelli Vosk salvati
def list_vosk_models(directory=os.path.expanduser("~/models/vosk")):
    models = []
    if os.path.exists(directory):
        for folder in os.listdir(directory):
            model_path = os.path.join(directory, folder)
            if os.path.isdir(model_path) and os.path.exists(os.path.join(model_path, "am", "final.mdl")):
                models.append((folder, model_path))
    return models

# Funzione per creare il file di configurazione JSON
def create_config_file():
    input_devices, output_devices = list_audio_devices()

    print("\nDispositivi di input disponibili:")
    for index, name in input_devices:
        print(f"[{index}] {name}")
    input_index = int(input("Seleziona l'indice del dispositivo di input: "))

    print("\nDispositivi di output disponibili:")
    for index, name in output_devices:
        print(f"[{index}] {name}")
    output_index = int(input("Seleziona l'indice del dispositivo di output: "))

    print("\nModelli Vosk disponibili:")
    available_vosk_models = list_vosk_models()
    for i, (model, _) in enumerate(available_vosk_models):
        print(f"[{i}] {model}")
    vosk_choice = input("Seleziona il modello Vosk o inserisci un percorso personalizzato: ")

    if vosk_choice.isdigit() and int(vosk_choice) < len(available_vosk_models):
        vosk_model_path = available_vosk_models[int(vosk_choice)][1]
    elif os.path.isdir(vosk_choice):
        vosk_model_path = vosk_choice
    else:
        print("Scelta non valida. Impostazione del modello predefinito.")
        vosk_model_path = os.path.expanduser("~/models/vosk/vosk-model-small-it-0.22")

    language = input("\nLingua TTS predefinita (it/en): ").strip().lower()
    if language not in ['it', 'en']:
        language = 'it'

    msg_start = input("\nMessaggio di benvenuto del robot: ").strip()
    if not msg_start:
        msg_start = "Ciao, sono pronto ad ascoltarti!"

    config = {
        "model_vosk_path": vosk_model_path,
        "vosk_model_url": "https://alphacephei.com/vosk/models/vosk-model-it-0.22.zip",
        "input_device_index": input_index,
        "output_device_index": output_index,
        "input_samplerate": 16000,
        "output_samplerate": 24000,
        "language": language,
        "msg_start": msg_start,
        "work_offline": True
    }

    #os.makedirs("config", exist_ok=True)
    #config_path = os.path.join("config", "config.json")
    config_path = '/home/ubuntu/src/marrtinorobot2/marrtinorobot2_voice/config/config.json'
    with open(config_path, "w") as f:
        json.dump(config, f, indent=4)

    print(f"\n✅ Configurazione salvata in {config_path}")

    # Test audio input
    print("\n🎙️ Inizia a parlare per il test audio (5 secondi)...")
    audio = sd.rec(int(config['input_samplerate'] * 5), samplerate=config['input_samplerate'], channels=1, dtype='int16', device=input_index)
    sd.wait()
    sf.write('test_input.wav', audio, config['input_samplerate'])
    print("✅ Audio di input registrato: test_input.wav")

    # Test audio output
    print("\n🔊 Riproduzione di un beep di test...")
    frequency = 1000
    duration = 1.0
    t = np.linspace(0, duration, int(config['output_samplerate'] * duration), False)
    wave = (0.5 * np.sin(2 * np.pi * frequency * t)).astype(np.float32)
    try:
        with sd.OutputStream(samplerate=config['output_samplerate'], channels=1, dtype='float32', device=output_index) as stream:
            stream.write(wave)
        print("✅ Beep riprodotto correttamente.")
    except Exception as e:
        print(f"❌ Errore durante la riproduzione del beep: {e}")

if __name__ == "__main__":
    create_config_file()
