# Analisi progetto

Data analisi: 2026-05-30

## Panoramica

Questo repository raccoglie un workspace ROS 2 per MARRtino con pacchetti robotici, strumenti Docker, interfacce web e componenti AI/voce.

Le aree principali sono:

- `marrtinorobot2/`: pacchetti principali del robot, bringup, navigazione, visione, riconoscimento e voce.
- `docker/humble-host/`: immagine e compose per ambiente ROS 2 Humble con desktop/VNC.
- `DynamixelSDK/`, `dynamixel-workbench/`, `dynamixel-workbench-msgs/`: supporto motori Dynamixel.
- `apriltag_ros/`, `ldlidar_stl_ros2/`, `m-explore-ros2/`, `vizanti/`: pacchetti di percezione, lidar, esplorazione e visualizzazione.
- `eva/`: assistente virtuale Python con Flask, Ollama, Telegram, STT Vosk/faster-whisper e TTS Piper.
- `MarrtinoN8N/`: stack n8n con bridge EVA, workflow Telegram, Qdrant/Postgres e integrazione TTS/ASR.

## Nuove cartelle analizzate

### `eva/`

`eva` contiene un'applicazione Python autonoma per assistente vocale/chatbot:

- entrypoint principali: `eva.py`, `stt_vosk.py`, `twochat.py`, `winguru.py`;
- configurazioni in `config/`;
- handler comandi in `handlers/`;
- template Flask in `templates/`;
- asset statici in `static/`;
- test e prove in `test/`;
- benchmark Ollama in `benchmark/`.

Elementi runtime individuati:

- `myenv/`: ambiente virtuale locale, non va versionato;
- `log/` e `twochat.log`: log generati a runtime;
- modelli Piper `.onnx`: file scaricabili e molto pesanti.

### `MarrtinoN8N/`

`MarrtinoN8N` contiene lo stack di automazione e integrazione:

- `docker-compose.yml`: servizi n8n, Postgres, Qdrant e bridge EVA;
- `eva_bridge.py`: bridge Flask/Telegram/Ollama/TTS;
- `src/asr_chatbot_bridge.py` e `src/asr_tts_node_piper.py`: nodi/bridge ASR e TTS;
- `workflow/`: export workflow n8n;
- `models/`: modelli Piper e cache Whisper.

Elementi runtime individuati:

- `.env`: file locale con segreti, ignorato;
- `log/`, `out/`, `shared/`, `models/whisper_cache/`: output e cache da non versionare.

## Duplicati e file da tenere sotto controllo

Duplicati identici trovati tra `eva` e `MarrtinoN8N`:

- `it_IT-paola-medium.onnx`;
- `it_IT-paola-medium.onnx.json`;
- `it_IT-aurora-medium.onnx`;
- `it_IT-aurora-medium.onnx.json`.

Questi file sono necessari per Piper, ma sono scaricabili e occupano molto spazio. Per evitare di aggiungere ulteriori duplicati con `eva`, i modelli `.onnx` e i rispettivi `.onnx.json` dentro `eva/models/piper/` sono ora ignorati da Git. Le copie locali duplicate in `eva/models/piper/` sono state sostituite con symlink relativi verso `MarrtinoN8N/models/`, mantenendo validi i percorsi di EVA senza occupare due volte lo spazio. I modelli gia presenti in `MarrtinoN8N/models/` sono stati lasciati al loro posto per non rompere il bridge Docker, che monta `./models:/app/models`.

File generati o inutili rimossi:

- cache Python `eva/handlers/__pycache__/`;
- log runtime `eva/log/`;
- log runtime `eva/twochat.log`;
- placeholder vuoti `eva/static/static.txt` e `eva/static/styles/styiles.txt`;
- metadato Windows/WSL `MarrtinoN8N/workflow/EVA e TELEGRAM BOT.json:Zone.Identifier`.
- copie fisiche duplicate dei modelli Piper in `eva/models/piper/`, rimpiazzate da symlink locali.

## Ottimizzazioni applicate

- Aggiornato `.gitignore` globale con pattern per ambienti Python, cache, log, file `.env`, output runtime e cache modelli.
- Ripulito `eva/.gitignore`, eliminando duplicazioni e pattern ripetuti.
- Rinominato `MarrtinoN8N/.env samplae` in `MarrtinoN8N/.env.sample`.
- Sostituiti valori sensibili nel sample `.env` con placeholder espliciti.
- Sostituiti i modelli Piper duplicati in `eva/models/piper/` con symlink verso `MarrtinoN8N/models/`.
- Documentata la duplicazione dei modelli Piper e scelta una strategia prudente: non aggiungere i modelli duplicati di `eva`, mantenendo funzionante lo stack `MarrtinoN8N`.

## Raccomandazioni successive

- Valutare una cartella comune per i modelli, ad esempio `models/piper/`, configurando sia `eva` sia `MarrtinoN8N` per leggerla dallo stesso percorso.
- Evitare di versionare file `.onnx`, cache Whisper, virtualenv e log.
- Aggiungere un file `.env.example` anche per `eva` se le configurazioni locali cresceranno.
- Controllare periodicamente `git status --ignored` prima dei commit, per verificare che non entrino artefatti runtime.
