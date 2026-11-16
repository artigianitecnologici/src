# marrtinorobot2_recognition

Nodo ROS 2 per riconoscimento facciale basato su **InsightFace** (ArcFace + ONNX Runtime). 
Sottoscrive `sensor_msgs/Image` su **`/camera/image_raw`** e pubblica `std_msgs/String` in JSON su **`/recognized_face`**.

> ⚠️ **Compatibilità OS/ROS**  
ROS 2 **Humble** è ufficialmente per **Ubuntu 22.04** (Python 3.10).  
Su **Ubuntu 24.04** la scelta naturale è **ROS 2 Jazzy** (Python 3.12). Il pacchetto funziona in entrambi i casi,
ma le librerie Python cambiano versione. Le istruzioni sotto coprono entrambi gli scenari.

---

## Dipendenze Python (pip)

Installare nel **Python usato da colcon** (consigliato creare un venv nel workspace):

- `insightface==0.7.3`
- `onnxruntime==1.18.0`  *(oppure `onnxruntime-gpu` se usi CUDA; su Apple Silicon: `onnxruntime-silicon`)*
- `numpy==1.26.4`
- `Pillow==10.4.0`

File pronto: [`requirements-ros.txt`](requirements-ros.txt).

```bash
# venv per il workspace
python3 -m venv ~/ros_ws/.venv
source ~/ros_ws/.venv/bin/activate
python -m pip install -U pip setuptools wheel
pip install -r src/marrtinorobot2_recognition/requirements-ros.txt

# facoltativo: GPU
# pip install onnxruntime-gpu==1.18.0
```

**Nota versioni:**
- Con **ROS 2 Humble (22.04 / Python 3.10)** puoi lasciare le versioni qui sopra.
- Con **ROS 2 Jazzy (24.04 / Python 3.12)** queste versioni sono compatibili; evita NumPy 2.x con InsightFace per ora.

---

## Struttura

```
marrtinorobot2_recognition/
  marrtinorobot2_recognition/
    __init__.py
    face_recognizer_node.py
  launch/
    face_recognizer.launch.py
  params/
    face_recognizer.yaml
  resource/
    marrtinorobot2_recognition
  package.xml
  setup.py
  setup.cfg
  requirements-ros.txt
  data/               # (opzionale) embeddings.npy + names.json
```

- **DB embedding**: questo nodo può condividere la stessa cartella `data/` della tua app Flask: 
  - `data/embeddings.npy`
  - `data/names.json`  
  Se assenti, il nodo pubblicherà comunque eventi, ma riconoscerà tutti come `"Unknown"`.

---

## Build

```bash
# workspace
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
# copia qui la cartella marrtinorobot2_recognition

# attiva venv e installa dipendenze Python (vedi sopra)
source ~/ros_ws/.venv/bin/activate
export COLCON_PYTHON_EXECUTABLE=$(which python)

cd ~/ros_ws
colcon build
source install/setup.bash
```

---

## Esecuzione

```bash
ros2 launch marrtinorobot2_recognition face_recognizer.launch.py
# oppure:
# ros2 run marrtinorobot2_recognition face_recognizer --ros-args -p db_dir:=/path/to/data
```

Controllo output:
```bash
ros2 topic echo /recognized_face
```

Addestramento da topic:
```bash
ros2 topic pub -1 /face_recognizer/command std_msgs/String \
  "{data: '{\"action\":\"enroll\",\"name\":\"Fabio\",\"samples\":5}'}"
```

Stato del nodo:
```bash
ros2 topic echo /face_recognizer/status
```

Esempio messaggio pubblicato (JSON in `std_msgs/String.data`):
```json
{"name":"Alice","similarity":0.73,"bbox":[120.4,80.2,220.1,210.9],"stamp":{"sec":123,"nanosec":456789}}
```

---

## Parametri principali

- `db_dir` (string) — cartella con `embeddings.npy` e `names.json` (default: `data`)
- `threshold` (float) — soglia di similarità coseno per decidere il match (default: `0.45`)
- `providers` (list[str]) — ONNX Runtime providers, es. `['CPUExecutionProvider']` o `['CUDAExecutionProvider']`
- `det_size_w`, `det_size_h` (int) — dimensione del riquadro di detection (default: `640x640`)
- `topic_image` (string) — topic di input immagine (default: `/camera/image_raw`)
- `topic_output` (string) — topic di output JSON (default: `/recognized_face`)
- `topic_command` (string) — topic comandi JSON per addestramento/test (default: `/face_recognizer/command`)
- `topic_status` (string) — topic status JSON del nodo (default: `/face_recognizer/status`)
- `enroll_samples` (int) — campioni acquisiti per memorizzare un volto (default: `5`)
- `enroll_timeout_sec` (float) — timeout acquisizione volto (default: `20.0`)
- `process_every_n` (int) — elabora 1 frame ogni N (default: `1`)

---

## Formato messaggio output

Il nodo pubblica su **`/recognized_face`** un `std_msgs/String` con payload JSON:
```json
{
  "name": "Unknown" | "<nome>",
  "similarity": <float 0..1>,
  "bbox": [x1, y1, x2, y2] | null,
  "stamp": {"sec": <sec>, "nanosec": <nsec>}
}
```

---

## Note sull’immagine di input

Al momento sono supportati gli encoding `rgb8` e `bgr8`. Se la camera pubblica altro (`mono8`, YUV, ecc.)
estendi la funzione `_rosimg_to_bgr8` in `face_recognizer_node.py` per includere la conversione desiderata.

---

## GPU (facoltativo)

Per usare la GPU CUDA:
1. Installa `onnxruntime-gpu` compatibile con la tua versione CUDA.
2. Avvia il nodo con `providers:=['CUDAExecutionProvider']` (vedi launch/params).

---

## Licenza

MIT

---

## Ricerca e avvicinamento a persone

`person_follower` usa YuNet a bassa risoluzione per rilevare il volto di una
persona. Camera e laser vengono sottoscritti soltanto durante un'azione, quindi
il nodo inattivo non elabora immagini.

```bash
ros2 launch marrtinorobot2_recognition person_follower.launch.py

# Cerca ruotando lentamente
ros2 topic pub --once /person_follower/command std_msgs/msg/String \
  "{data: '{\"action\":\"search\"}'}"

# Avvicinati all'ultimo bersaglio, oppure cercane uno
ros2 topic pub --once /person_follower/command std_msgs/msg/String \
  "{data: '{\"action\":\"approach\"}'}"

# Arresto immediato
ros2 topic pub --once /person_follower/command std_msgs/msg/String \
  "{data: '{\"action\":\"stop\"}'}"
```

Il movimento lineare è consentito solo con una scansione laser recente. Il
robot si arresta a `stop_distance`, se perde il bersaglio o allo scadere del
timeout. I parametri sono in `params/person_follower.yaml`.
