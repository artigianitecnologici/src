# Manuale dei Nodi di MarrtinoRobot2

## Introduzione
Questo documento fornisce una panoramica dei nodi ROS 2 utilizzati nel progetto MarrtinoRobot2. I nodi descritti sono progettati per gestire funzioni robotiche come interazione sociale, controllo dei movimenti e gestione delle emozioni.

---

## Elenco dei Nodi

### **1. Nodo: interactive_node**

#### **Descrizione**
`interactive_node` è un nodo che consente al robot di interagire vocalmente e gestualmente con gli utenti. Utilizza topic ROS 2 per ricevere comandi e restituire risposte vocali o emozionali.

#### **Topic Pubblicati**
- **`/social/gesture`**
  - **Tipo di messaggio:** `std_msgs/String`
  - **Descrizione:** Pubblica comandi per gesti del robot, come alzare le braccia o salutare.

- **`/social/emotion`**
  - **Tipo di messaggio:** `std_msgs/String`
  - **Descrizione:** Pubblica emozioni come "happy", "neutral" o "speak".

- **`/speech/to_speak`**
  - **Tipo di messaggio:** `std_msgs/String`
  - **Descrizione:** Pubblica messaggi vocali da riprodurre tramite il sintetizzatore del robot.

#### **Topic Sottoscritti**
- **`/social/asr`**
  - **Tipo di messaggio:** `std_msgs/String`
  - **Descrizione:** Riceve comandi vocali riconosciuti tramite ASR (Automatic Speech Recognition).

- **`/speech/status`**
  - **Tipo di messaggio:** `std_msgs/String`
  - **Descrizione:** Monitora lo stato del parlato (START/STOP) per sincronizzare le emozioni.

---

### **2. Nodo: navigation_node**

#### **Descrizione**
`navigation_node` gestisce il controllo del movimento del robot, inclusa la navigazione verso obiettivi specifici e la gestione degli ostacoli.

#### **Topic Pubblicati**
- **`/cmd_vel`**
  - **Tipo di messaggio:** `geometry_msgs/Twist`
  - **Descrizione:** Invia comandi di velocità lineare e angolare per controllare i motori.

- **`/navigation/status`**
  - **Tipo di messaggio:** `std_msgs/String`
  - **Descrizione:** Indica lo stato della navigazione (es. "Navigating", "Goal Reached").

#### **Topic Sottoscritti**
- **`/move_base_simple/goal`**
  - **Tipo di messaggio:** `geometry_msgs/PoseStamped`
  - **Descrizione:** Riceve obiettivi di navigazione definiti dall'utente.

- **`/sensor/obstacle`**
  - **Tipo di messaggio:** `sensor_msgs/LaserScan`
  - **Descrizione:** Monitora la presenza di ostacoli per evitarli durante il movimento.

---

### **3. Nodo: vision_node**

#### **Descrizione**
`vision_node` processa i dati visivi per funzioni come il riconoscimento di oggetti o volti.

#### **Topic Pubblicati**
- **`/vision/objects`**
  - **Tipo di messaggio:** `std_msgs/String`
  - **Descrizione:** Pubblica i nomi degli oggetti riconosciuti nella scena.

- **`/vision/faces`**
  - **Tipo di messaggio:** `std_msgs/String`
  - **Descrizione:** Pubblica informazioni sui volti riconosciuti, inclusi ID e posizioni.

#### **Topic Sottoscritti**
- **`/camera/image_raw`**
  - **Tipo di messaggio:** `sensor_msgs/Image`
  - **Descrizione:** Riceve i dati grezzi dalla fotocamera del robot.

---

### **4. Nodo: emotion_node**

#### **Descrizione**
`emotion_node` gestisce la rappresentazione delle emozioni del robot basata sugli input ricevuti da altri nodi.

#### **Topic Pubblicati**
- **`/display/emotion`**
  - **Tipo di messaggio:** `std_msgs/String`
  - **Descrizione:** Mostra un'emozione specifica sul display del robot.

#### **Topic Sottoscritti**
- **`/social/emotion`**
  - **Tipo di messaggio:** `std_msgs/String`
  - **Descrizione:** Riceve comandi per aggiornare lo stato emotivo del robot.

---

### **5. Nodo: speech_node**

#### **Descrizione**
`speech_node` utilizza un motore TTS (Text-to-Speech) per convertire messaggi testuali in audio.

#### **Topic Pubblicati**
- **`/speech/status`**
  - **Tipo di messaggio:** `std_msgs/String`
  - **Descrizione:** Indica lo stato del parlato (es. "START", "STOP").

#### **Topic Sottoscritti**
- **`/speech/to_speak`**
  - **Tipo di messaggio:** `std_msgs/String`
  - **Descrizione:** Riceve messaggi testuali da convertire in audio.

---

## Configurazione e Avvio dei Nodi

1. **Configurazione:** Assicurati che tutti i file di configurazione siano corretti, inclusi parametri per la rete, sensori e motori.
2. **Avvio:** Usa i seguenti comandi per avviare i nodi:
   - **Interactive Node:**
     ```bash
     ros2 run marrtinorobot2 interactive_node
     ```
   - **Navigation Node:**
     ```bash
     ros2 run marrtinorobot2 navigation_node
     ```
   - **Vision Node:**
     ```bash
     ros2 run marrtinorobot2 vision_node
     ```

---

## Possibili Miglioramenti
- **Integrazione di nuove funzionalità:** Ad esempio, il riconoscimento vocale migliorato o la navigazione avanzata.
- **Ottimizzazione delle risorse:** Ridurre il consumo di CPU e memoria ottimizzando i nodi esistenti.
- **Aggiunta di nuovi nodi:** Per funzionalità come la gestione di bracci robotici o l'interazione remota.

---

## Conclusione
Questo manuale offre una panoramica dei nodi principali di MarrtinoRobot2 e delle loro funzionalità. Per ulteriori dettagli, consulta la documentazione tecnica o contatta il team di sviluppo.

