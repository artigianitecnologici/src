# Documentazione dei Nodi ROS 2 Utilizzati

## Introduzione
Questo documento descrive i nodi ROS 2 implementati per il progetto basato su ROS 2 Humble. Il sistema si compone di un nodo interattivo principale chiamato `InteractiveNode`, progettato per gestire la comunicazione con vari topic e integrare un bot che risponde ai comandi vocali e gestuali.

---

## Nodo: InteractiveNode

### Descrizione
`InteractiveNode` è un nodo ROS 2 progettato per interagire con un sistema robotico tramite comandi vocali e gestuali. Fornisce funzionalità di pubblicazione su topic relativi a emozioni, gesti e parlato e ascolta i topic per riconoscimento vocale e stato del parlato.

### Topic gestiti

#### Publisher
- **`/social/gesture`**  
  Pubblica comandi gestuali per il robot, come alzare le braccia o salutare.  
  **Tipo di messaggio:** `std_msgs/String`

- **`/social/emotion`**  
  Pubblica emozioni visive per il robot, ad esempio "speak" o "normal".  
  **Tipo di messaggio:** `std_msgs/String`

- **`/speech/to_speak`**  
  Pubblica messaggi di parlato generati dal sistema per comunicare con l'utente.  
  **Tipo di messaggio:** `std_msgs/String`

#### Subscriber
- **`/social/asr`**  
  Ascolta comandi vocali riconosciuti (ASR - Automatic Speech Recognition) per processarli e attivare azioni corrispondenti.  
  **Tipo di messaggio:** `std_msgs/String`

- **`/speech/status`**  
  Ascolta lo stato del modulo di sintesi vocale (START/STOP) per sincronizzare le emozioni del robot durante il parlato.  
  **Tipo di messaggio:** `std_msgs/String`

---

### Funzionalità Principali

#### 1. Gestione del Parlato (`say`)
Genera messaggi vocali utilizzando il topic `/speech/to_speak`.

- **Parametri:**  
  - `msg` (string): Il messaggio da trasmettere.  
  - `language` (string): La lingua del messaggio.  

- **Esempio:**  
  ```python
  self.say("Ciao, come posso aiutarti?", "it")
