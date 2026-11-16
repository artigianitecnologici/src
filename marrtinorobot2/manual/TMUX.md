# Manuale per l'Utilizzo di Tmux

## Introduzione
Tmux (Terminal Multiplexer) è uno strumento potente che consente di gestire più sessioni di terminale in una singola finestra. Con Tmux, puoi dividere il terminale in più riquadri, creare sessioni persistenti e lavorare in modo più efficiente.

---

## **Installazione**

### **Ubuntu/Debian**
```bash
sudo apt update
sudo apt install tmux
```

### **MacOS**
```bash
brew install tmux
```

### **Verifica Installazione**
```bash
tmux -V
```

---

## **Comandi di Base**

### **1. Avvio di Tmux**
- **Comando:**
  ```bash
  tmux
  ```
- **Descrizione:** Avvia una nuova sessione di Tmux.

### **2. Creazione di una Sessione**
- **Comando:**
  ```bash
  tmux new -s <nome_sessione>
  ```
- **Esempio:**
  ```bash
  tmux new -s lavoro
  ```
- **Descrizione:** Crea una nuova sessione di Tmux con un nome specifico.

### **3. Elenco delle Sessioni**
- **Comando:**
  ```bash
  tmux ls
  ```
- **Descrizione:** Mostra tutte le sessioni attive.

### **4. Collegamento a una Sessione Esistente**
- **Comando:**
  ```bash
  tmux attach -t <nome_sessione>
  ```
- **Esempio:**
  ```bash
  tmux attach -t lavoro
  ```
- **Descrizione:** Collega la sessione Tmux specificata al terminale attuale.

### **5. Terminare una Sessione**
- **Comando:**
  ```bash
  exit
  ```
- **Descrizione:** Termina la sessione corrente.

---

## **Navigazione nei Riquadri**

### **Dividere il Terminale**
- **Dividere in orizzontale:**
  ```bash
  Ctrl-b "
  ```
- **Dividere in verticale:**
  ```bash
  Ctrl-b %
  ```

### **Spostarsi tra i Riquadri**
- **Comando:**
  ```bash
  Ctrl-b freccia
  ```
- **Descrizione:** Usa le frecce direzionali per spostarti tra i riquadri.

### **Chiudere un Riquadro**
- **Comando:**
  ```bash
  Ctrl-d
  ```
- **Descrizione:** Chiude il riquadro corrente.

---

## **Gestione delle Finestre**

### **Creare una Nuova Finestra**
- **Comando:**
  ```bash
  Ctrl-b c
  ```
- **Descrizione:** Crea una nuova finestra.

### **Passare tra le Finestre**
- **Comando:**
  ```bash
  Ctrl-b n
  ```
- **Descrizione:** Passa alla finestra successiva.

- **Comando:**
  ```bash
  Ctrl-b p
  ```
- **Descrizione:** Passa alla finestra precedente.

- **Comando:**
  ```bash
  Ctrl-b <numero>
  ```
- **Descrizione:** Passa a una finestra specifica.

### **Elenco delle Finestre**
- **Comando:**
  ```bash
  Ctrl-b w
  ```
- **Descrizione:** Mostra un elenco di tutte le finestre.

### **Chiudere una Finestra**
- **Comando:**
  ```bash
  Ctrl-b &
  ```
- **Descrizione:** Chiude la finestra corrente.

---

## **Sessioni Persistenti**
Una delle caratteristiche più potenti di Tmux è la capacità di mantenere le sessioni attive anche dopo la disconnessione.

### **1. Avvio di una Sessione Persistente**
- **Comando:**
  ```bash
  tmux new -s <nome_sessione>
  ```
- **Descrizione:** Avvia una sessione che rimane attiva anche dopo la chiusura del terminale.

### **2. Riconnessione a una Sessione**
- **Comando:**
  ```bash
  tmux attach -t <nome_sessione>
  ```

### **3. Disconnessione senza Terminare la Sessione**
- **Comando:**
  ```bash
  Ctrl-b d
  ```
- **Descrizione:** Disconnette la sessione, lasciandola attiva in background.

---

## **Personalizzazione di Tmux**
Puoi personalizzare Tmux modificando il file di configurazione `~/.tmux.conf`.

### **Esempio di Configurazione**
```bash
# Cambia il tasto predefinito da Ctrl-b a Ctrl-a
set-option -g prefix C-a
unbind C-b
bind C-a send-prefix

# Abilita il mouse
set -g mouse on

# Mostra lo stato delle finestre
setw -g automatic-rename on
```

### **Applicare le Modifiche**
- **Comando:**
  ```bash
  tmux source-file ~/.tmux.conf
  ```

---

## **Comandi Utili per il Debug**

### **Visualizzare Log di Tmux**
- **Comando:**
  ```bash
  tmux show-messages
  ```

### **Visualizzare Configurazione Corrente**
- **Comando:**
  ```bash
  tmux show-options -g
  ```

---

## Conclusione
Tmux è uno strumento indispensabile per chi lavora spesso con il terminale. Permette di ottimizzare il flusso di lavoro e mantenere sessioni persistenti. Con la pratica, Tmux diventa un compagno insostituibile per lo sviluppo e l'amministrazione di sistemi.

