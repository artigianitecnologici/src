# Manuale dei Comandi Docker

## Introduzione
Docker è una piattaforma che consente agli sviluppatori di creare, distribuire e eseguire applicazioni all'interno di container. Questo manuale fornisce una guida pratica ai comandi Docker più utilizzati.

---

## **Installazione di Docker**

1. **Ubuntu**:
   ```bash
   sudo apt update
   sudo apt install docker.io
   ```
2. **Verifica installazione:**
   ```bash
   docker --version
   ```
3. **Avviare il servizio Docker:**
   ```bash
   sudo systemctl start docker
   sudo systemctl enable docker
   ```

---

## **Comandi di Base**

### 1. Verifica dello stato di Docker
- **Comando:**
  ```bash
  docker info
  ```
- **Descrizione:** Mostra informazioni sul demone Docker, tra cui numero di container e immagini.

### 2. Elenco dei container
- **Container attivi:**
  ```bash
  docker ps
  ```
- **Tutti i container (anche quelli inattivi):**
  ```bash
  docker ps -a
  ```

### 3. Avvio di un container
- **Esegui un container interattivo:**
  ```bash
  docker run -it ubuntu
  ```
- **Avvia un container esistente:**
  ```bash
  docker start <container_id>
  ```

### 4. Arresto di un container
- **Stop di un container specifico:**
  ```bash
  docker stop <container_id>
  ```

### 5. Rimozione di un container
- **Rimuovi un container specifico:**
  ```bash
  docker rm <container_id>
  ```
- **Rimuovi tutti i container inattivi:**
  ```bash
  docker container prune
  ```

---

## **Gestione delle Immagini**

### 1. Elenco delle immagini locali
- **Comando:**
  ```bash
  docker images
  ```

### 2. Scaricare un'immagine dal Docker Hub
- **Comando:**
  ```bash
  docker pull <image_name>
  ```
- **Esempio:**
  ```bash
  docker pull nginx
  ```

### 3. Creazione di un'immagine
- **Comando:**
  ```bash
  docker build -t <nome_immagine> .
  ```
- **Esempio:**
  ```bash
  docker build -t mia_immagine:1.0 .
  ```

### 4. Rimozione di un'immagine
- **Comando:**
  ```bash
  docker rmi <image_id>
  ```

### 5. Pulizia delle immagini inutilizzate
- **Comando:**
  ```bash
  docker image prune
  ```

---

## **Gestione dei Volumi**

### 1. Elenco dei volumi
- **Comando:**
  ```bash
  docker volume ls
  ```

### 2. Creazione di un volume
- **Comando:**
  ```bash
  docker volume create <nome_volume>
  ```

### 3. Rimozione di un volume
- **Comando:**
  ```bash
  docker volume rm <nome_volume>
  ```

### 4. Pulizia dei volumi inutilizzati
- **Comando:**
  ```bash
  docker volume prune
  ```

---

## **Reti Docker**

### 1. Elenco delle reti
- **Comando:**
  ```bash
  docker network ls
  ```

### 2. Creazione di una rete
- **Comando:**
  ```bash
  docker network create <nome_rete>
  ```

### 3. Collegamento di un container a una rete
- **Comando:**
  ```bash
  docker network connect <nome_rete> <container_id>
  ```

### 4. Rimozione di una rete
- **Comando:**
  ```bash
  docker network rm <nome_rete>
  ```

---

## **Utilità Avanzate**

### 1. Esecuzione di comandi all'interno di un container
- **Comando:**
  ```bash
  docker exec -it <container_id> <comando>
  ```
- **Esempio:**
  ```bash
  docker exec -it <container_id> bash
  ```

### 2. Log di un container
- **Comando:**
  ```bash
  docker logs <container_id>
  ```

### 3. Controllo delle risorse di un container
- **Comando:**
  ```bash
  docker stats <container_id>
  ```

---

## **Comandi Utili per il Debug**

### 1. Ispezione di un container
- **Comando:**
  ```bash
  docker inspect <container_id>
  ```

### 2. Verifica dello spazio utilizzato
- **Comando:**
  ```bash
  docker system df
  ```

### 3. Pulizia generale
- **Comando:**
  ```bash
  docker system prune
  ```
- **Descrizione:** Rimuove container, immagini, volumi e reti non utilizzati.

---

## **Esempi Pratici**

### Creazione e Avvio di un Web Server con Nginx
1. Scaricare l'immagine Nginx:
   ```bash
   docker pull nginx
   ```
2. Avviare un container Nginx:
   ```bash
   docker run -d -p 8080:80 nginx
   ```
3. Verificare l'accesso a `http://localhost:8080`.

### Creazione di un Ambiente di Sviluppo Python
1. Creare un file `Dockerfile`:
   ```dockerfile
   FROM python:3.9
   WORKDIR /app
   COPY . .
   RUN pip install -r requirements.txt
   CMD ["python", "app.py"]
   ```
2. Costruire l'immagine:
   ```bash
   docker build -t python-app .
   ```
3. Avviare il container:
   ```bash
   docker run -d -p 5000:5000 python-app
   ```

---

## Conclusione
Questo manuale offre una panoramica completa dei comandi Docker più utilizzati. Docker semplifica la creazione, la gestione e la distribuzione di applicazioni, rendendolo uno strumento essenziale per gli sviluppatori. Per ulteriori dettagli, visita la [documentazione ufficiale di Docker](https://docs.docker.com).

