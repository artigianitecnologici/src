#!/bin/bash

# Aggiorna il sistema
echo "Aggiornando il sistema..."
sudo apt update && sudo apt upgrade -y

sudo apt install -y curl ca-certificates


# Rimuove eventuali versioni preinstallate di Docker
echo "Rimuovendo vecchie installazioni di Docker..."
sudo apt remove -y docker docker-engine docker.io containerd runc

# Aggiungi la chiave GPG ufficiale di Docker
echo "Aggiungendo la chiave GPG ufficiale di Docker..."
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Configura il repository Docker
echo "Configurando il repository ufficiale di Docker..."
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Aggiorna i pacchetti
echo "Aggiornando i pacchetti..."
sudo apt update

# Imposta permessi appropriati sulla chiave
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Installa Docker e i componenti necessari
echo "Installando Docker e i suoi componenti..."
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Verifica l'installazione di Docker
echo "Verificando l'installazione di Docker..."
docker --version || { echo "Errore: Docker non è installato correttamente."; exit 1; }

# Verifica l'installazione di Docker Compose
echo "Verificando l'installazione di Docker Compose..."
docker compose version || { echo "Errore: Docker Compose non è installato correttamente."; exit 1; }

# Aggiungi l'utente al gruppo Docker per evitare di usare sudo
echo "Aggiungendo l'utente al gruppo Docker..."
sudo usermod -aG docker $USER

# Avvia Docker e abilita l'avvio automatico
echo "Avviando Docker e abilitando l'avvio automatico..."
sudo systemctl enable docker --now

echo "Installazione completata. Riavvia il sistema o fai logout per applicare le modifiche al gruppo Docker."
