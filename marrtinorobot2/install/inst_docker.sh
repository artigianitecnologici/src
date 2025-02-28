#!/bin/bash

# Aggiorna il sistema
echo "Aggiornando il sistema..."
sudo apt update && sudo apt upgrade -y

# Rimuove docker.io se già installato
echo "Rimuovendo docker.io se presente..."
sudo apt remove -y docker.io

# Aggiungi la chiave GPG ufficiale di Docker
echo "Aggiungendo la chiave GPG ufficiale di Docker..."
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Aggiungi il repository ufficiale di Docker
echo "Aggiungendo il repository ufficiale di Docker..."
echo "deb [arch=amd64 signed-by=/usr/share/keyrsudoings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Installa Docker (ultima versione disponibile)
echo "Installando Docker..."
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io

# Verifica la versione di Docker
echo "Verifica della versione di Docker..."
docker --version

# Installa il plugin Docker Compose
echo "Installando Docker Compose come plugin..."
sudo apt install -y docker-compose-plugin

# Verifica la versione di Docker Compose
echo "Verifica della versione di Docker Compose..."
docker compose version

# Aggiungi l'utente al gruppo Docker per evitare di usare sudo
echo "Aggiungendo l'utente al gruppo Docker..."
sudo usermod -aG docker $USER

# Avvia Docker e abilita l'avvio automatico
echo "Avviando Docker e abilitando l'avvio automatico..."
sudo systemctl start docker
sudo systemctl enable docker

echo "Installazione completata. Riavvia il sistema o fai logout per applicare le modifiche al gruppo Docker."
