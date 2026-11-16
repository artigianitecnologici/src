#!/bin/bash

# Aggiorna il sistema
echo "Aggiornamento del sistema..."
sudo apt update && sudo apt upgrade -y

# Installa i prerequisiti
echo "Installazione dei prerequisiti..."
sudo apt install -y curl python3-pip

# Installa Docker
echo "Installazione di Docker..."
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Aggiungi l'utente corrente al gruppo Docker
echo "Aggiunta dell'utente al gruppo Docker..."
sudo usermod -aG docker $USER

# Abilita e avvia Docker
echo "Abilitazione e avvio del servizio Docker..."
sudo systemctl enable docker
sudo systemctl start docker

# Installa Docker Compose
echo "Installazione di Docker Compose..."
sudo pip3 install docker-compose

# Verifica le versioni installate
echo "Verifica delle versioni di Docker e Docker Compose..."
docker --version
docker-compose --version

# Messaggio finale
echo "Installazione completata! Ricorda di uscire e rientrare nell'account utente per utilizzare Docker senza sudo."
