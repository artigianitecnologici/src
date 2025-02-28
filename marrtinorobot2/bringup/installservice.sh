#!/bin/bash

# Nome del file del servizio
SERVICE_FILE="docker-start.service"

# Percorso di destinazione
DEST_PATH="/etc/systemd/system"

# Verifica se il file esiste
if [[ ! -f $SERVICE_FILE ]]; then
  echo "Errore: Il file $SERVICE_FILE non esiste nella directory corrente."
  exit 1
fi

# Copia il file nella directory di systemd
echo "Copia di $SERVICE_FILE in $DEST_PATH..."
sudo cp "$SERVICE_FILE" "$DEST_PATH"

# Ricarica systemd per riconoscere il nuovo servizio
echo "Ricaricamento di systemd..."
sudo systemctl daemon-reload

# Abilita il servizio all'avvio
echo "Abilitazione del servizio $SERVICE_FILE..."
sudo systemctl enable docker-start.service

# Avvia il servizio immediatamente
echo "Avvio del servizio $SERVICE_FILE..."
sudo systemctl start docker-start.service

# Mostra lo stato del servizio
echo "Stato del servizio $SERVICE_FILE:"
sudo systemctl status docker-start.service
