#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export REPOSITORY_CACHEBUST="$(date +%s)"

echo "Ricostruisco marrtinorobot2:mate-vnc riscaricando il repository..."
docker compose build --no-cache humble

echo "Riavvio il container humble..."
docker compose up -d --force-recreate humble

echo "Aggiornamento completato."
