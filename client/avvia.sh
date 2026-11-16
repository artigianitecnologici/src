#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ ! -x .venv/bin/python ]]; then
  echo "Ambiente Python mancante. Esegui:" >&2
  echo "  python3 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt" >&2
  echo "  sudo apt install -y python3-tk"
  exit 1
fi
exec .venv/bin/python sonia_client.py "$@"
