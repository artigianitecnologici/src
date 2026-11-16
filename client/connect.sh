#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec bash ./avvia.sh --url "${SONIA_URL:-http://192.168.1.199:8096}" "$@"
