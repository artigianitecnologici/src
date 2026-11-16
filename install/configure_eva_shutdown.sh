#!/usr/bin/env bash
set -euo pipefail

EVA_USER="${1:-${SUDO_USER:-$USER}}"
if ! id "$EVA_USER" >/dev/null 2>&1; then
  echo "Utente non trovato: $EVA_USER" >&2
  exit 1
fi

SUDOERS_FILE="/etc/sudoers.d/marrtino-eva-shutdown"
TEMP_FILE="$(mktemp)"
trap 'rm -f "$TEMP_FILE"' EXIT

printf '%s ALL=(root) NOPASSWD: /sbin/shutdown -h now, /sbin/shutdown -r now\n' "$EVA_USER" >"$TEMP_FILE"
chmod 0440 "$TEMP_FILE"
visudo -cf "$TEMP_FILE"
install -o root -g root -m 0440 "$TEMP_FILE" "$SUDOERS_FILE"

echo "Permessi shutdown EVA configurati per $EVA_USER in $SUDOERS_FILE"
