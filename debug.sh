#!/bin/bash

# Controlla se è stato passato un parametro
if [ -z "$1" ]; then
    # Nessun parametro passato: mostra le sessioni attive
    echo "Sessioni tmux attive:"
    tmux ls
else
    # Parametro passato: tenta di collegarsi alla sessione indicata
    echo "Collegamento alla sessione tmux: $1"
    tmux attach-session -t "$1"
fi