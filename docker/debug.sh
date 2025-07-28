#!/bin/bash

# Ottieni l'elenco dei container Docker attivi
containers=($(docker ps --format '{{.Names}}'))

# Verifica se ci sono container attivi
if [ ${#containers[@]} -eq 0 ]; then
    echo "⚠️  Nessun container Docker attivo trovato."
    exit 1
fi

# Mostra il menu di selezione
echo "Scegli un container:"
for i in "${!containers[@]}"; do
    echo "$((i+1))) ${containers[$i]}"
done

# Richiedi all'utente di effettuare una scelta
read -p "#? " scelta

# Verifica se l'input è un numero valido
if ! [[ "$scelta" =~ ^[0-9]+$ ]] || [ "$scelta" -lt 1 ] || [ "$scelta" -gt "${#containers[@]}" ]; then
    echo "❌ Scelta non valida."
    exit 1
fi

# Ottieni il nome del container selezionato
container="${containers[$((scelta-1))]}"
echo "🔗 Collegamento al container '$container' via tmux..."

# Verifica se ci sono sessioni tmux attive nel container
if docker exec -it "$container" tmux ls &>/dev/null; then
    docker exec -it "$container" tmux attach
else
    echo "⚠️  Nessuna sessione tmux attiva trovata nel container '$container'."
    echo "Puoi avviarne una con: docker exec -it $container tmux new -s nome_sessione"
fi
