#!/bin/sh
echo "Usando l'opzione -a, Docker eliminerà tutte le immagini inutilizzate, incluse quelle con tag, a condizione che non siano collegate a container attivi."
echo "Vuoi procedere con l'eliminazione? (s/n)"
read risposta

if [ "$risposta" = "s" ]; then
    docker image prune -a
    echo "Immagini inutilizzate eliminate."
elif [ "$risposta" = "n" ]; then
    echo "Operazione annullata."
else
    echo "Input non valido. Inserisci 's' per sì o 'n' per no."
fi
