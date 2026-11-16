#!/bin/sh
echo "Usando l'opzione -a, Docker eliminerà tutte le immagini."
echo "Vuoi procedere con l'eliminazione? (s/n)"
read risposta

if [ "$risposta" = "s" ]; then
    docker container stop $(docker container ls -aq) && docker system prune -af --volumes
    echo "Immagini inutilizzate eliminate."
elif [ "$risposta" = "n" ]; then
    echo "Operazione annullata."
else
    echo "Input non valido. Inserisci 's' per sì o 'n' per no."
fi







