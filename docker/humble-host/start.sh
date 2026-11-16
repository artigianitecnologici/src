
#!/bin/bash

# Valori attesi per le variabili
EXPECTED_VALUES=(
    "MARRTINOROBOT2_WEBI=$HOME/src/marrtinorobot2/marrtinorobot2_webinterface"
    "MARRTINOROBOT2_HOME=$HOME/marrtinorobot2"
)

# Flag per errori
ERRORS=0

echo "=== Controllo delle variabili d'ambiente ==="

# Loop su ogni variabile e valore atteso
for ITEM in "${EXPECTED_VALUES[@]}"; do
    VAR=$(echo "$ITEM" | cut -d'=' -f1)
    EXPECTED_VALUE=$(echo "$ITEM" | cut -d'=' -f2)
    CURRENT_VALUE=$(printenv "$VAR")

    if [ -z "$CURRENT_VALUE" ]; then
        echo "❌ La variabile $VAR non è definita!"
        ERRORS=$((ERRORS + 1))
    elif [ "$CURRENT_VALUE" != "$EXPECTED_VALUE" ]; then
        echo "❌ La variabile $VAR non è corretta!"
        echo "   Atteso: $EXPECTED_VALUE"
        echo "   Attuale: $CURRENT_VALUE"
        ERRORS=$((ERRORS + 1))
    else
        echo "✅ $VAR è definita correttamente: $CURRENT_VALUE"
    fi
done

# Esito finale
if [ $ERRORS -eq 0 ]; then
    echo "🎉 Tutto è configurato correttamente!"
    echo "Lancio docker-compose..."
    docker compose up --build
else
    echo "⚠️ Ci sono $ERRORS problemi da correggere. Controlla e riprova."
    exit 1
fi
