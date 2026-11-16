# Release

## 2026-05-30 - Stato corrente

### Aggiunto

- Aggiunta documentazione di analisi progetto in `ANALISI_PROGETTO.md`.
- Aggiunto questo file `RELEASE.md` per tracciare le modifiche effettuate.
- Integrata la nuova cartella `eva/` con assistente virtuale, STT, TTS, template web, configurazioni e test.
- Presente la cartella `MarrtinoN8N/` con stack n8n, bridge EVA, workflow Telegram e servizi Docker.
- Aggiunta configurazione chatbot in `asr-tts.json` per scegliere se rispondere tramite EVA o tramite N8N.
- Aggiunta pagina `/chatbot` nel configuratore voice per selezionare provider, endpoint e timeout.
- Aggiunta pagina `config.php` nella web interface principale per gestire la scelta EVA/N8N da browser.
- Aggiunto link `Config` nel menu `nav.php` e nuovo box `CONFIG` nella home `index.php`.

### Modificato

- Aggiornato `.gitignore` per escludere ambienti Python, cache, log, `.env`, output runtime e modelli/cache scaricabili.
- Ripulito `eva/.gitignore`, rimuovendo regole duplicate.
- Rinominato `MarrtinoN8N/.env samplae` in `MarrtinoN8N/.env.sample`.
- Sanitizzato `MarrtinoN8N/.env.sample` sostituendo password e chiavi di esempio con placeholder.
- Sostituiti i modelli Piper duplicati in `eva/models/piper/` con symlink locali verso `MarrtinoN8N/models/`.
- Aggiornato `asr_chatbot_bridge.py` per leggere dinamicamente il provider chatbot da `asr-tts.json`.
- Resa configurabile la porta Flask di `MarrtinoN8N/eva_bridge.py`, con default `5002` per evitare conflitti con EVA su `5000`.
- Rilevate modifiche gia presenti ai file Docker in `docker/humble-host/` per cache bust del clone repository e build via compose.

### Rimosso

- Rimossi log runtime e cache Python da `eva/`.
- Rimossi placeholder vuoti non referenziati in `eva/static/`.
- Rimosso il file metadati `Zone.Identifier` dal workflow n8n.
- Rimosse le copie fisiche duplicate dei modelli Piper da `eva/models/piper/`.

### Note sui duplicati

- I modelli Piper italiani risultano duplicati tra `eva/models/piper/` e `MarrtinoN8N/models/`.
- I duplicati locali in `eva/models/piper/*.onnx` e `eva/models/piper/*.onnx.json` sono stati esclusi da Git per evitare di aumentare il peso del repository.
- In locale, i percorsi in `eva/models/piper/` ora puntano ai file in `MarrtinoN8N/models/`.
- I modelli gia presenti in `MarrtinoN8N/models/` sono stati mantenuti per compatibilita con `docker-compose.yml`.
