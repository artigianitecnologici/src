# Client desktop di Sonia Lite Web — accesso diretto LAN

Il server è **questo progetto**, Sonia Lite Web. Client Python e browser usano
le stesse API HTTP sulla porta **8096**, senza SSH o gateway.
Il modello, le skill e i servizi STT/TTS restano sul server; microfono e altoparlanti
sono quelli del PC client. La conversazione è condivisa fra tutti i dispositivi.

## Preparare il server

Sul computer `192.168.1.199`, dalla cartella del progetto:

```bash
./avvia_web.sh
```

L'ascolto predefinito è `0.0.0.0:8096`, quindi tutti i dispositivi della LAN possono
collegarsi usando l'IP del server. Dopo l'aggiornamento riavvia il processo già attivo.
Se utilizzi il servizio utente:

```bash
systemctl --user restart sonia-lite-web.service
```

Per scegliere una porta o un indirizzo di ascolto:

```bash
SONIA_WEB_HOST=0.0.0.0 SONIA_WEB_PORT=8096 ./avvia_web.sh
# Per tornare all'accesso solo locale:
SONIA_WEB_HOST=127.0.0.1 ./avvia_web.sh
```

Verifica l'indirizzo con `hostname -I`. Se UFW è attivo, sul server autorizza
la sottorete effettiva (qui l'esempio `192.168.1.0/24`):

```bash
sudo ufw allow from 192.168.1.0/24 to any port 8096 proto tcp
```

Non serve esporre llama-server o il servizio audio alla LAN: le loro porte
`8080` e `8092` possono restare su localhost.

## Server in WSL2: autorizzare le porte su Windows

Se Sonia risponde dentro WSL ma non dagli altri PC, controlla anche il firewall
Windows e quello Hyper-V. L'avvio con `--host 0.0.0.0` da solo non apre le porte.
Nei log `[web] 127.0.0.1` indica il client che invia la richiesta, non l'indirizzo
su cui il server è in ascolto.

Queste istruzioni riguardano WSL2 con rete **mirrored**. Verifica dentro WSL:

```bash
wslinfo --networking-mode
ss -ltn 'sport = :8096'
curl --max-time 5 http://127.0.0.1:8096/
```

La modalità deve essere `mirrored`, l'ascolto `0.0.0.0:8096` e curl deve restituire
la pagina HTML. In modalità NAT serve una configurazione diversa, con inoltro
porte Windows verso l'IP di WSL. `192.168.1.199` è un esempio: usa l'IP LAN attuale
del computer Windows.

### Creazione manuale nel firewall Windows

Sul computer Windows che ospita WSL, con privilegi amministrativi:

1. Premi **Win+R**, digita `wf.msc` e apri **Windows Defender Firewall con sicurezza avanzata**.
2. Seleziona **Regole connessioni in entrata** e poi **Nuova regola…**.
3. Scegli **Porta**, poi **TCP** e **Porte locali specifiche: 8096**.
4. Seleziona **Consenti la connessione** e il profilo **Privato**, se la LAN fidata
   è configurata come rete privata in Windows.
5. Assegna il nome **Sonia Web WSL** e completa la procedura.
6. Apri le proprietà della regola: verifica che sia abilitata e, in **Ambito →
   Indirizzi IP remoti**, limita l'accesso alla sottorete della LAN, per esempio
   `192.168.1.0/24`.

Per SSH crea una seconda regola con porta TCP **22**, nome **SSH WSL** e lo stesso
ambito, solo se vuoi accedere al terminale da altri PC. SSH non serve al client Sonia.

Questa procedura crea regole del firewall Windows. Le regole specifiche Hyper-V
si gestiscono con i comandi sotto: se il firewall Hyper-V continua a bloccare
l'ingresso, la sola regola creata in `wf.msc` può non bastare.

### Creazione da PowerShell e regole Hyper-V

Apri **PowerShell come amministratore su Windows**, fuori dalla shell Linux.
Per creare la regola Windows equivalente alla procedura manuale:

```powershell
New-NetFirewallRule -Name "Sonia-Web-LAN" -DisplayName "Sonia Web LAN" -Direction Inbound -Protocol TCP -LocalPort 8096 -RemoteAddress 192.168.1.0/24 -Profile Private -Action Allow -Enabled True
```

Se l'hai già creata manualmente, non è necessario duplicarla. Per consentire
esplicitamente Sonia nel firewall Hyper-V di WSL:

```powershell
New-NetFirewallHyperVRule -Name "WSL-Sonia-Web" -DisplayName "Sonia Web WSL" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol TCP -LocalPorts 8096 -RemoteAddresses 192.168.1.0/24 -Action Allow -Enabled True
```

Per SSH, se necessario:

```powershell
New-NetFirewallRule -Name "WSL-SSH-LAN" -DisplayName "SSH WSL LAN" -Direction Inbound -Protocol TCP -LocalPort 22 -RemoteAddress 192.168.1.0/24 -Profile Private -Action Allow -Enabled True
New-NetFirewallHyperVRule -Name "WSL-SSH" -DisplayName "SSH WSL" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol TCP -LocalPorts 22 -RemoteAddresses 192.168.1.0/24 -Action Allow -Enabled True
```

Se una regola Hyper-V con lo stesso nome esiste già, aggiornala invece di ricrearla:

```powershell
Set-NetFirewallHyperVRule -Name "WSL-Sonia-Web" -RemoteAddresses 192.168.1.0/24 -Enabled True
# Solo se usi SSH e la sua regola esiste già:
Set-NetFirewallHyperVRule -Name "WSL-SSH" -RemoteAddresses 192.168.1.0/24 -Enabled True
```

Controlla le regole effettive e verifica che `Enabled` sia `True`:

```powershell
Get-NetFirewallHyperVVMSetting -PolicyStore ActiveStore -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
Get-NetFirewallHyperVRule -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' | Format-Table Name,Enabled,Direction,Action,Protocol,LocalPorts
```

Dall'altro PC della LAN apri **http://192.168.1.199:8096/**. Da un client Windows
puoi prima controllare la connessione TCP:

```powershell
Test-NetConnection 192.168.1.199 -Port 8096
# Solo per verificare SSH:
Test-NetConnection 192.168.1.199 -Port 22
```

Deve comparire `TcpTestSucceeded : True`. Per SSH il server deve anche essere
installato e attivo dentro WSL (`sudo apt install openssh-server` e
`sudo systemctl enable --now ssh`). Se UFW è attivo, autorizza anche lì la porta
richiesta dalla sottorete LAN. Non occorre disabilitare i firewall.

Riferimenti: [rete WSL e modalità mirrored](https://learn.microsoft.com/en-us/windows/wsl/networking)
e [firewall Hyper-V](https://learn.microsoft.com/en-us/windows/security/operating-system-security/network-security/windows-firewall/hyper-v-firewall).

## Installare e avviare il client

Su Ubuntu / Raspberry Pi OS Desktop:

```bash
sudo apt install -y python3 python3-venv python3-tk libportaudio2
cd ~/sonia-lite-web/client
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
bash connect.sh
```

`connect.sh` avvia il client con `http://192.168.1.199:8096`. Non apre tunnel.
Per un server diverso:

```bash
SONIA_URL=http://192.168.1.50:8096 bash connect.sh
# Oppure:
bash avvia.sh --url http://192.168.1.50:8096
```

Premi **Collega**, scrivi «Ciao Sonia, mi rispondi?» e premi **Invia**.
Non appare alcuna conferma relativa alla fiducia nella connessione.
L'URL può essere modificato anche nell'interfaccia.

Su Windows, con Python e Tkinter installati:

```powershell
cd client
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe sonia_client.py --url http://192.168.1.199:8096
```

## Browser e verifica della rete

Da qualsiasi dispositivo della stessa LAN apri **http://192.168.1.199:8096**.
Puoi verificare che risponda anche dal terminale client:

```bash
curl --max-time 10 http://192.168.1.199:8096/
```

Deve arrivare la pagina HTML di Sonia. Non esiste un endpoint `/health` su questa
web app; il client ottiene il token di sessione dalla pagina e interroga `/api/state`
e `/api/events`. I messaggi vengono inviati a `/api/chat`.

## Chat, voce e comandi

- **Invia**: inoltra il messaggio al modello configurato sul server e mostra la risposta finale.
- **Parla / Termina frase**: registra fino a 60 secondi, invia la registrazione al servizio
  STT del server e inoltra il testo riconosciuto alla chat. Non invia silenzio continuo
  e non usa VAD o wake word. La trascrizione non è in tempo reale.
- **Audio ON/OFF**: abilita o silenzia la riproduzione locale dei WAV TTS del server.
- **Ferma audio**: interrompe la riproduzione locale e silenzia la richiesta corrente;
  l'elaborazione del modello sul server continua.
- **Nuova chat**: azzera la conversazione condivisa, previa conferma; attendi la fine
  di eventuali richieste in corso.
- Le richieste di esecuzione degli script vengono mostrate per approvazione e si
  riferiscono al server Sonia.

STT e TTS richiedono il servizio audio configurato sul server (vedi il README principale).
La chat scritta funziona anche senza audio. Sono supportati WAV PCM16 mono.
Per scegliere i dispositivi locali:

```bash
.venv/bin/python sonia_client.py --list-devices
.venv/bin/python sonia_client.py --input-device 2 --output-device 3
```

Gli indici sono esempi. Per le prime prove usa cuffie. Il microfono del client desktop
funziona anche su HTTP LAN; il microfono **del browser** richiede invece HTTPS o localhost.

## Diagnosi

| Problema | Controllo |
|---|---|
| Timeout o connessione rifiutata | Server avviato e riavviato dopo l'aggiornamento? IP corretto? Porta 8096 ammessa nel firewall? |
| Token assente | L'URL deve puntare alla radice di Sonia Lite Web, porta 8096. |
| Token non valido dopo riavvio server | Premi Scollega e poi Collega per recuperare il nuovo token. |
| Sonia è già occupata | La chat è condivisa: attendi che termini la richiesta dell'altro dispositivo. |
| Chat funziona ma voce assente | Controlla Audio ON, i dispositivi PortAudio e la configurazione STT/TTS sul server. |
| Modello non disponibile | Verifica il modello e il suo server nelle impostazioni della web app. |

## Accesso alla LAN

HTTP non cifra la connessione. Chi può accedere alla web app riceve il token della
sessione e può usare le sue funzioni, incluse le approvazioni: usa questa configurazione
sulla rete locale fidata, con firewall limitato alla sottorete prevista, senza inoltro
porte su Internet. Non è un servizio con account separati per ciascun utente.
I nomi host arbitrari sono rifiutati dal server: usa un indirizzo IP privato della LAN.

## Test

Dalla radice del progetto:

```bash
python3 -m unittest discover -s client -p 'test_*.py' -v
python3 -m unittest discover -s tests -p 'test_sonia_web.py' -v
```

I test usano un server locale e un modello simulato; non certificano la configurazione
del modello, dei dispositivi audio o del firewall su `192.168.1.199`.

## Attivazione vocale «Sonia»

Per usare il client come assistente a mani libere installa sul **PC client**:

```bash
sudo apt install -y libportaudio2
bash client/installa_wake_word.sh
```

Esegui i comandi dalla radice del progetto. Lo script installa Vosk nell'ambiente
`client/.venv` e scarica una sola volta il modello italiano ufficiale
`vosk-model-small-it-0.22` (circa 48 MB) in `client/models/`.
Riferimento: [modelli Vosk](https://alphacephei.com/vosk/models).

Riavvia il client, premi **Collega**, poi **Ascolto Sonia OFF** per portarlo su ON.
Pronuncia una frase che inizi con il nome:

> Sonia, che tempo fa ad Ariccia oggi?

Fai una pausa alla fine (almeno 1,2 secondi, più l'eventuale attesa del risultato finale Vosk): il client riconosce localmente l’attivazione e invia la
frase allo STT del server, poi inoltra il testo a Sonia senza premere Parla o Invia.
Ogni frase è limitata a 30 secondi.
Basta anche pronunciare soltanto «Sonia»: dopo la pausa parte la trascrizione,
senza richiedere altre parole per attivarsi. Per inviare una domanda completa,
pronuncia nome e domanda nella stessa frase. Sono accettati anche «Ehi Sonia» e «Ciao Sonia».
Per avere dati meteo aggiornati serve la skill meteo attiva sul server e l'accesso
Internet del server al servizio meteorologico; il modello da solo non dispone
necessariamente del meteo di oggi.

Il microfono rimane aperto mentre questa modalità è ON. La parola di attivazione viene riconosciuta
sul client: l’audio delle frasi senza attivazione viene scartato localmente.
Dopo «Sonia», la registrazione della frase viene inviata allo STT del server
e il testo riconosciuto viene inoltrato alla chat. Non si salva audio su disco.
Il piccolo modello italiano può sbagliare parole o produrre attivazioni indesiderate:
per il collaudo usa un ambiente silenzioso e cuffie.

L'ascolto si sospende durante l'elaborazione e la riproduzione delle risposte vocali.
Premendo nuovamente il pulsante, **Parla**, **Scollega**, oppure chiudendo il client,
l'ascolto continuo si arresta. All'avvio è sempre OFF e va riattivato dopo una
riconnessione. La dettatura manuale con **Parla** continua a usare lo STT del server;
la modalità **Ascolto Sonia** usa Vosk per l’attivazione e lo stesso STT del server
per trascrivere la domanda: deve quindi funzionare prima la dettatura con **Parla**.

Se compare «Errore wake word», controlla il messaggio nella chat e ripeti lo script
di installazione. Per verificare il microfono usa `--list-devices` e, se necessario,
selezionalo con `--input-device NUMERO` come per la dettatura manuale.


### Configurare la parola di attivazione

Modifica `client/config.json` (sul PC client):

```json
{
  "wake_word": "aurora"
}
```

Il valore predefinito è `sonia`. Sono ammesse parole o brevi frasi composte da
lettere e spazi (massimo 40 caratteri). Maiuscole e minuscole sono equivalenti.
La parola deve essere riconoscibile dal modello italiano Vosk: nomi stranieri o
parole assenti dal suo vocabolario possono non funzionare.

Disattiva e riattiva il pulsante **Ascolto** per rileggere il file, senza
riavviare il client. Il pulsante e i messaggi mostrano il nome configurato.
Nell'esempio puoi dire «Aurora, che tempo fa ad Ariccia oggi?».
Un JSON non valido o una parola vuota impediscono l'attivazione con un messaggio
che indica il file da correggere.

Per usare un file diverso:

```bash
bash client/connect.sh --config /percorso/config-client.json
```


L'ascolto continuo segue il ciclo **attesa del nome → una registrazione →
trascrizione/risposta → nuova attesa del nome**. Dopo ogni richiesta occorre
pronunciare di nuovo la parola configurata. In attesa, il microfono alimenta
il riconoscitore italiano locale con vocabolario completo. La frase viene
mantenuta in RAM per un massimo di 30 secondi e viene scartata se il risultato
finale non conferma il nome all'inizio, con confidenza di almeno 0,85.
Le ipotesi parziali non autorizzano mai l'invio. Nessun audio viene inviato al
server prima di questa conferma; non si usa più la grammatica limitata al nome. Il messaggio «In attesa della
parola …» indica che non è attiva una registrazione della domanda.


### Se compare «Ascolto sospeso»

Il testo dopo il punto indica la causa attuale: trascrizione sul server,
elaborazione del modello, preparazione della voce, riproduzione o breve pausa
successiva. Quando termina, il client torna automaticamente a «In attesa della
parola …». Il motivo viene aggiornato anche quando cambia durante la pausa.

Una coda audio con dispositivo arrestato, oppure senza avanzamento della
riproduzione da oltre due secondi, viene svuotata con un messaggio di errore:
non può mantenere l'ascolto sospeso indefinitamente. **Ferma audio** annulla
anche l'attesa della sintesi vocale; **Audio OFF** evita l'attesa della voce.

Se compare «trascrizione in corso sul server», verifica prima la dettatura con
**Parla** e la configurazione STT sul server. Se il server segnala «Qwen non
raggiungibile», avvia llama-server e verifica l'URL del modello nelle impostazioni
web: l'URL del modello è distinto da quello del client (`:8096`).

## Collegamento al bridge ASR di Marrtino ROS2

Avvia il client sullo stesso computer del bridge ROS:

```bash
./avvia.sh --robot-bridge --url http://192.168.1.199:8096
```

Premi **Collega**. Il client espone `http://127.0.0.1:5003/json`:
ROS invia il testo di `/ASR`, Sonia risponde e ROS pubblica il testo su
`/speech/to_speak`. Nel file `marrtinorobot2_voice/config/asr-tts.json`
sono impostati `chatbot.provider: "sonia"`, l'endpoint nella chiave
`chatbot.endpoints.sonia` e un timeout di lettura di 135 secondi.
Riavvia il bridge ROS dopo la modifica. Se usi `ASR_TTS_CONFIG`, aggiorna
il file indicato da quella variabile.

Per evitare due riproduzioni della risposta, disattiva **Audio ON** nel
client quando usi il TTS del robot. La conversazione resta condivisa:
attendi la risposta prima di inviare altre domande dal client o dal browser.

Se il client gira su un altro PC, avvialo con `--robot-host 0.0.0.0` e
imposta `chatbot.endpoints.sonia` su `http://IP_DEL_PC_CLIENT:5003/json`.
L'endpoint del bridge va usato solo sulla rete fidata del robot.
