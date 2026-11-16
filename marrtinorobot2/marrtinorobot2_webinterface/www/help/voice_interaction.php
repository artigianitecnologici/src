<?php
// Elenco utente derivato dagli handler EVA. Rimane nel pacchetto web perché
// in produzione la document root non può necessariamente leggere eva/handlers.
$groups = [
    [
        'title' => 'Presentazione e informazioni',
        'description' => 'Presentazione di MARRtino e informazioni sul team Robotics3D.',
        'commands' => [
            'Chi sei?', 'Come ti chiami?', 'Presentati', 'Parlami di te',
            'Parlami di Robotics3D', 'Di cosa si occupa Robotics3D?',
            'Chi fa parte di Robotics3D?',
        ],
    ],
    [
        'title' => 'Ora',
        'description' => 'Consulta l’ora locale del robot.',
        'commands' => ['Che ora è?', 'Che ore sono?', 'Dimmi che ore sono', 'Sai l’ora?'],
    ],
    [
        'title' => 'Timer',
        'description' => 'Crea, controlla o cancella timer. Sono accettati secondi, minuti e ore.',
        'commands' => [
            'Imposta un timer di cinque minuti', 'Metti un timer da 30 secondi',
            'Timer cucina 10 minuti', 'Quanti timer ci sono?', 'Mostra i timer attivi',
            'Cancella il timer 1', 'Cancella tutti i timer',
        ],
    ],
    [
        'title' => 'Movimento della base',
        'description' => 'Muove il robot per una distanza limitata oppure lo ruota dell’angolo richiesto. Senza valore usa 30 cm o 30 gradi.',
        'warning' => 'Controlla che lo spazio attorno al robot sia libero prima di impartire un comando.',
        'commands' => [
            'Avanti', 'Avanti di 50 centimetri', 'Vai avanti di 1 metro',
            'Indietro', 'Indietro di 20 centimetri',
            'Gira a destra di 30 gradi', 'Ruota a sinistra di 90 gradi',
            'Destra', 'Sinistra', 'Fermati',
        ],
    ],
    [
        'title' => 'Ricerca e avvicinamento persone',
        'description' => 'Controlla il rilevatore e l’avvicinamento automatico alle persone.',
        'commands' => [
            'Cerca una persona', 'Trova una persona', 'Avvicinati alla persona',
            'Vieni qui', 'Ferma la ricerca', 'Annulla avvicinamento',
        ],
    ],
    [
        'title' => 'Modalità programmazione',
        'description' => 'Attiva il passaggio diretto dei comandi, ne controlla lo stato o lo disattiva.',
        'commands' => [
            'Avvia programmazione', 'Attiva modalità comando', 'Abilita comandi',
            'Stato programmazione', 'Fine programmazione', 'Disattiva modalità comando',
        ],
    ],
    [
        'title' => 'Sistema',
        'description' => 'Diagnostica e alimentazione del computer del robot.',
        'warning' => 'Attenzione: spegnimento e riavvio interrompono tutti i servizi del robot.',
        'commands' => [
            'Stato sistema', 'Stato rete', 'Uptime', 'Riavvia il robot',
            'Spegni il robot', 'Spegniti', 'Spengiti',
        ],
    ],
];

// robot_commands.py usa le keyword dei cataloghi Bringup e Navigation.
$serviceCommands = [];
foreach (['../bringup/commands.json', '../bringup/navigation.json'] as $catalogPath) {
    $raw = @file_get_contents(__DIR__ . '/' . $catalogPath);
    $catalog = $raw !== false ? json_decode($raw, true) : [];
    if (!is_array($catalog)) {
        continue;
    }
    foreach ($catalog as $entry) {
        if (!is_array($entry)) {
            continue;
        }
        foreach (($entry['voiceKeywords'] ?? []) as $keyword) {
            $keyword = trim((string) $keyword);
            if ($keyword !== '') {
                $serviceCommands[] = 'Avvia ' . $keyword;
                $serviceCommands[] = 'Ferma ' . $keyword;
            }
        }
    }
}
$serviceCommands = array_values(array_unique($serviceCommands));
if ($serviceCommands) {
    $groups[] = [
        'title' => 'Servizi robot e navigazione',
        'description' => 'Avvia o ferma i servizi autorizzati configurati nei pannelli Bringup e Navigation.',
        'commands' => $serviceCommands,
    ];
}
?>
<!DOCTYPE html>
<html lang="it">
<head>
  <meta charset="utf-8">
  <title>Help - Interazione vocale</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
  <link rel="stylesheet" href="../bootstrap/css/bootstrap.min.css">
  <script src="../js/jquery-3.4.1.min.js"></script>
  <script src="../bootstrap/js/bootstrap.min.js"></script>
  <script src="../js/roslib.min.js"></script>
  <style>
    body { background: #f5f6fa; }
    .help-shell { max-width: 1050px; padding-top: 28px; padding-bottom: 50px; }
    .intro-card, .command-card { border: 0; border-radius: 10px; box-shadow: 0 8px 28px rgba(20, 25, 40, .09); }
    .wake-word { color: rgb(157, 32, 104); font-weight: 700; }
    .feature-title { color: rgb(157, 32, 104); font-weight: 700; }
    .command-chip { display: inline-block; margin: 4px 5px 4px 0; padding: 7px 11px; border: 1px solid #dce1ea; border-radius: 999px; background: #f8f9fc; }
  </style>
  <script>
    var ros = new ROSLIB.Ros({url: 'ws:' + window.location.hostname + ':9090'});
    ros.on('connection', function () { document.getElementById('status').textContent = 'Connected'; });
    ros.on('error', function () { document.getElementById('status').textContent = 'Error'; });
    ros.on('close', function () { document.getElementById('status').textContent = 'Closed'; });
  </script>
</head>
<body>
  <?php include '../nav.php'; ?>
  <main class="container help-shell">
    <h1>Interazione vocale</h1>
    <p class="lead text-muted">Comandi e suggerimenti per parlare con MARRtino.</p>

    <section class="card intro-card mb-4">
      <div class="card-body">
        <h2 class="h5">Come impartire un comando</h2>
        <p class="mb-2">Pronuncia prima la wake word <span class="wake-word">“Marrtino”</span> oppure <span class="wake-word">“Eva”</span>, quindi il comando senza fare una lunga pausa.</p>
        <p class="mb-0"><strong>Esempio:</strong> “Marrtino, imposta un timer di cinque minuti”. Se il robot riconosce soltanto la wake word, puoi pronunciare il comando entro 8 secondi.</p>
      </div>
    </section>

    <h2 class="h4 mb-3">Comandi disponibili per funzionalità</h2>
    <?php foreach ($groups as $group): ?>
      <section class="card command-card mb-3">
        <div class="card-body">
          <h3 class="h5 feature-title"><?= htmlspecialchars($group['title'], ENT_QUOTES, 'UTF-8') ?></h3>
          <p class="text-muted"><?= htmlspecialchars($group['description'], ENT_QUOTES, 'UTF-8') ?></p>
          <?php if (!empty($group['warning'])): ?>
            <div class="alert alert-warning py-2"><?= htmlspecialchars($group['warning'], ENT_QUOTES, 'UTF-8') ?></div>
          <?php endif; ?>
          <div>
            <?php foreach ($group['commands'] as $command): ?>
              <span class="command-chip">“<?= htmlspecialchars($command, ENT_QUOTES, 'UTF-8') ?>”</span>
            <?php endforeach; ?>
          </div>
        </div>
      </section>
    <?php endforeach; ?>

    <p class="mt-4 text-muted">Oltre ai comandi elencati puoi rivolgere domande libere al chatbot, sempre iniziando con la wake word.</p>
  </main>
</body>
</html>
