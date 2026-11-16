<?php
function first_existing_path($candidates, $directory = false)
{
    foreach ($candidates as $candidate) {
        if (!$candidate) {
            continue;
        }
        $resolved = realpath($candidate);
        if ($resolved !== false && ($directory ? is_dir($resolved) : is_file($resolved))) {
            return $resolved;
        }
    }
    return false;
}

$configPath = first_existing_path([
    getenv('ASR_TTS_CONFIG'),
    '/robot-config/asr-tts.json',
    __DIR__ . '/../../marrtinorobot2_voice/config/asr-tts.json',
    '/home/ubuntu/src/marrtinorobot2/marrtinorobot2_voice/config/asr-tts.json',
    '/home/marrtino/src/marrtinorobot2/marrtinorobot2_voice/config/asr-tts.json',
]);
$modelsPath = first_existing_path([
    '/robot-models/piper',
    __DIR__ . '/../../marrtinorobot2_voice/models/piper',
    '/home/ubuntu/src/marrtinorobot2/marrtinorobot2_voice/models/piper',
    '/home/marrtino/src/marrtinorobot2/marrtinorobot2_voice/models/piper',
], true);
$message = '';
$messageType = 'success';

function default_chatbot_config()
{
    return [
        'provider' => 'eva',
        'endpoints' => [
            'eva' => 'http://127.0.0.1:5000/json',
            'n8n' => 'http://127.0.0.1:5002/json',
        ],
        'connect_timeout' => 3,
        'read_timeout' => 45,
        'max_retries' => 2,
    ];
}

function h($value)
{
    return htmlspecialchars((string)$value, ENT_QUOTES, 'UTF-8');
}

function read_config_file($path)
{
    if (!$path || !file_exists($path)) {
        throw new RuntimeException('File configurazione non trovato.');
    }

    $raw = file_get_contents($path);
    $data = json_decode($raw, true);
    if (!is_array($data)) {
        throw new RuntimeException('JSON configurazione non valido.');
    }

    return $data;
}

function normalize_url_value($value, $fallback)
{
    $value = trim((string)$value);
    if ($value === '') {
        return $fallback;
    }
    return $value;
}

function post_bool($name)
{
    return isset($_POST[$name]) && $_POST[$name] === '1';
}

function post_number($name, $fallback, $minimum = null, $integer = false)
{
    $raw = $_POST[$name] ?? $fallback;
    if (!is_numeric($raw)) {
        throw new InvalidArgumentException("Valore numerico non valido: {$name}");
    }
    $value = $integer ? (int)$raw : (float)$raw;
    return $minimum === null ? $value : max($minimum, $value);
}

function discover_piper_voices($modelsPath)
{
    $voices = [];
    if (!$modelsPath || !is_dir($modelsPath)) {
        return $voices;
    }
    foreach (glob($modelsPath . '/*.onnx') ?: [] as $model) {
        $stem = basename($model, '.onnx');
        $alias = strtolower($stem);
        if (preg_match('/^[a-z]{2}_[A-Z]{2}-(.+)-(x_low|low|medium|high)$/', $stem, $match)) {
            $alias = strtolower($match[1] . '-' . $match[2]);
        }
        $voices[$alias] = basename($model);
    }
    ksort($voices);
    return $voices;
}

function discover_alsa_devices()
{
    $devices = [];
    $pcmPath = '/host-asound/pcm';
    if (!is_readable($pcmPath)) {
        return $devices;
    }
    foreach (file($pcmPath, FILE_IGNORE_NEW_LINES | FILE_SKIP_EMPTY_LINES) ?: [] as $line) {
        if (strpos($line, 'playback') === false) {
            continue;
        }
        if (preg_match('/^(\d+)-(\d+):\s*([^:]+)/', $line, $match)) {
            $card = (int)$match[1];
            $device = (int)$match[2];
            $alsaName = "plughw:{$card},{$device}";
            $devices[$alsaName] = trim($match[3]) . " — {$alsaName}";
        }
    }
    return $devices;
}

function write_config_file($path, $data)
{
    $encoded = json_encode($data, JSON_PRETTY_PRINT | JSON_UNESCAPED_SLASHES | JSON_UNESCAPED_UNICODE);
    if ($encoded === false) {
        throw new RuntimeException('Impossibile serializzare la configurazione.');
    }

    $lockPath = $path . '.lock';
    $lock = fopen($lockPath, 'c');
    if (!$lock) {
        throw new RuntimeException('Impossibile aprire il lock di configurazione.');
    }

    try {
        if (!flock($lock, LOCK_EX)) {
            throw new RuntimeException('Impossibile bloccare il file configurazione.');
        }

        if (file_exists($path)) {
            @copy($path, $path . '.bak');
        }

        // Scrive sull'inode esistente: in questo modo proprietario, gruppo e
        // ACL del bind mount restano invariati. Un rename da un tempnam PHP
        // creerebbe invece un file 0600 di www-data, illeggibile dal nodo ROS.
        $written = file_put_contents($path, $encoded . PHP_EOL, LOCK_EX);
        if ($written === false) {
            throw new RuntimeException('Impossibile scrivere il file configurazione.');
        }
        flock($lock, LOCK_UN);
    } finally {
        fclose($lock);
    }
}

try {
    $config = read_config_file($configPath);
} catch (Throwable $e) {
    $config = [];
    $message = $e->getMessage();
    $messageType = 'danger';
}

$chatbot = default_chatbot_config();
if (isset($config['chatbot']) && is_array($config['chatbot'])) {
    $chatbot = array_replace_recursive($chatbot, $config['chatbot']);
}
$piperVoices = discover_piper_voices($modelsPath);
$alsaDevices = discover_alsa_devices();
if (!$piperVoices) {
    // Mantiene la selezione utilizzabile anche quando PHP è confinato nella
    // document-root e non può elencare la cartella dei modelli.
    $piperVoices = [
        'aurora-medium' => 'it_IT-aurora-medium.onnx',
        'paola-medium' => 'it_IT-paola-medium.onnx',
        'riccardo-x_low' => 'it_IT-riccardo-x_low.onnx',
        'serena-medium' => 'it_IT-serena-medium.onnx',
        'serena-high' => 'it_IT-serena-high.onnx',
        'lessac-medium' => 'en_US-lessac-medium.onnx',
    ];
}

if ($_SERVER['REQUEST_METHOD'] === 'POST' && $configPath) {
    try {
        $provider = strtolower(trim($_POST['provider'] ?? 'eva'));
        if (!in_array($provider, ['eva', 'n8n'], true)) {
            $provider = 'eva';
        }

        $chatbot = [
            'provider' => $provider,
            'endpoints' => [
                'eva' => normalize_url_value($_POST['eva_url'] ?? '', 'http://127.0.0.1:5000/json'),
                'n8n' => normalize_url_value($_POST['n8n_url'] ?? '', 'http://127.0.0.1:5002/json'),
            ],
            'connect_timeout' => max(1, (float)($_POST['connect_timeout'] ?? 3)),
            'read_timeout' => max(1, (float)($_POST['read_timeout'] ?? 45)),
            'max_retries' => max(0, (int)($_POST['max_retries'] ?? 2)),
        ];

        $wakeWords = array_values(array_filter(array_map('trim', explode(',', $_POST['wake_words'] ?? ''))));
        if (!$wakeWords) {
            throw new InvalidArgumentException('Inserisci almeno una wake word.');
        }

        $specialGestures = json_decode($_POST['tts_special_gestures'] ?? '{}', true);
        if (!is_array($specialGestures)) {
            throw new InvalidArgumentException('JSON gesti speciali non valido.');
        }

        $config['output_samplerate'] = post_number('output_samplerate', 48000, 8000, true);
        $config['output_device_index'] = post_number('output_device_index', 1, 0, true);
        $config['input_samplerate'] = post_number('input_samplerate', 16000, 8000, true);
        $config['language'] = trim($_POST['language'] ?? 'it') ?: 'it';
        $config['work_offline'] = post_bool('work_offline');
        $config['msg_start'] = trim($_POST['msg_start'] ?? '');
        $config['wake_words'] = $wakeWords;
        $config['wake_match'] = in_array($_POST['wake_match'] ?? '', ['prefix', 'anywhere', 'exact'], true)
            ? $_POST['wake_match'] : 'prefix';
        $config['wake_word_max_distance'] = post_number('wake_word_max_distance', 1, 0, true);
        $config['question_timeout_sec'] = post_number('question_timeout_sec', 8, 1);
        $config['case_sensitive'] = post_bool('case_sensitive');
        $config['normalize_accents'] = post_bool('normalize_accents');
        $config['beep'] = [
            'enabled' => post_bool('beep_enabled'),
            'frequency' => post_number('beep_frequency', 1000, 20, true),
            'duration_ms' => post_number('beep_duration_ms', 180, 0, true),
            'volume' => min(1, post_number('beep_volume', 0.2, 0)),
        ];
        $config['msg_ack'] = trim($_POST['msg_ack'] ?? '');
        $config['tts_pre_audio_delay_ms'] = post_number('tts_pre_audio_delay_ms', 0, 0, true);
        $config['tts_anim_delay_ms'] = post_number('tts_anim_delay_ms', 0, 0, true);
        $config['tts_pre_delay_ms'] = post_number('tts_pre_delay_ms', 0, 0, true);
        $config['tts_special_gestures'] = $specialGestures;
        $config['piper_models_dir'] = trim($_POST['piper_models_dir'] ?? '');
        $italianVoice = strtolower(trim($_POST['piper_voice_it'] ?? 'riccardo-x_low'));
        $englishVoice = strtolower(trim($_POST['piper_voice_en'] ?? 'lessac-medium'));
        if ($italianVoice === '' || $englishVoice === '') {
            throw new InvalidArgumentException('Seleziona un modello Piper italiano e uno inglese.');
        }
        if (!isset($piperVoices[$italianVoice]) || strpos(strtolower($piperVoices[$italianVoice]), 'it_') !== 0) {
            throw new InvalidArgumentException('Il modello Piper italiano selezionato non è valido.');
        }
        if (!isset($piperVoices[$englishVoice]) || strpos(strtolower($piperVoices[$englishVoice]), 'en_') !== 0) {
            throw new InvalidArgumentException('Il modello Piper inglese selezionato non è valido.');
        }
        $config['piper_voices'] = [
            'it' => $italianVoice,
            'en' => $englishVoice,
        ];
        $config['piper_default_voice'] = $config['language'] === 'en' ? $englishVoice : $italianVoice;
        $config['piper_length_scale'] = post_number('piper_length_scale', 1, 0.1);
        $config['piper_noise_scale'] = post_number('piper_noise_scale', 0.667, 0);
        $config['piper_noise_w'] = post_number('piper_noise_w', 0.8, 0);
        $config['piper_playback'] = [
            'device' => trim($_POST['piper_device'] ?? ''),
            'normalize' => post_bool('piper_normalize'),
            'volume' => post_number('piper_volume', 1, 0),
            'pitch' => post_number('piper_pitch', 100),
            'tempo' => post_number('piper_tempo', 0.9, 0.1),
            'treble' => post_number('piper_treble', 3),
            'highpass' => post_number('piper_highpass', 120, 0),
        ];
        $config['debug'] = post_bool('debug');
        $config['chatbot'] = $chatbot;
        write_config_file($configPath, $config);
        $message = 'Configurazione ASR, TTS e chatbot salvata. Riavvia il nodo voce per applicarla.';
        $messageType = 'success';
    } catch (Throwable $e) {
        $message = $e->getMessage();
        $messageType = 'danger';
    }
}
?>
<!DOCTYPE html>
<html lang="it">

<head>
  <meta charset="utf-8">
  <title>MARRtino Config</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
  <link rel="stylesheet" href="bootstrap/css/bootstrap.min.css">
  <script src="js/jquery-3.4.1.min.js"></script>
  <script src="bootstrap/js/bootstrap.min.js"></script>
  <script src="js/roslib.min.js"></script>
  <script src="js/eventemitter2.min.js"></script>
  <script>
    var ros = new ROSLIB.Ros({ url: 'ws:' + window.location.hostname + ':9090' });
    ros.on('connection', function () { document.getElementById("status").innerHTML = "Connected"; });
    ros.on('error', function () { document.getElementById("status").innerHTML = "Error"; });
    ros.on('close', function () { document.getElementById("status").innerHTML = "Closed"; });
  </script>
  <style>
    :root {
      --accent: rgb(157, 32, 104);
      --accent-2: #38d6ff;
      --bg-0: #070a12;
      --bg-1: #0d1322;
      --panel: rgba(14, 20, 35, 0.9);
      --line: rgba(255, 255, 255, 0.13);
      --text: #eef3ff;
      --muted: #9aa7bd;
    }
    body {
      min-height: 100vh;
      color: var(--text);
      background:
        radial-gradient(circle at 18% 12%, rgba(157, 32, 104, 0.28), transparent 30%),
        radial-gradient(circle at 88% 20%, rgba(56, 214, 255, 0.16), transparent 28%),
        linear-gradient(135deg, var(--bg-0), var(--bg-1) 56%, #190d1a);
    }
    .config-card {
      color: var(--text);
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      box-shadow: 0 24px 80px rgba(0, 0, 0, 0.34), 0 0 0 1px rgba(157, 32, 104, 0.12);
    }
    .provider-option {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 14px;
      height: 100%;
      background: rgba(255, 255, 255, 0.04);
    }
    .provider-option input {
      margin-right: 8px;
    }
    .form-label {
      font-weight: 600;
    }
    .form-control {
      color: var(--text);
      border-color: var(--line);
      background: rgba(255, 255, 255, 0.07);
    }
    .form-control:focus {
      color: #fff;
      border-color: var(--accent-2);
      background: rgba(255, 255, 255, 0.1);
      box-shadow: 0 0 0 .2rem rgba(56, 214, 255, 0.14);
    }
    select.form-control option { color: #111; background: #fff; }
    .text-muted, small.text-muted { color: var(--muted) !important; }
    hr { border-color: var(--line); }
    .btn-save {
      color: #fff;
      border-color: var(--accent);
      background: var(--accent);
      box-shadow: 0 0 18px rgba(157, 32, 104, 0.34);
    }
    .btn-cyber {
      color: #dff8ff;
      border: 1px solid rgba(56, 214, 255, 0.5);
      border-radius: 6px;
      background: rgba(255, 255, 255, 0.07);
    }
    .btn-cyber:hover { color: #fff; background: rgba(56, 214, 255, 0.14); }
    code {
      color: #dff8ff;
      background: rgba(255, 255, 255, 0.08);
      border-radius: 6px;
      padding: 2px 6px;
    }
  </style>
</head>

<body>
  <?php include "nav.php"; ?>

  <div class="container mt-4 mb-5">
    <div class="row">
      <div class="col-lg-10 mx-auto">
        <div class="card config-card">
          <div class="card-body">
            <div class="d-flex align-items-center justify-content-between flex-wrap mb-3">
              <div>
                <h4 class="mb-1">Configurazione voce e chatbot</h4>
                <p class="text-muted mb-0">Personalizza ASR, Piper TTS, audio e provider delle risposte.</p>
              </div>
              <div class="mt-2 mt-md-0">
                <a class="btn btn-cyber mr-2" href="/bringup/bringup.php">Back to Bringup</a>
                <a class="btn btn-cyber" href="index.php">Home</a>
              </div>
            </div>

            <?php if ($message !== ''): ?>
              <div class="alert alert-<?php echo htmlspecialchars($messageType, ENT_QUOTES, 'UTF-8'); ?>">
                <?php echo htmlspecialchars($message, ENT_QUOTES, 'UTF-8'); ?>
              </div>
            <?php endif; ?>

            <p class="text-muted">File: <code><?php echo htmlspecialchars((string)$configPath, ENT_QUOTES, 'UTF-8'); ?></code></p>

            <form method="post" action="config.php">
              <h5 class="mt-2">Riconoscimento vocale (ASR)</h5>
              <div class="row">
                <div class="col-md-4 form-group">
                  <label class="form-label" for="input_samplerate">Sample rate ingresso</label>
                  <input class="form-control" type="number" min="8000" id="input_samplerate" name="input_samplerate" value="<?php echo h($config['input_samplerate'] ?? 16000); ?>">
                </div>
                <div class="col-md-4 form-group">
                  <label class="form-label" for="language">Lingua ASR</label>
                  <select class="form-control" id="language" name="language">
                    <option value="it" <?php echo (($config['language'] ?? 'it') === 'it') ? 'selected' : ''; ?>>Italiano</option>
                    <option value="en" <?php echo (($config['language'] ?? '') === 'en') ? 'selected' : ''; ?>>English</option>
                  </select>
                </div>
                <div class="col-md-4 form-group">
                  <label class="form-label" for="question_timeout_sec">Attesa domanda (secondi)</label>
                  <input class="form-control" type="number" min="1" step="0.5" id="question_timeout_sec" name="question_timeout_sec" value="<?php echo h($config['question_timeout_sec'] ?? 8); ?>">
                </div>
              </div>
              <div class="form-group">
                <label class="form-label" for="wake_words">Wake word (separate da virgola)</label>
                <input class="form-control" id="wake_words" name="wake_words" value="<?php echo h(implode(', ', $config['wake_words'] ?? ['martino'])); ?>">
              </div>
              <div class="row">
                <div class="col-md-6 form-group">
                  <label class="form-label" for="wake_match">Modalità wake word</label>
                  <select class="form-control" id="wake_match" name="wake_match">
                    <?php foreach (['prefix', 'anywhere', 'exact'] as $mode): ?>
                      <option value="<?php echo h($mode); ?>" <?php echo (($config['wake_match'] ?? 'prefix') === $mode) ? 'selected' : ''; ?>><?php echo h($mode); ?></option>
                    <?php endforeach; ?>
                  </select>
                </div>
                <div class="col-md-6 form-group">
                  <label class="form-label" for="wake_word_max_distance">Tolleranza errore wake word</label>
                  <input class="form-control" type="number" min="0" max="3" id="wake_word_max_distance" name="wake_word_max_distance" value="<?php echo h($config['wake_word_max_distance'] ?? 1); ?>">
                </div>
              </div>
              <div class="row mb-3">
                <?php foreach ([
                    'work_offline' => 'Lavora offline',
                    'case_sensitive' => 'Distingui maiuscole/minuscole',
                    'normalize_accents' => 'Normalizza accenti',
                    'debug' => 'Log di debug'
                ] as $name => $label): ?>
                  <div class="col-md-3">
                    <div class="custom-control custom-switch">
                      <input type="checkbox" class="custom-control-input" id="<?php echo h($name); ?>" name="<?php echo h($name); ?>" value="1" <?php echo !empty($config[$name]) ? 'checked' : ''; ?>>
                      <label class="custom-control-label" for="<?php echo h($name); ?>"><?php echo h($label); ?></label>
                    </div>
                  </div>
                <?php endforeach; ?>
              </div>

              <hr>
              <h5>Sintesi vocale Piper</h5>
              <?php
                $configuredPiperVoices = is_array($config['piper_voices'] ?? null) ? $config['piper_voices'] : [];
                $legacyVoice = strtolower($config['piper_default_voice'] ?? 'riccardo-x_low');
                $selectedItalianVoice = strtolower($configuredPiperVoices['it'] ?? $legacyVoice);
                $selectedEnglishVoice = strtolower($configuredPiperVoices['en'] ?? 'lessac-medium');
                $italianVoices = array_filter($piperVoices, function ($filename) {
                    return strpos(strtolower($filename), 'it_') === 0;
                });
                $englishVoices = array_filter($piperVoices, function ($filename) {
                    return strpos(strtolower($filename), 'en_') === 0;
                });
              ?>
              <div class="row">
                <div class="col-md-4 form-group">
                  <label class="form-label" for="piper_voice_it">Modello italiano</label>
                  <select class="form-control" id="piper_voice_it" name="piper_voice_it" required>
                    <?php foreach ($italianVoices as $voice => $filename): ?>
                      <option value="<?php echo h($voice); ?>" <?php echo $selectedItalianVoice === $voice ? 'selected' : ''; ?>><?php echo h($voice . ' — ' . $filename); ?></option>
                    <?php endforeach; ?>
                  </select>
                  <?php if (!$italianVoices): ?><small class="text-danger">Nessun modello Piper italiano trovato.</small><?php endif; ?>
                </div>
                <div class="col-md-4 form-group">
                  <label class="form-label" for="piper_voice_en">English model</label>
                  <select class="form-control" id="piper_voice_en" name="piper_voice_en" required>
                    <?php foreach ($englishVoices as $voice => $filename): ?>
                      <option value="<?php echo h($voice); ?>" <?php echo $selectedEnglishVoice === $voice ? 'selected' : ''; ?>><?php echo h($voice . ' — ' . $filename); ?></option>
                    <?php endforeach; ?>
                  </select>
                  <?php if (!$englishVoices): ?><small class="text-danger">No English Piper model found.</small><?php endif; ?>
                </div>
                <div class="col-md-4 form-group">
                  <label class="form-label" for="piper_models_dir">Cartella modelli Piper</label>
                  <input class="form-control" id="piper_models_dir" name="piper_models_dir" value="<?php echo h($config['piper_models_dir'] ?? ''); ?>">
                </div>
              </div>
              <div class="row">
                <div class="col-md-4 form-group"><label class="form-label" for="piper_length_scale">Length scale</label><input class="form-control" type="number" step="0.01" min="0.1" id="piper_length_scale" name="piper_length_scale" value="<?php echo h($config['piper_length_scale'] ?? 1); ?>"><small class="text-muted">Più alto = voce più lenta.</small></div>
                <div class="col-md-4 form-group"><label class="form-label" for="piper_noise_scale">Noise scale</label><input class="form-control" type="number" step="0.01" min="0" id="piper_noise_scale" name="piper_noise_scale" value="<?php echo h($config['piper_noise_scale'] ?? 0.667); ?>"></div>
                <div class="col-md-4 form-group"><label class="form-label" for="piper_noise_w">Noise W</label><input class="form-control" type="number" step="0.01" min="0" id="piper_noise_w" name="piper_noise_w" value="<?php echo h($config['piper_noise_w'] ?? 0.8); ?>"></div>
              </div>
              <?php $playback = $config['piper_playback'] ?? []; ?>
              <div class="form-group">
                <label class="form-label" for="piper_device">Dispositivo ALSA per voce e beep</label>
                <?php $selectedAlsaDevice = $playback['device'] ?? ''; ?>
                <select class="form-control" id="piper_device" name="piper_device">
                  <option value="" <?php echo $selectedAlsaDevice === '' ? 'selected' : ''; ?>>Automatico dall'indice di uscita</option>
                  <?php if ($selectedAlsaDevice !== '' && !isset($alsaDevices[$selectedAlsaDevice])): ?>
                    <option value="<?php echo h($selectedAlsaDevice); ?>" selected><?php echo h($selectedAlsaDevice . ' — configurato'); ?></option>
                  <?php endif; ?>
                  <?php foreach ($alsaDevices as $device => $label): ?>
                    <option value="<?php echo h($device); ?>" <?php echo $selectedAlsaDevice === $device ? 'selected' : ''; ?>><?php echo h($label); ?></option>
                  <?php endforeach; ?>
                </select>
                <small class="text-muted">Lascia vuoto per ricavarlo automaticamente dall'indice del dispositivo di uscita.</small>
              </div>
              <div class="row">
                <?php foreach ([
                    'volume' => ['Volume', 1, 0.05], 'pitch' => ['Pitch', 100, 1],
                    'tempo' => ['Tempo', 0.9, 0.05], 'treble' => ['Alti', 3, 0.5],
                    'highpass' => ['Filtro passa-alto', 120, 1]
                ] as $key => $details): ?>
                  <div class="col-md form-group">
                    <label class="form-label" for="piper_<?php echo h($key); ?>"><?php echo h($details[0]); ?></label>
                    <input class="form-control" type="number" step="<?php echo h($details[2]); ?>" id="piper_<?php echo h($key); ?>" name="piper_<?php echo h($key); ?>" value="<?php echo h($playback[$key] ?? $details[1]); ?>">
                  </div>
                <?php endforeach; ?>
              </div>
              <div class="custom-control custom-switch mb-3">
                <input type="checkbox" class="custom-control-input" id="piper_normalize" name="piper_normalize" value="1" <?php echo !isset($playback['normalize']) || $playback['normalize'] ? 'checked' : ''; ?>>
                <label class="custom-control-label" for="piper_normalize">Normalizza volume Piper</label>
              </div>

              <hr>
              <h5>Audio e messaggi</h5>
              <div class="row">
                <div class="col-md-4 form-group"><label class="form-label" for="output_samplerate">Sample rate uscita</label><input class="form-control" type="number" min="8000" id="output_samplerate" name="output_samplerate" value="<?php echo h($config['output_samplerate'] ?? 48000); ?>"></div>
                <div class="col-md-4 form-group"><label class="form-label" for="output_device_index">Indice dispositivo uscita</label><input class="form-control" type="number" min="0" id="output_device_index" name="output_device_index" value="<?php echo h($config['output_device_index'] ?? 1); ?>"></div>
                <div class="col-md-4 form-group"><label class="form-label" for="tts_pre_audio_delay_ms">Ritardo audio (ms)</label><input class="form-control" type="number" min="0" id="tts_pre_audio_delay_ms" name="tts_pre_audio_delay_ms" value="<?php echo h($config['tts_pre_audio_delay_ms'] ?? 0); ?>"></div>
              </div>
              <div class="row">
                <div class="col-md-6 form-group"><label class="form-label" for="msg_start">Messaggio iniziale</label><input class="form-control" id="msg_start" name="msg_start" value="<?php echo h($config['msg_start'] ?? ''); ?>"></div>
                <div class="col-md-6 form-group"><label class="form-label" for="msg_ack">Messaggio di conferma</label><input class="form-control" id="msg_ack" name="msg_ack" value="<?php echo h($config['msg_ack'] ?? ''); ?>"></div>
              </div>
              <?php $beep = $config['beep'] ?? []; ?>
              <div class="row">
                <div class="col-md-3 form-group"><div class="custom-control custom-switch mt-4"><input type="checkbox" class="custom-control-input" id="beep_enabled" name="beep_enabled" value="1" <?php echo !empty($beep['enabled']) ? 'checked' : ''; ?>><label class="custom-control-label" for="beep_enabled">Beep attivo</label></div></div>
                <div class="col-md-3 form-group"><label class="form-label" for="beep_frequency">Frequenza beep</label><input class="form-control" type="number" min="20" id="beep_frequency" name="beep_frequency" value="<?php echo h($beep['frequency'] ?? 1000); ?>"></div>
                <div class="col-md-3 form-group"><label class="form-label" for="beep_duration_ms">Durata beep (ms)</label><input class="form-control" type="number" min="0" id="beep_duration_ms" name="beep_duration_ms" value="<?php echo h($beep['duration_ms'] ?? 180); ?>"></div>
                <div class="col-md-3 form-group"><label class="form-label" for="beep_volume">Volume beep</label><input class="form-control" type="number" min="0" max="1" step="0.05" id="beep_volume" name="beep_volume" value="<?php echo h($beep['volume'] ?? 0.2); ?>"></div>
              </div>
              <div class="row">
                <div class="col-md-6 form-group"><label class="form-label" for="tts_anim_delay_ms">Ritardo animazione (ms)</label><input class="form-control" type="number" min="0" id="tts_anim_delay_ms" name="tts_anim_delay_ms" value="<?php echo h($config['tts_anim_delay_ms'] ?? 0); ?>"></div>
                <div class="col-md-6 form-group"><label class="form-label" for="tts_pre_delay_ms">Pre-delay TTS (ms)</label><input class="form-control" type="number" min="0" id="tts_pre_delay_ms" name="tts_pre_delay_ms" value="<?php echo h($config['tts_pre_delay_ms'] ?? 0); ?>"></div>
              </div>
              <div class="form-group">
                <label class="form-label" for="tts_special_gestures">Gesti speciali TTS (JSON)</label>
                <textarea class="form-control text-monospace" rows="8" id="tts_special_gestures" name="tts_special_gestures"><?php echo h(json_encode($config['tts_special_gestures'] ?? [], JSON_PRETTY_PRINT | JSON_UNESCAPED_UNICODE)); ?></textarea>
              </div>

              <hr>
              <h5>Chatbot</h5>
              <div class="row mb-3">
                <div class="col-md-6 mb-3 mb-md-0">
                  <label class="provider-option d-block">
                    <input type="radio" name="provider" value="eva" <?php echo ($chatbot['provider'] === 'eva') ? 'checked' : ''; ?>>
                    <strong>EVA</strong>
                    <span class="d-block text-muted mt-1">Risposte dirette dall'assistente EVA.</span>
                  </label>
                </div>
                <div class="col-md-6">
                  <label class="provider-option d-block">
                    <input type="radio" name="provider" value="n8n" <?php echo ($chatbot['provider'] === 'n8n') ? 'checked' : ''; ?>>
                    <strong>N8N</strong>
                    <span class="d-block text-muted mt-1">Risposte dal workflow N8N tramite eva_bridge.</span>
                  </label>
                </div>
              </div>

              <div class="form-group">
                <label class="form-label" for="eva_url">Endpoint EVA</label>
                <input class="form-control" id="eva_url" name="eva_url" value="<?php echo htmlspecialchars($chatbot['endpoints']['eva'], ENT_QUOTES, 'UTF-8'); ?>">
              </div>

              <div class="form-group">
                <label class="form-label" for="n8n_url">Endpoint N8N</label>
                <input class="form-control" id="n8n_url" name="n8n_url" value="<?php echo htmlspecialchars($chatbot['endpoints']['n8n'], ENT_QUOTES, 'UTF-8'); ?>">
              </div>

              <div class="row">
                <div class="col-md-4">
                  <div class="form-group">
                    <label class="form-label" for="connect_timeout">Timeout connessione</label>
                    <input class="form-control" id="connect_timeout" name="connect_timeout" type="number" min="1" step="0.5" value="<?php echo htmlspecialchars((string)$chatbot['connect_timeout'], ENT_QUOTES, 'UTF-8'); ?>">
                  </div>
                </div>
                <div class="col-md-4">
                  <div class="form-group">
                    <label class="form-label" for="read_timeout">Timeout risposta</label>
                    <input class="form-control" id="read_timeout" name="read_timeout" type="number" min="1" step="1" value="<?php echo htmlspecialchars((string)$chatbot['read_timeout'], ENT_QUOTES, 'UTF-8'); ?>">
                  </div>
                </div>
                <div class="col-md-4">
                  <div class="form-group">
                    <label class="form-label" for="max_retries">Tentativi extra</label>
                    <input class="form-control" id="max_retries" name="max_retries" type="number" min="0" step="1" value="<?php echo htmlspecialchars((string)$chatbot['max_retries'], ENT_QUOTES, 'UTF-8'); ?>">
                  </div>
                </div>
              </div>

              <button class="btn btn-save" type="submit">Salva configurazione</button>
            </form>
          </div>
        </div>
      </div>
    </div>
  </div>
</body>

</html>
