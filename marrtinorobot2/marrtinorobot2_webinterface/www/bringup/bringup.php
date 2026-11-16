<?php
$isNavigationPage = isset($pageMode) && $pageMode === 'navigation';
$jsonFile = $isNavigationPage ? 'navigation.json' : 'commands.json';
$pageFile = $isNavigationPage ? 'navigation.php' : 'bringup.php';
$pageTitle = $isNavigationPage ? 'MARRtino Navigation' : 'MARRtino ROS2 Bringup';
$pageSubtitle = $isNavigationPage
    ? 'Console dedicata a LiDAR, mappatura, esplorazione e navigazione.'
    : 'Console di avvio nodi, autostart e diagnostica ROS2.';

function saveCommands($jsonFile, $commands) {
    $commands = array_values($commands);
    $json = json_encode($commands, JSON_PRETTY_PRINT | JSON_UNESCAPED_SLASHES) . PHP_EOL;
    return @file_put_contents($jsonFile, $json, LOCK_EX) !== false;
}
$saveError = '';

// Se il modulo viene inviato, salva i dati
if ($_SERVER['REQUEST_METHOD'] === 'POST') {
    $commands = json_decode(file_get_contents($jsonFile), true);
    if (!is_array($commands)) {
        $commands = [];
    }

    if (isset($_POST['moveCommand'])) {
        $moveParts = explode('|', $_POST['moveCommand'], 2);
        $titleToMove = $moveParts[0] ?? '';
        $direction = $moveParts[1] ?? '';
        $index = null;
        foreach ($commands as $i => $command) {
            if ($command['title'] === $titleToMove) {
                $index = $i;
                break;
            }
        }

        if ($index !== null) {
            $target = $direction === 'up' ? $index - 1 : $index + 1;
            if ($target >= 0 && $target < count($commands)) {
                $tmp = $commands[$target];
                $commands[$target] = $commands[$index];
                $commands[$index] = $tmp;
            }
        }

        if (saveCommands($jsonFile, $commands)) {
            header('Location: ' . $pageFile);
            exit;
        }

        $saveError = 'Impossibile salvare commands.json: verifica i permessi di scrittura del file.';
    }

    $selectedOptions = isset($_POST['selectedOptions']) ? $_POST['selectedOptions'] : [];
    $monitoredOptions = isset($_POST['monitoredOptions']) ? $_POST['monitoredOptions'] : [];

    // Aggiorna i comandi con lo stato selezionato e monitorato
    foreach ($commands as &$command) {
        $command['selected'] = in_array($command['title'], $selectedOptions);
        $command['monitor'] = in_array($command['title'], $monitoredOptions);
    }

    // Salva il file aggiornato
    if (saveCommands($jsonFile, $commands)) {
        header('Location: ' . $pageFile);
        exit;
    }

    $saveError = 'Impossibile salvare commands.json: verifica i permessi di scrittura del file.';
}

// Leggi i dati dei comandi dal file JSON
$commandsRaw = @file_get_contents($jsonFile);
$commands = $commandsRaw !== false ? json_decode($commandsRaw, true) : [];
if (!is_array($commands)) {
    $commands = [];
    $saveError = "Impossibile leggere {$jsonFile}: verifica formato e permessi.";
}
$autostartCount = count(array_filter($commands, function ($command) {
    return !empty($command['selected']);
}));
$manualCount = count($commands) - $autostartCount;
$monitorCount = count(array_filter($commands, function ($command) {
    return !empty($command['monitor']);
}));
$asrChatbotMonitor = false;
foreach ($commands as $command) {
    if (($command['title'] ?? '') === 'asr-chatbot-bridge') {
        $asrChatbotMonitor = !empty($command['monitor']);
        break;
    }
}
?>

<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8" />
    <title><?= htmlspecialchars($pageTitle) ?></title>
    <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
    <link rel="stylesheet" href="../bootstrap/css/bootstrap.min.css">
    <script src="../js/jquery-3.4.1.min.js"></script>
    <script src="../bootstrap/js/bootstrap.min.js"></script>
    <script type="text/javascript" src="../js/roslib.min.js"></script>
    <style>
        :root {
            --accent: rgb(157, 32, 104);
            --accent-2: #38d6ff;
            --accent-3: #42f59b;
            --bg-0: #070a12;
            --bg-1: #0d1322;
            --panel: rgba(14, 20, 35, 0.9);
            --panel-2: rgba(255, 255, 255, 0.06);
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

        .bringup-shell {
            padding: 24px;
        }

        .page-heading {
            display: flex;
            align-items: flex-end;
            justify-content: space-between;
            gap: 18px;
            margin-bottom: 20px;
        }

        .page-heading h1 {
            margin: 0;
            font-size: clamp(1.7rem, 3vw, 2.6rem);
            font-weight: 800;
            letter-spacing: 0;
        }

        .page-heading p {
            margin: 6px 0 0;
            color: var(--muted);
        }

        .top-actions {
            display: flex;
            gap: 10px;
            margin-top: 14px;
            flex-wrap: wrap;
        }

        .monitor-hint {
            color: #cbd5e1;
            font-size: 0.92rem;
        }

        .status-grid {
            display: grid;
            grid-template-columns: repeat(3, minmax(150px, 1fr));
            gap: 12px;
            min-width: min(100%, 520px);
        }

        .metric {
            border: 1px solid var(--line);
            border-radius: 8px;
            padding: 12px 14px;
            background: linear-gradient(180deg, rgba(255,255,255,0.08), rgba(255,255,255,0.035));
            box-shadow: inset 0 1px 0 rgba(255,255,255,0.08);
        }

        .metric span {
            display: block;
            color: var(--muted);
            font-size: 0.78rem;
            text-transform: uppercase;
        }

        .metric strong {
            display: block;
            margin-top: 4px;
            font-size: 1.45rem;
            line-height: 1;
        }

        .control-panel {
            border: 1px solid var(--line);
            border-radius: 8px;
            background: var(--panel);
            box-shadow: 0 24px 80px rgba(0, 0, 0, 0.34), 0 0 0 1px rgba(157, 32, 104, 0.12);
            overflow: hidden;
        }

        .panel-title {
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 14px 16px;
            border-bottom: 1px solid var(--line);
            background: linear-gradient(90deg, rgba(157, 32, 104, 0.24), rgba(56, 214, 255, 0.08));
        }

        .panel-title h2,
        .panel-title h5 {
            margin: 0;
            font-size: 1rem;
            font-weight: 700;
        }

        .log-toolbar {
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .log-level-select {
            min-width: 108px;
            height: 32px;
            padding: 3px 8px;
            color: var(--text);
            border: 1px solid var(--line);
            border-radius: 6px;
            background: #111827;
        }

        .log-level-select option {
            color: #111;
            background: #fff;
        }

        .table-console {
            margin: 0;
            color: var(--text);
        }

        .table-console thead th {
            border: 0;
            color: #cbd5e1;
            background: rgba(255, 255, 255, 0.055);
            font-size: 0.78rem;
            text-transform: uppercase;
        }

        .table-console td {
            border-top: 1px solid rgba(255,255,255,0.08);
            vertical-align: middle;
        }

        .table-console tbody tr {
            background: transparent;
        }

        .table-console tbody tr.autostart-row {
            background: rgba(157, 32, 104, 0.13);
        }

        .cmd-name {
            font-weight: 700;
            color: #fff;
        }

        .cmd-desc {
            color: var(--muted);
            max-width: 420px;
        }

        .switch-cell input {
            width: 18px;
            height: 18px;
            accent-color: var(--accent);
        }

        .action-group {
            display: inline-flex;
            gap: 8px;
            white-space: nowrap;
        }

        .btn-cyber {
            border-radius: 6px;
            border: 1px solid rgba(255,255,255,0.18);
            color: #fff;
            background: rgba(255,255,255,0.07);
        }

        .btn-start {
            border-color: rgba(66, 245, 155, 0.5);
            color: #dfffee;
        }

        .btn-stop {
            border-color: rgba(255, 101, 132, 0.55);
            color: #ffe3ea;
        }

        .btn-restart {
            border-color: rgba(255, 193, 7, 0.6);
            color: #fff3cd;
        }

        .btn-save {
            background: var(--accent);
            border-color: var(--accent);
            color: #fff;
            box-shadow: 0 0 18px rgba(157, 32, 104, 0.34);
        }

        .btn-edit {
            border-color: rgba(56, 214, 255, 0.5);
            color: #dff8ff;
        }

        .log-console {
            height: 520px;
            margin: 0;
            padding: 16px;
            color: #d7fff2;
            background: #050914;
            border: 0;
            border-radius: 0;
            overflow-y: auto;
            white-space: pre-wrap;
            font-size: 0.9rem;
        }

        .log-question {
            color: #ffd54f;
            font-weight: 700;
        }

        .log-answer {
            color: #42f59b;
            font-weight: 700;
        }

        @media (max-width: 991px) {
            .page-heading {
                display: block;
            }

            .status-grid {
                margin-top: 14px;
                grid-template-columns: repeat(3, 1fr);
                min-width: 0;
            }
        }

        @media (max-width: 640px) {
            .bringup-shell {
                padding: 16px 10px;
            }

            .status-grid {
                grid-template-columns: 1fr;
            }

            .action-group {
                display: grid;
                grid-template-columns: 1fr 1fr;
            }
        }
    </style>
    <script type="text/javascript">
        var teleop;
        var ros = new ROSLIB.Ros({
            url: 'ws:' + window.location.hostname + ':9090'
        });

        ros.on('connection', function () {
            document.getElementById("status").innerHTML = "Connected";
        });

        ros.on('error', function (error) {
            document.getElementById("status").innerHTML = "Error";
        });

        ros.on('close', function () {
            document.getElementById("status").innerHTML = "Closed";
        });

        window.onload = function () {
            // Eventuali inizializzazioni
        }
    </script>
</head>
<body>
<?php include "../nav.php" ?>
    <div class="container-fluid bringup-shell">
        <?php if ($saveError): ?>
            <div class="alert alert-danger"><?= htmlspecialchars($saveError) ?></div>
        <?php endif; ?>

        <div class="page-heading">
            <div>
                <h1><?= htmlspecialchars($pageTitle) ?></h1>
                <p><?= htmlspecialchars($pageSubtitle) ?></p>
                <p class="monitor-hint">Monitor abilita lo streaming live stdout/stderr del comando nella console log.</p>
                <div class="top-actions">
                    <a href="performance.php" class="btn btn-cyber btn-edit">Performance Monitor</a>
                    <a href="edit_commands.php?set=<?= $isNavigationPage ? 'navigation' : 'bringup' ?>" class="btn btn-cyber btn-edit">Edit Commands</a>
                    <?php if ($isNavigationPage): ?>
                        <a href="bringup.php" class="btn btn-cyber btn-edit">Back to Bringup</a>
                    <?php else: ?>
                        <a href="navigation.php" class="btn btn-cyber btn-edit">Navigation</a>
                    <?php endif; ?>
                    <a href="/config.php" class="btn btn-cyber btn-edit">Config</a>
                </div>
            </div>
            <div class="status-grid">
                <div class="metric">
                    <span>Comandi</span>
                    <strong><?= count($commands) ?></strong>
                </div>
                <div class="metric">
                    <span>Autostart</span>
                    <strong><?= $autostartCount ?></strong>
                </div>
                <div class="metric">
                    <span>Manuali</span>
                    <strong><?= $manualCount ?></strong>
                </div>
            </div>
        </div>
        <form method="POST" action="">
            <div class="row">
                <!-- Tabella con i comandi -->
                <div class="col-lg-7 mb-4">
                    <div class="control-panel">
                    <div class="panel-title">
                        <h2>Node Matrix</h2>
                        <span><?= $monitorCount ?> monitor</span>
                    </div>
                    <div class="table-responsive">
                    <table class="table table-console">
                        <thead>
                            <tr>
                                <th>Command</th>
                                <th>Description</th>
                                <th>Autostart</th>
                                <th>Monitor</th>
                                <th>Order</th>
                                <th>Actions</th>
                            </tr>
                        </thead>
                        <tbody>
                            <?php foreach ($commands as $index => $command): ?>
                                <tr class="<?= $command['selected'] ? 'autostart-row' : '' ?>">
                                    <td class="cmd-name"><?= htmlspecialchars($command['title']) ?></td>
                                    <td class="cmd-desc"><?= htmlspecialchars($command['description']) ?></td>
                                    <td align="center" class="switch-cell">
                                        <input type="checkbox" name="selectedOptions[]" value="<?= htmlspecialchars($command['title']) ?>" <?= $command['selected'] ? 'checked' : '' ?>>
                                    </td>
                                    <td align="center" class="switch-cell">
                                        <input type="checkbox" name="monitoredOptions[]" value="<?= htmlspecialchars($command['title']) ?>" <?= $command['monitor'] ? 'checked' : '' ?>>
                                    </td>
                                    <td>
                                        <button type="submit" name="moveCommand" value="<?= htmlspecialchars($command['title'] . '|up') ?>" class="btn btn-sm btn-cyber" <?= $index === 0 ? 'disabled' : '' ?>>Up</button>
                                        <button type="submit" name="moveCommand" value="<?= htmlspecialchars($command['title'] . '|down') ?>" class="btn btn-sm btn-cyber" <?= $index === count($commands) - 1 ? 'disabled' : '' ?>>Down</button>
                                    </td>
                                    <td>
                                        <div class="action-group">
                                            <button type="button" onclick='sendCommand(<?= htmlspecialchars(json_encode($command['startCommand'], JSON_HEX_APOS | JSON_HEX_QUOT), ENT_QUOTES, 'UTF-8') ?>)' class="btn btn-sm btn-cyber btn-start">Start</button>
                                            <button type="button" onclick='sendCommand(<?= htmlspecialchars(json_encode($command['stopCommand'], JSON_HEX_APOS | JSON_HEX_QUOT), ENT_QUOTES, 'UTF-8') ?>)' class="btn btn-sm btn-cyber btn-stop">Stop</button>
                                            <button type="button" onclick='restartCommand(<?= htmlspecialchars(json_encode($command['stopCommand'], JSON_HEX_APOS | JSON_HEX_QUOT), ENT_QUOTES, 'UTF-8') ?>, <?= htmlspecialchars(json_encode($command['startCommand'], JSON_HEX_APOS | JSON_HEX_QUOT), ENT_QUOTES, 'UTF-8') ?>)' class="btn btn-sm btn-cyber btn-restart">Restart</button>
                                        </div>
                                    </td>
                                </tr>
                            <?php endforeach; ?>
                        </tbody>
                    </table>
                    </div>
                    <div class="panel-title">
                        <button type="submit" class="btn btn-save">Save Preferences</button>
                        <a href="edit_commands.php?set=<?= $isNavigationPage ? 'navigation' : 'bringup' ?>" class="btn btn-cyber btn-edit">Edit Commands</a>
                    </div>
                    </div>
         
                </div>

                <!-- Area log -->
                <div class="col-lg-5 mb-4">
                    <div class="control-panel">
                        <div class="panel-title">
                            <h5>Live Log</h5>
                            <div class="log-toolbar">
                                <select id="log_level" class="log-level-select" aria-label="Filtra livello log">
                                    <option value="ALL">ALL</option>
                                    <option value="DEBUG">DEBUG</option>
                                    <option value="INFO">INFO</option>
                                    <option value="WARN">WARN</option>
                                    <option value="ERROR">ERROR / ERR</option>
                                </select>
                                <button id="clear_log" type="button" class="btn btn-sm btn-cyber btn-stop">Clear</button>
                                <span>port 9236</span>
                            </div>
                        </div>
                        <pre id="log_msg" class="log-console"></pre>
                    </div>
                </div>
            </div>
        </form>
    </div>

    <script>
        // Define the topic to send commands
        var commandTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/shell_command',
            messageType: 'std_msgs/String'
        });

        // Funzione per inviare comandi
        function sendCommand(command) {
            var commandMessage = new ROSLIB.Message({
                data: `echo '${command}' | netcat -w 1 localhost 9236`
            });
            commandTopic.publish(commandMessage);
            console.log("Sent command: " + command);
        }

        function restartCommand(stopCommand, startCommand) {
            sendCommand(stopCommand);
            window.setTimeout(function () {
                sendCommand(startCommand);
            }, 1200);
        }

        // Sottoscrizione ai log
        var txt_log = new ROSLIB.Topic({
            ros: ros,
            name: '/log_msg',
            messageType: 'std_msgs/String'
        });

        var logConsole = document.getElementById("log_msg");
        var logLevelSelect = document.getElementById("log_level");
        var clearLogButton = document.getElementById("clear_log");
        var logEntries = [];
        var maxLogEntries = 5000;

        function detectLogLevel(text) {
            // Preferisce il livello ROS interno: ad esempio
            // "[asr-tts-piper] ERR: [INFO] ..." viene classificato INFO.
            var bracketLevels = Array.from(text.matchAll(/\[(DEBUG|INFO|WARN(?:ING)?|ERROR|FATAL)\]/gi));
            if (bracketLevels.length) {
                var inner = bracketLevels[bracketLevels.length - 1][1].toUpperCase();
                if (inner === 'WARNING') return 'WARN';
                if (inner === 'FATAL') return 'ERROR';
                return inner;
            }
            var outer = text.match(/\b(DEBUG|INFO|WARN(?:ING)?|ERR(?:OR)?|FATAL)\s*:/i);
            if (!outer) return 'INFO';
            var level = outer[1].toUpperCase();
            if (level === 'WARNING') return 'WARN';
            if (level === 'ERR' || level === 'FATAL') return 'ERROR';
            return level;
        }

        function detectInteractionType(text) {
            if (/\b(?:Hai detto|ASR)\s*:/i.test(text)) return 'question';
            if (/\bChatbot\s*>\s*:/i.test(text)) return 'answer';
            return '';
        }

        function normalizeLogLine(line) {
            var outer = line.match(/^(\[[^\]]+\]\s+(?:DEBUG|INFO|WARN|ERROR)\s*:\s*)(.*)$/i);
            if (!outer) return line;

            var message = outer[2].replace(
                /^\[(?:DEBUG|INFO|WARN(?:ING)?|ERROR|FATAL)\]\s*(?:\[[0-9]+(?:\.[0-9]+)?\]\s*)?\[[^\]]+\]\s*:\s*/i,
                ''
            );
            message = message.replace(/^\[(?:DEBUG|INFO|WARN(?:ING)?|ERROR|FATAL)\]\s*/i, '');
            return outer[1] + message;
        }

        function appendRenderedEntries(entries) {
            var fragment = document.createDocumentFragment();
            entries.forEach(function (entry) {
                var line = document.createElement('span');
                if (entry.type === 'question') line.className = 'log-question';
                if (entry.type === 'answer') line.className = 'log-answer';
                line.textContent = entry.text;
                fragment.appendChild(line);
            });
            logConsole.appendChild(fragment);
        }

        function renderLogs() {
            var selectedLevel = logLevelSelect.value;
            var visible = logEntries.filter(function (entry) {
                return selectedLevel === 'ALL' || entry.level === selectedLevel;
            });
            logConsole.textContent = '';
            appendRenderedEntries(visible);
            logConsole.scrollTop = logConsole.scrollHeight;
        }

        function appendLog(text) {
            var lines = String(text).split(/\r?\n/);
            if (lines.length && lines[lines.length - 1] === '') lines.pop();
            lines = lines.map(normalizeLogLine);
            lines.forEach(function (line) {
                logEntries.push({
                    level: detectLogLevel(line),
                    type: detectInteractionType(line),
                    text: line + "\n"
                });
            });
            if (logEntries.length > maxLogEntries) {
                logEntries.splice(0, logEntries.length - maxLogEntries);
            }
            var selectedLevel = logLevelSelect.value;
            var visibleEntries = lines.map(function (line) {
                return {
                    level: detectLogLevel(line),
                    type: detectInteractionType(line),
                    text: line + "\n"
                };
            }).filter(function (entry) {
                return selectedLevel === 'ALL' || entry.level === selectedLevel;
            });
            appendRenderedEntries(visibleEntries);
            logConsole.scrollTop = logConsole.scrollHeight;
        }

        logLevelSelect.addEventListener('change', renderLogs);
        clearLogButton.addEventListener('click', function () {
            logEntries = [];
            logConsole.textContent = '';
        });

        txt_log.subscribe(function (m) {
            appendLog(m.data);
        });

        // The bridge may already be running when Monitor is enabled. Reading
        // its ROS logger directly makes its live output available immediately,
        // without requiring robot_bringup.py to have launched the process.
        var asrChatbotMonitorEnabled = <?= $asrChatbotMonitor ? 'true' : 'false' ?>;
        if (asrChatbotMonitorEnabled) {
            var rosoutTopic = new ROSLIB.Topic({
                ros: ros,
                name: '/rosout',
                messageType: 'rcl_interfaces/Log'
            });
            var rosLogLevels = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'};
            rosoutTopic.subscribe(function (m) {
                if (m.name === 'chatbot_bridge_node' || m.name.endsWith('.chatbot_bridge_node')) {
                    var level = rosLogLevels[m.level] || String(m.level);
                    appendLog('[asr-chatbot-bridge] ' + level + ': ' + m.msg);
                }
            });
        }

        // /log_msg is a live topic, so messages emitted during Docker autostart
        // predate the browser connection. Fetch the persisted startup log once,
        // then continue with the live stream above.
        var commandOutput = new ROSLIB.Topic({
            ros: ros,
            name: '/command_output',
            messageType: 'std_msgs/String'
        });
        commandOutput.subscribe(function (m) {
            appendLog(m.data);
        });

        ros.on('connection', function () {
            commandTopic.publish(new ROSLIB.Message({
                data: 'tail -n 200 /tmp/robot_bringup.log 2>&1'
            }));
        });
    </script>
</body>
</html>
