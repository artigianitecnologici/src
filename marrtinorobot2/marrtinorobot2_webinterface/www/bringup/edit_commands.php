<?php
$dataset = ($_GET['set'] ?? $_POST['set'] ?? 'bringup') === 'navigation' ? 'navigation' : 'bringup';
$jsonFile = $dataset === 'navigation' ? 'navigation.json' : 'commands.json';
$returnPage = $dataset === 'navigation' ? 'navigation.php' : 'bringup.php';

// Leggi il file JSON
$commands = json_decode(file_get_contents($jsonFile), true);
if (!is_array($commands)) {
    $commands = [];
}
$saveError = '';

function saveCommands($jsonFile, $commands) {
    $commands = array_values($commands);
    $json = json_encode($commands, JSON_PRETTY_PRINT | JSON_UNESCAPED_SLASHES) . PHP_EOL;
    return @file_put_contents($jsonFile, $json, LOCK_EX) !== false;
}

// Se il modulo viene inviato, aggiorna il file JSON
if ($_SERVER['REQUEST_METHOD'] === 'POST') {
    if (isset($_POST['saveCommand'])) {
        // Salva o aggiorna un comando esistente
        $originalTitle = trim($_POST['originalTitle'] ?? '');
        $title = trim($_POST['title']);
        $description = trim($_POST['description']);
        $startCommand = trim($_POST['startCommand']);
        $stopCommand = trim($_POST['stopCommand']);
        $selected = isset($_POST['selected']) ? true : false;
        $monitor = isset($_POST['monitor']) ? true : false;
        $voiceKeywords = array_values(array_filter(array_map('trim', explode(',', $_POST['voiceKeywords'] ?? ''))));

        // Cerca il comando nel JSON
        $found = false;
        foreach ($commands as &$command) {
            if ($command['title'] === ($originalTitle !== '' ? $originalTitle : $title)) {
                $command['title'] = $title;
                $command['description'] = $description;
                $command['startCommand'] = $startCommand;
                $command['stopCommand'] = $stopCommand;
                $command['selected'] = $selected;
                $command['monitor'] = $monitor;
                $command['voiceKeywords'] = $voiceKeywords;
                $found = true;
                break;
            }
        }

        // Se non trovato, aggiungilo
        if (!$found) {
            $commands[] = [
                'title' => $title,
                'description' => $description,
                'startCommand' => $startCommand,
                'stopCommand' => $stopCommand,
                'selected' => $selected,
                'monitor' => $monitor
                ,'voiceKeywords' => $voiceKeywords
            ];
        }
    }

    if (isset($_POST['moveCommand'], $_POST['direction'])) {
        $titleToMove = $_POST['moveCommand'];
        $direction = $_POST['direction'];
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
    }

    if (isset($_POST['deleteCommand'])) {
        // Elimina un comando
        $titleToDelete = $_POST['deleteCommand'];
        $commands = array_filter($commands, function ($command) use ($titleToDelete) {
            return $command['title'] !== $titleToDelete;
        });
    }

    // Salva le modifiche nel file JSON
    if (saveCommands($jsonFile, $commands)) {
        header('Location: edit_commands.php?set=' . urlencode($dataset));
        exit;
    }

    $saveError = 'Impossibile salvare commands.json: verifica i permessi di scrittura del file.';
}
?>

<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8" />
    <title>Edit Commands</title>
    <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
    <link rel="stylesheet" href="../bootstrap/css/bootstrap.min.css">
    <script src="../js/jquery-3.4.1.min.js"></script>
    <script src="../bootstrap/js/bootstrap.min.js"></script>
    <script type="text/javascript" src="../js/roslib.min.js"></script>
    <style>
        :root {
            --accent: rgb(157, 32, 104);
            --accent-2: #38d6ff;
            --bg-0: #070a12;
            --bg-1: #0d1322;
            --panel: rgba(14, 20, 35, 0.92);
            --line: rgba(255, 255, 255, 0.13);
            --text: #eef3ff;
            --muted: #9aa7bd;
        }

        body {
            min-height: 100vh;
            color: var(--text);
            background:
                radial-gradient(circle at 15% 12%, rgba(157, 32, 104, 0.26), transparent 32%),
                radial-gradient(circle at 88% 18%, rgba(56, 214, 255, 0.14), transparent 28%),
                linear-gradient(135deg, var(--bg-0), var(--bg-1) 58%, #190d1a);
        }

        .edit-shell {
            padding-top: 28px;
            padding-bottom: 34px;
        }

        .edit-heading {
            display: flex;
            align-items: center;
            justify-content: space-between;
            gap: 16px;
            margin-bottom: 18px;
        }

        .edit-heading h1 {
            margin: 0;
            font-weight: 800;
            letter-spacing: 0;
        }

        .control-panel {
            border: 1px solid var(--line);
            border-radius: 8px;
            background: var(--panel);
            box-shadow: 0 24px 80px rgba(0, 0, 0, 0.34), 0 0 0 1px rgba(157, 32, 104, 0.12);
            overflow: hidden;
        }

        .panel-pad {
            padding: 18px;
        }

        .panel-title {
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 14px 16px;
            border-bottom: 1px solid var(--line);
            background: linear-gradient(90deg, rgba(157, 32, 104, 0.24), rgba(56, 214, 255, 0.08));
        }

        .panel-title h2 {
            margin: 0;
            font-size: 1rem;
            font-weight: 700;
        }

        .form-control,
        textarea.form-control {
            color: var(--text);
            border-color: rgba(255,255,255,0.16);
            background: rgba(255,255,255,0.06);
        }

        .form-control:focus {
            color: var(--text);
            border-color: var(--accent);
            background: rgba(255,255,255,0.08);
            box-shadow: 0 0 0 0.2rem rgba(157, 32, 104, 0.18);
        }

        label {
            color: #d9e3f7;
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

        .table-console tbody tr.autostart-row {
            background: rgba(157, 32, 104, 0.13);
        }

        .btn-accent {
            color: #fff;
            border-color: var(--accent);
            background: var(--accent);
            box-shadow: 0 0 18px rgba(157, 32, 104, 0.34);
        }

        .btn-cyber {
            border-radius: 6px;
            border: 1px solid rgba(255,255,255,0.18);
            color: #fff;
            background: rgba(255,255,255,0.07);
        }

        .badge-state {
            display: inline-block;
            min-width: 44px;
            padding: 4px 8px;
            border-radius: 999px;
            text-align: center;
            background: rgba(255,255,255,0.08);
            color: var(--muted);
        }

        .badge-state.on {
            color: #fff;
            background: rgba(157, 32, 104, 0.5);
        }
    </style>
</head>
<body>
<?php include "../nav.php" ?>
<div class="container-fluid edit-shell">
    <?php if ($saveError): ?>
        <div class="alert alert-danger"><?= htmlspecialchars($saveError) ?></div>
    <?php endif; ?>

    <div class="edit-heading">
        <div>
            <h1>Edit Commands</h1>
            <p class="text-muted mb-0">Catalogo dei nodi disponibili per Start, Stop e Autostart.</p>
        </div>
        <a href="<?= htmlspecialchars($returnPage) ?>" class="btn btn-cyber">Back to <?= $dataset === 'navigation' ? 'Navigation' : 'Bringup' ?></a>
    </div>

    <!-- Pulsante Aggiungi -->
    <button class="btn btn-accent mb-3" onclick="openEditModal()">Add Command</button>

    <!-- Modulo per aggiungere/modificare -->
    <div id="editModal" class="control-panel mb-4" style="display: none;">
        <div class="panel-title">
            <h2>Command Editor</h2>
            <button type="button" class="btn btn-sm btn-cyber" onclick="closeEditModal()">Close</button>
        </div>
        <div class="panel-pad">
        <form method="POST" action="">
            <input type="hidden" name="set" value="<?= htmlspecialchars($dataset) ?>">
            <input type="hidden" id="originalTitle" name="originalTitle" value="">
            <div class="mb-3">
                <label for="title" class="form-label">Title</label>
                <input type="text" id="title" name="title" class="form-control" required>
            </div>
            <div class="mb-3">
                <label for="description" class="form-label">Description</label>
                <textarea id="description" name="description" class="form-control" rows="2" required></textarea>
            </div>
            <div class="mb-3">
                <label for="startCommand" class="form-label">Start Command</label>
                <input type="text" id="startCommand" name="startCommand" class="form-control" required>
            </div>
            <div class="mb-3">
                <label for="stopCommand" class="form-label">Stop Command</label>
                <input type="text" id="stopCommand" name="stopCommand" class="form-control" required>
            </div>
            <div class="mb-3">
                <label for="voiceKeywords" class="form-label">Parole chiave vocali</label>
                <input type="text" id="voiceKeywords" name="voiceKeywords" class="form-control" placeholder="es. lidar, laser, rplidar">
                <small class="text-muted">Separate da virgola. Usa “avvia” o “ferma” prima della parola chiave.</small>
            </div>
            <div class="form-check mb-3">
                <input type="checkbox" id="selected" name="selected" class="form-check-input">
                <label for="selected" class="form-check-label">Autostart</label>
            </div>
            <div class="form-check mb-3">
                <input type="checkbox" id="monitor" name="monitor" class="form-check-input">
                <label for="monitor" class="form-check-label">Monitor</label>
            </div>
            <button type="submit" name="saveCommand" class="btn btn-accent">Save Command</button>
            <button type="button" class="btn btn-cyber" onclick="closeEditModal()">Cancel</button>
        </form>
        </div>
    </div>

    <div class="control-panel">
    <div class="panel-title">
        <h2>Existing Commands</h2>
        <span><?= count($commands) ?> total</span>
    </div>
    <div class="table-responsive">
    <table class="table table-console">
        <thead>
        <tr>
            <th>Title</th>
            <th>Description</th>
            <th>Start Command</th>
            <th>Stop Command</th>
            <th>Voice Keywords</th>
            <th>Autostart</th>
            <th>Monitor</th>
            <th>Order</th>
            <th>Actions</th>
        </tr>
        </thead>
        <tbody>
        <?php foreach ($commands as $index => $command): ?>
            <tr class="<?= $command['selected'] ? 'autostart-row' : '' ?>">
                <td><?= htmlspecialchars($command['title']) ?></td>
                <td><?= htmlspecialchars($command['description']) ?></td>
                <td><?= htmlspecialchars($command['startCommand']) ?></td>
                <td><?= htmlspecialchars($command['stopCommand']) ?></td>
                <td><?= htmlspecialchars(implode(', ', $command['voiceKeywords'] ?? [])) ?></td>
                <td><span class="badge-state <?= $command['selected'] ? 'on' : '' ?>"><?= $command['selected'] ? 'Yes' : 'No' ?></span></td>
                <td><span class="badge-state <?= $command['monitor'] ? 'on' : '' ?>"><?= $command['monitor'] ? 'Yes' : 'No' ?></span></td>
                <td>
                    <form method="POST" action="" style="display:inline;">
                        <input type="hidden" name="set" value="<?= htmlspecialchars($dataset) ?>">
                        <input type="hidden" name="moveCommand" value="<?= htmlspecialchars($command['title']) ?>">
                        <button type="submit" name="direction" value="up" class="btn btn-sm btn-cyber" <?= $index === 0 ? 'disabled' : '' ?>>Up</button>
                        <button type="submit" name="direction" value="down" class="btn btn-sm btn-cyber" <?= $index === count($commands) - 1 ? 'disabled' : '' ?>>Down</button>
                    </form>
                </td>
                <td>
                    <form method="POST" action="" style="display:inline;">
                        <input type="hidden" name="set" value="<?= htmlspecialchars($dataset) ?>">
                        <button type="submit" name="deleteCommand" value="<?= htmlspecialchars($command['title']) ?>" class="btn btn-sm btn-cyber">Delete</button>
                    </form>
                    <button class="btn btn-sm btn-accent" onclick='editCommand(<?= htmlspecialchars(json_encode($command, JSON_HEX_APOS | JSON_HEX_QUOT), ENT_QUOTES, 'UTF-8') ?>)'>Edit</button>
                </td>
            </tr>
        <?php endforeach; ?>
        </tbody>
    </table>
    </div>
    </div>
</div>

<script>
    var ros = new ROSLIB.Ros({
        url: 'ws:' + window.location.hostname + ':9090'
    });

    ros.on('connection', function () {
        document.getElementById("status").innerHTML = "Connected";
    });

    ros.on('error', function () {
        document.getElementById("status").innerHTML = "Error";
    });

    ros.on('close', function () {
        document.getElementById("status").innerHTML = "Closed";
    });

    // Mostra il modulo di modifica
    function openEditModal() {
        document.getElementById("editModal").style.display = "block";
    }

    // Nasconde il modulo di modifica
    function closeEditModal() {
        document.getElementById("editModal").style.display = "none";
    }

    // Compila il modulo per modificare un comando esistente
    function editCommand(command) {
        document.getElementById("originalTitle").value = command.title || "";
        document.getElementById("title").value = command.title || "";
        document.getElementById("description").value = command.description || "";
        document.getElementById("startCommand").value = command.startCommand || "";
        document.getElementById("stopCommand").value = command.stopCommand || "";
        document.getElementById("voiceKeywords").value = (command.voiceKeywords || []).join(", ");
        document.getElementById("selected").checked = !!command.selected;
        document.getElementById("monitor").checked = !!command.monitor;
        openEditModal();
    }
</script>
</body>
</html>
