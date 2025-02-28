<?php
// Percorso del file JSON
$jsonFile = 'commands.json';

// Leggi il file JSON
$commands = json_decode(file_get_contents($jsonFile), true);

// Se il modulo viene inviato, aggiorna il file JSON
if ($_SERVER['REQUEST_METHOD'] === 'POST') {
    if (isset($_POST['saveCommand'])) {
        // Salva o aggiorna un comando esistente
        $title = $_POST['title'];
        $description = $_POST['description'];
        $startCommand = $_POST['startCommand'];
        $stopCommand = $_POST['stopCommand'];
        $selected = isset($_POST['selected']) ? true : false;
        $monitor = isset($_POST['monitor']) ? true : false;

        // Cerca il comando nel JSON
        $found = false;
        foreach ($commands as &$command) {
            if ($command['title'] === $title) {
                $command['description'] = $description;
                $command['startCommand'] = $startCommand;
                $command['stopCommand'] = $stopCommand;
                $command['selected'] = $selected;
                $command['monitor'] = $monitor;
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
            ];
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
    file_put_contents($jsonFile, json_encode($commands, JSON_PRETTY_PRINT));
}
?>

<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8" />
    <title>Edit Commands</title>
    <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
    <link rel="stylesheet" href="../bootstrap/css/bootstrap.min.css">
</head>
<body>
<div class="container mt-4">
    <h1>Edit Commands</h1>

    <!-- Pulsante Aggiungi -->
    <button class="btn btn-primary mb-3" onclick="openEditModal()">Add Command</button>

    <!-- Modulo per aggiungere/modificare -->
    <div id="editModal" class="border rounded p-4 bg-light" style="display: none;">
        <form method="POST" action="">
            <input type="hidden" id="editMode" name="editMode" value="">
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
            <div class="form-check mb-3">
                <input type="checkbox" id="selected" name="selected" class="form-check-input">
                <label for="selected" class="form-check-label">Autostart</label>
            </div>
            <div class="form-check mb-3">
                <input type="checkbox" id="monitor" name="monitor" class="form-check-input">
                <label for="monitor" class="form-check-label">Monitor</label>
            </div>
            <button type="submit" name="saveCommand" class="btn btn-success">Save Command</button>
            <button type="button" class="btn btn-secondary" onclick="closeEditModal()">Cancel</button>
        </form>
    </div>

    <h2 class="mt-4">Existing Commands</h2>
    <table class="table table-bordered">
        <thead>
        <tr>
            <th>Title</th>
            <th>Description</th>
            <th>Start Command</th>
            <th>Stop Command</th>
            <th>Autostart</th>
            <th>Monitor</th>
            <th>Actions</th>
        </tr>
        </thead>
        <tbody>
        <?php foreach ($commands as $command): ?>
            <tr>
                <td><?= htmlspecialchars($command['title']) ?></td>
                <td><?= htmlspecialchars($command['description']) ?></td>
                <td><?= htmlspecialchars($command['startCommand']) ?></td>
                <td><?= htmlspecialchars($command['stopCommand']) ?></td>
                <td><?= $command['selected'] ? 'Yes' : 'No' ?></td>
                <td><?= $command['monitor'] ? 'Yes' : 'No' ?></td>
                <td>
                    <form method="POST" action="" style="display:inline;">
                        <button type="submit" name="deleteCommand" value="<?= htmlspecialchars($command['title']) ?>" class="btn btn-danger btn-sm">Delete</button>
                    </form>
                    <button class="btn btn-warning btn-sm" onclick="editCommand('<?= htmlspecialchars($command['title']) ?>', '<?= htmlspecialchars($command['description']) ?>', '<?= htmlspecialchars($command['startCommand']) ?>', '<?= htmlspecialchars($command['stopCommand']) ?>', <?= $command['selected'] ? 'true' : 'false' ?>, <?= $command['monitor'] ? 'true' : 'false' ?>)">Edit</button>
                </td>
            </tr>
        <?php endforeach; ?>
        </tbody>
    </table>
</div>

<script>
    // Mostra il modulo di modifica
    function openEditModal() {
        document.getElementById("editModal").style.display = "block";
    }

    // Nasconde il modulo di modifica
    function closeEditModal() {
        document.getElementById("editModal").style.display = "none";
    }

    // Compila il modulo per modificare un comando esistente
    function editCommand(title, description, startCommand, stopCommand, selected, monitor) {
        document.getElementById("title").value = title;
        document.getElementById("description").value = description;
        document.getElementById("startCommand").value = startCommand;
        document.getElementById("stopCommand").value = stopCommand;
        document.getElementById("selected").checked = selected;
        document.getElementById("monitor").checked = monitor;
        openEditModal();
    }
</script>
</body>
</html>
