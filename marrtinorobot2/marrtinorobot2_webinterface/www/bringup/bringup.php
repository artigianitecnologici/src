<?php
// Percorso del file JSON
$jsonFile = 'commands.json';

// Se il modulo viene inviato, salva i dati
if ($_SERVER['REQUEST_METHOD'] === 'POST') {
    $selectedOptions = isset($_POST['selectedOptions']) ? $_POST['selectedOptions'] : [];
    $monitoredOptions = isset($_POST['monitoredOptions']) ? $_POST['monitoredOptions'] : [];

    // Leggi il file JSON esistente
    $commands = json_decode(file_get_contents($jsonFile), true);

    // Aggiorna i comandi con lo stato selezionato e monitorato
    foreach ($commands as &$command) {
        $command['selected'] = in_array($command['title'], $selectedOptions);
        $command['monitor'] = in_array($command['title'], $monitoredOptions);
    }

    // Salva il file aggiornato
    file_put_contents($jsonFile, json_encode($commands, JSON_PRETTY_PRINT));
}

// Leggi i dati dei comandi dal file JSON
$commands = json_decode(file_get_contents($jsonFile), true);
?>

<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8" />
    <title>MARRtino ROS2 Bringup</title>
    <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
    <link rel="stylesheet" href="../bootstrap/css/bootstrap.min.css">
    <script src="../js/jquery-3.4.1.min.js"></script>
    <script src="../bootstrap/js/bootstrap.min.js"></script>
    <script type="text/javascript" src="../js/roslib.min.js"></script>
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
    <div class="container-fluid">
        <h1>MARRtino ROS2 Bringup</h1>
        <form method="POST" action="">
            <div class="row">
                <!-- Tabella con i comandi -->
                <div class="col-md-6">
                    <table class="table table-bordered">
                        <thead>
                            <tr>
                                <th>Command</th>
                                <th>Description</th>
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
                                    <td align="center">
                                        <input type="checkbox" name="selectedOptions[]" value="<?= htmlspecialchars($command['title']) ?>" <?= $command['selected'] ? 'checked' : '' ?>>
                                    </td>
                                    <td align="center">
                                        <input type="checkbox" name="monitoredOptions[]" value="<?= htmlspecialchars($command['title']) ?>" <?= $command['monitor'] ? 'checked' : '' ?>>
                                    </td>
                                    <td>
                                        <button type="button" onclick="sendCommand('<?= htmlspecialchars($command['startCommand']) ?>')" class="btn btn-outline-primary">Start</button>
                                        <button type="button" onclick="sendCommand('<?= htmlspecialchars($command['stopCommand']) ?>')" class="btn btn-outline-primary">Stop</button>
                                    </td>
                                </tr>
                            <?php endforeach; ?>
                        </tbody>
                    </table>
                    <button type="submit" class="btn btn-success">Save Preferences</button>
                     
                     <a href="edit_commands.php" class="btn btn-warning">Edit Commands</a>
         
                </div>

                <!-- Area log -->
                <div class="col-md-6">
                    <h5>Log Messages:</h5>
                    <pre id="log_msg" class="p-3 mb-2 bg-light border rounded" style="height: 400px; overflow-y: scroll;">Log messages will appear here...</pre>
                </div>
            </div>
        </form>
    </div>

    <script>
        // Configurazione ROS
        var ros = new ROSLIB.Ros({
            url: 'ws:' + window.location.hostname + ':9090'
        });

        ros.on('connection', function () {
            console.log('Connected to ROS websocket server.');
        });

        ros.on('error', function (error) {
            console.error('Error connecting to ROS websocket server:', error);
        });

        ros.on('close', function () {
            console.log('Connection to ROS websocket server closed.');
        });

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

        // Sottoscrizione ai log
        var txt_log = new ROSLIB.Topic({
            ros: ros,
            name: '/log_msg',
            messageType: 'std_msgs/String'
        });

        txt_log.subscribe(function (m) {
            document.getElementById("log_msg").innerHTML += m.data + "\n";  // Append new messages
        });
    </script>
</body>
</html>
