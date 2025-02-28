<!DOCTYPE html>
<html>

<head>
  <meta charset="utf-8" />
  <title>MARRtino Social Interface</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
  
  <!-- Bootstrap CSS -->
  <link rel="stylesheet" href="bootstrap/css/bootstrap.min.css">
  <script src="js/jquery-3.4.1.min.js"></script>
  <script src="bootstrap/js/bootstrap.min.js"></script>
  <script type="text/javascript" src="js/roslib.min.js"></script>
  <script type="text/javascript" src="js/eventemitter2.min.js"></script>

  <script type="text/javascript">
    var ros = new ROSLIB.Ros({
      url: 'ws:' + window.location.hostname + ':9090'
    });

    ros.on('connection', function () {
      document.getElementById("status").innerHTML = "Connected";
      console.log("Connected to websocket server.");
    });

    ros.on('error', function (error) {
      document.getElementById("status").innerHTML = "Error: " + error;
      console.log("Error connecting to websocket server:", error);
    });

    ros.on('close', function () {
      document.getElementById("status").innerHTML = "Closed";
      console.log("Connection to websocket server closed.");
    });

    // Define the topic to send commands
    var commandTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/shell_command',
      messageType: 'std_msgs/String'
    });

    // Function to send commands
    function sendCommand(command) {
      var commandMessage = new ROSLIB.Message({
        data: `echo '${command}' | netcat -w 1 localhost 9236`
      });
      commandTopic.publish(commandMessage);
      console.log("Sent command: " + command);
    }

    // Define the topic to receive output
    var outputTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/command_output',
      messageType: 'std_msgs/String'
    });

    // Function to receive and display the output
    outputTopic.subscribe(function(message) {
      document.getElementById('output').textContent = message.data;
    });

    var txt_log = new ROSLIB.Topic({
      ros: ros,
      name: '/log_msg',
      messageType: 'std_msgs/String'
    });

    txt_log.subscribe(function (m) {
      document.getElementById("log_msg").innerHTML += m.data + "\n";  // Append new messages
    });

    var txt_logerr = new ROSLIB.Topic({
      ros: ros,
      name: '/logerr_msg',
      messageType: 'std_msgs/String'
    });

    txt_logerr.subscribe(function (m) {
      document.getElementById("logerr_msg").innerHTML += m.data + "\n";  // Append new error messages
    });

    var txt_nodes = new ROSLIB.Topic({
      ros: ros,
      name: '/nodes_list',
      messageType: 'std_msgs/String'
    });

    txt_nodes.subscribe(function (m) {
      var formattedNodes = m.data.replace(/,/g, '\n');  // Sostituisci le virgole con ritorni a capo
      document.getElementById("nodes_msg").innerHTML = formattedNodes + "\n";  // Visualizza la lista con ritorni a capo
    });


    // Function to execute selected commands one by one
    function executeSelectedCommands() {
      var commands = [];

      // Check which checkboxes are selected
      if (document.getElementById('bringup_checkbox').checked) {
        commands.push('robot_start');
      }
      if (document.getElementById('microros_checkbox').checked) {
        commands.push('microros_start');
      }
      if (document.getElementById('webcam_checkbox').checked) {
        commands.push('webcam_start');
      }
      if (document.getElementById('tts_checkbox').checked) {
        commands.push('tts_start');
      }
      if (document.getElementById('voicerecognition_checkbox').checked) {
        commands.push('voicerecognition_start');
      }
      if (document.getElementById('pantilt_checkbox').checked) {
        commands.push('pantilt_start');
      }
      if (document.getElementById('facetracker_checkbox').checked) {
        commands.push('facetracker_start');
      }
      if (document.getElementById('slam_checkbox').checked) {
        commands.push('slam_start');
      }
      if (document.getElementById('navigation_checkbox').checked) {
        commands.push('navigation_start');
      }

      // Execute commands one by one
      function executeNextCommand() {
        if (commands.length === 0) return;

        var nextCommand = commands.shift();
        sendCommand(nextCommand);

        // Wait a few seconds before executing the next command
        setTimeout(executeNextCommand, 2000); // Adjust the delay as needed
      }

      executeNextCommand();
    }
  </script>
</head>

<body>

  <?php include "nav.php" ?>

  <div class="container-fluid">
    <div class="row">
      <!-- Left column with buttons -->
      <div class="col-md-6">
  </br>
      <h5>Command:</h5>
        <table class="table table-bordered">
          <tr height="40">
            <td width="280">Bringup</td>
            <td width="80" align="center">
              <input type="checkbox" id="bringup_checkbox">
            </td>
            <td>
              <button onclick="sendCommand('robot_start')" class="btn btn-outline-primary">Start</button>
              <button onclick="sendCommand('robot_kill')" class="btn btn-outline-primary">Stop</button>
            </td>
          </tr>
          <tr height="40">
            <td width="280">MicroROS</td>
            <td width="80" align="center">
              <input type="checkbox" id="microros_checkbox">
            </td>
            <td>
              <button onclick="sendCommand('microros_start')" class="btn btn-outline-primary">Start</button>
              <button onclick="sendCommand('microros_kill')" class="btn btn-outline-primary">Stop</button>
            </td>
          </tr>
          <tr height="40">
            <td width="280">WebCam</td>
            <td width="80" align="center">
              <input type="checkbox" id="webcam_checkbox">
            </td>
            <td>
              <button class="btn btn-outline-primary" onclick="sendCommand('webcam_start')">Start</button>
              <button class="btn btn-outline-primary" onclick="sendCommand('webcam_stop')">Stop</button>
            </td>
          </tr>
          <tr height="40">
            <td width="280">TTS</td>
            <td width="80" align="center">
              <input type="checkbox" id="tts_checkbox">
            </td>
            <td>
              <button class="btn btn-outline-primary" onclick="sendCommand('tts_start')">Start</button>
              <button class="btn btn-outline-primary" onclick="sendCommand('tts_stop')">Stop</button>
            </td>
          </tr>
          <tr height="40">
            <td width="280">Voice Recognition</td>
            <td width="80" align="center">
              <input type="checkbox" id="voicerecognition_checkbox">
            </td>
            <td>
              <button class="btn btn-outline-primary" onclick="sendCommand('voicerecognition_start')">Start</button>
              <button class="btn btn-outline-primary" onclick="sendCommand('voicerecognition_stop')">Stop</button>
            </td>
          </tr>
          <tr height="40">
            <td width="280">Pan Tilt</td>
            <td width="80" align="center">
              <input type="checkbox" id="pantilt_checkbox">
            </td>
            <td>
              <button onclick="sendCommand('pantilt_start')" class="btn btn-outline-primary">Start</button>
              <button onclick="sendCommand('pantilt_kill')" class="btn btn-outline-primary">Stop</button>
            </td>
          </tr>
          <tr height="40">
            <td width="280">Face Tracker</td>
            <td width="80" align="center">
              <input type="checkbox" id="facetracker_checkbox">
            </td>
            <td>
              <button onclick="sendCommand('facetracker_start')" class="btn btn-outline-primary">Start</button>
              <button onclick="sendCommand('facetracker_kill')" class="btn btn-outline-primary">Stop</button>
            </td>
          </tr>
          <tr height="40">
            <td width="280">Slam</td>
            <td width="80" align="center">
              <input type="checkbox" id="slam_checkbox">
            </td>
            <td>
              <button onclick="sendCommand('slam_start')" class="btn btn-outline-primary">Start</button>
              <button onclick="sendCommand('slam_kill')" class="btn btn-outline-primary">Stop</button>
            </td>
          </tr>
          <tr height="40">
            <td width="280">Navigation</td>
            <td width="80" align="center">
              <input type="checkbox" id="navigation_checkbox">
            </td>
            <td>
              <button onclick="sendCommand('navigation_start')" class="btn btn-outline-primary">Start</button>
              <button onclick="sendCommand('navigation_kill')" class="btn btn-outline-primary">Stop</button>
            </td>
          </tr>
        </table>

        <button onclick="executeSelectedCommands()" class="btn btn-success">Execute Selected Commands</button>
      </div>

      <!-- Right column with logs -->
      <div class="col-md-6">
        <!-- Log message display -->
        <div class="mt-4">
          <h5>Log Messages:</h5>
          <pre id="log_msg" class="p-3 mb-2 bg-light border rounded" style="height: 400px; overflow-y: scroll;">Log messages will appear here...</pre>
        </div>

        <!-- Error log message display -->
        <div class="mt-4">
          <h5>Error Log Messages:</h5>
          <pre id="logerr_msg" class="p-3 mb-2 bg-light border rounded" style="height: 150px; overflow-y: scroll;">Error log messages will appear here...</pre>
        </div>
        <div class="mt-4">
          <h5>Nodes:</h5>
          <pre id="nodes_msg" class="p-3 mb-2 bg-light border rounded" style="height: 150px; overflow-y: scroll;">Nodes will appear here...</pre>
        </div>
        <!-- Output message display -->
        <div class="mt-4">
          <h5>Command Output:</h5>
          <pre id="output" class="p-3 mb-2 bg-light border rounded">Output will appear here...</pre>
        </div>
      </div>
    </div>
  </div>

</body>

</html>
