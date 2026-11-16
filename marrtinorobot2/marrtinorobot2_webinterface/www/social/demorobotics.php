<!DOCTYPE html>
<html>

<head>
  <meta charset="utf-8" />
  <title>MARRtino Face Recognition</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">

  <link rel="stylesheet" href="../bootstrap/css/bootstrap.min.css">
  <script src="js/jquery-3.4.1.min.js"></script>
  <script src="../bootstrap/js/bootstrap.min.js"></script>
  <script type="text/javascript" src="js/eventemitter2.min.js"></script>
  <script type="text/javascript" src="js/roslib.min.js"></script>

  <style>
    body {
      background: #f4f6f8;
      color: #1f2933;
    }

    .top-band {
      background: #111827;
      color: #fff;
      padding: 16px 0;
      margin-bottom: 18px;
    }

    .panel {
      background: #fff;
      border: 1px solid #d7dee8;
      border-radius: 6px;
      padding: 16px;
      margin-bottom: 16px;
    }

    .video-frame {
      width: 100%;
      max-width: 760px;
      aspect-ratio: 4 / 3;
      object-fit: contain;
      background: #0b0f14;
      border: 1px solid #283445;
      border-radius: 4px;
    }

    .status-pill {
      display: inline-block;
      min-width: 96px;
      padding: 4px 10px;
      border-radius: 999px;
      background: #6c757d;
      color: #fff;
      text-align: center;
      font-weight: 600;
    }

    .status-ok {
      background: #198754;
    }

    .status-warn {
      background: #fd7e14;
    }

    .status-bad {
      background: #dc3545;
    }

    .metric {
      font-size: 2rem;
      font-weight: 700;
      line-height: 1.1;
    }

    .enroll-guide {
      padding: 14px;
      margin-bottom: 14px;
      border: 2px solid #fd7e14;
      border-radius: 6px;
      background: #fff7ed;
      text-align: center;
    }

    .enroll-guide strong {
      display: block;
      font-size: 1.25rem;
    }

    .log-box {
      height: 180px;
      overflow: auto;
      background: #0b0f14;
      color: #d6e4ff;
      border-radius: 4px;
      padding: 10px;
      font-family: monospace;
      font-size: 0.9rem;
      white-space: pre-wrap;
    }
  </style>
</head>

<body>
  <?php include "../nav.php" ?>

  <div class="top-band">
    <div class="container-fluid">
      <h2 class="mb-1">Riconoscimento persona</h2>
      <div>Connection status: <span id="rosStatus" class="status-pill">Closed</span></div>
    </div>
  </div>

  <div class="container-fluid">
    <div class="row">
      <div class="col-lg-7">
        <div class="panel text-center">
          <img id="video" class="video-frame" src="" alt="camera stream">
        </div>
      </div>

      <div class="col-lg-5">
        <div class="panel">
          <h4>Addestramento</h4>
          <div id="enrollGuide" class="enroll-guide">
            <strong>In attesa</strong>
            <span id="enrollInstruction">Inserisci il nome e avvia la memorizzazione.</span>
            <div class="progress mt-2">
              <div id="enrollProgress" class="progress-bar bg-warning" style="width: 0%">0/7</div>
            </div>
          </div>
          <div class="form-group">
            <label for="personName">Nome persona</label>
            <input id="personName" class="form-control" type="text" placeholder="Es. Fabio">
          </div>
          <div class="form-group">
            <label for="sampleCount">Campioni volto</label>
            <input id="sampleCount" class="form-control" type="number" min="5" max="20" value="7">
          </div>
          <button class="btn btn-danger btn-block" onclick="startEnrollment()">Memorizza volto</button>
          <button class="btn btn-outline-secondary btn-block" onclick="cancelEnrollment()">Annulla addestramento</button>
        </div>

        <div class="panel">
          <h4>Test riconoscimento</h4>
          <div class="row">
            <div class="col-7">
              <div>Persona riconosciuta</div>
              <div id="recognizedName" class="metric">-</div>
            </div>
            <div class="col-5">
              <div>Similarita'</div>
              <div id="similarity" class="metric">0.00</div>
            </div>
          </div>
          <button class="btn btn-primary btn-block mt-3" onclick="requestStatus()">Test / Stato nodo</button>
        </div>

        <div class="panel">
          <h4>Database volti</h4>
          <p id="dbInfo" class="mb-2">Nessuna informazione disponibile.</p>
          <button class="btn btn-outline-primary btn-sm" onclick="reloadDb()">Ricarica DB</button>
          <button class="btn btn-outline-danger btn-sm" onclick="forgetPerson()">Cancella nome</button>
        </div>
      </div>
    </div>

    <div class="row">
      <div class="col-12">
        <div class="panel">
          <h4>Log</h4>
          <div id="log" class="log-box"></div>
        </div>
      </div>
    </div>
  </div>

  <script>
    var ros = new ROSLIB.Ros({
      url: 'ws:' + window.location.hostname + ':9090'
    });

    var commandTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/face_recognizer/command',
      messageType: 'std_msgs/String'
    });

    var statusTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/face_recognizer/status',
      messageType: 'std_msgs/String'
    });

    var resultTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/recognized_face',
      messageType: 'std_msgs/String'
    });

    function setRosStatus(text, cssClass) {
      var el = document.getElementById('rosStatus');
      el.className = 'status-pill ' + cssClass;
      el.innerHTML = text;
    }

    function addLog(text) {
      var now = new Date().toLocaleTimeString();
      var log = document.getElementById('log');
      log.textContent = '[' + now + '] ' + text + '\n' + log.textContent;
    }

    function publishCommand(command) {
      commandTopic.publish(new ROSLIB.Message({
        data: JSON.stringify(command)
      }));
      addLog('Comando inviato: ' + JSON.stringify(command));
    }

    function startEnrollment() {
      var name = document.getElementById('personName').value.trim();
      var samples = parseInt(document.getElementById('sampleCount').value || '5', 10);
      if (!name) {
        addLog('Inserisci il nome della persona prima di addestrare.');
        return;
      }
      publishCommand({ action: 'enroll', name: name, samples: samples });
    }

    function cancelEnrollment() {
      publishCommand({ action: 'cancel' });
    }

    function requestStatus() {
      publishCommand({ action: 'status' });
    }

    function reloadDb() {
      publishCommand({ action: 'reload' });
    }

    function forgetPerson() {
      var name = document.getElementById('personName').value.trim();
      if (!name) {
        addLog('Inserisci il nome da cancellare.');
        return;
      }
      publishCommand({ action: 'forget', name: name });
    }

    function updateDbInfo(msg) {
      var names = msg.known_names || [];
      var text = 'Campioni: ' + (msg.known_count || 0);
      if (names.length) {
        text += ' - Persone: ' + names.join(', ');
      }
      document.getElementById('dbInfo').textContent = text;
    }

    function updateEnrollment(msg) {
      var guide = document.getElementById('enrollGuide');
      var instruction = document.getElementById('enrollInstruction');
      var progress = document.getElementById('enrollProgress');
      var collected = Number(msg.collected || 0);
      var samples = Number(msg.samples || 7);
      var percent = Math.min(100, Math.round(collected * 100 / samples));

      if (msg.state === 'enrolling') {
        guide.querySelector('strong').textContent = 'Campione ' + (collected + 1) + ' di ' + samples;
        instruction.textContent = msg.pose || msg.message;
      } else if (msg.state === 'error') {
        guide.querySelector('strong').textContent = 'Acquisizione non riuscita';
        instruction.textContent = msg.message;
      } else if (collected) {
        guide.querySelector('strong').textContent = 'Memorizzazione completata';
        instruction.textContent = msg.message;
        percent = 100;
      }
      progress.style.width = percent + '%';
      progress.textContent = collected + '/' + samples;
    }

    ros.on('connection', function () {
      setRosStatus('Connected', 'status-ok');
      addLog('Connesso a rosbridge.');
      requestStatus();
    });

    ros.on('error', function () {
      setRosStatus('Error', 'status-bad');
      addLog('Errore connessione rosbridge.');
    });

    ros.on('close', function () {
      setRosStatus('Closed', 'status-warn');
      addLog('Connessione rosbridge chiusa.');
    });

    statusTopic.subscribe(function (m) {
      try {
        var msg = JSON.parse(m.data);
        addLog(msg.message || m.data);
        updateDbInfo(msg);
        updateEnrollment(msg);
      } catch (err) {
        addLog(m.data);
      }
    });

    resultTopic.subscribe(function (m) {
      try {
        var msg = JSON.parse(m.data);
        document.getElementById('recognizedName').textContent = msg.name || '-';
        document.getElementById('similarity').textContent = Number(msg.similarity || 0).toFixed(2);
      } catch (err) {
        addLog('Risultato non valido: ' + m.data);
      }
    });

    window.onload = function () {
      document.getElementById('video').src =
        'http://' + window.location.hostname + ':29090/stream?topic=/camera/image_raw&type=mjpeg&quality=65';
    };
  </script>
</body>

</html>
