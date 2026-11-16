<?php
$groups = [
    'Voce e interazione' => [
        ['asr_tts_node_piper', 'marrtinorobot2_voice', 'ASR faster-whisper e sintesi vocale Piper.'],
        ['tts_node', 'marrtinorobot2_voice', 'Sintesi vocale dai messaggi ROS.'],
        ['tts_offline_node', 'marrtinorobot2_voice', 'Sintesi vocale offline alternativa.'],
        ['chatbot_bridge_node', 'marrtinorobot2_voice', 'Collega /ASR al chatbot EVA o N8N e pubblica la risposta vocale.'],
        ['robot_cmd_ros', 'marrtinorobot2_voice', 'Interpreta ed esegue comandi vocali ROS.'],
    ],
    'Movimento e attuatori' => [
        ['micro_ros_agent', 'micro_ros_agent', 'Collegamento seriale con il controllore della base mobile.'],
        ['ekf_filter_node', 'robot_localization', 'Fusione dei dati di odometria tramite filtro EKF.'],
        ['dynamixel_controller', 'marrtinorobot2_dynamixel', 'Controllo di testa, braccia e servomotori Dynamixel.'],
        ['gesture_node', 'marrtinorobot2_webinterface', 'Esegue gesti, espressioni e animazioni sociali.'],
        ['joy_node', 'joy', 'Acquisisce i comandi dal joystick.'],
        ['teleop_twist_joy', 'teleop_twist_joy', 'Converte il joystick in comandi di velocità.'],
    ],
    'LiDAR e percezione ostacoli' => [
        ['sllidar_node', 'sllidar_ros2', 'Driver per il LiDAR Slamtec C1.'],
        ['rplidar_composition', 'rplidar_ros', 'Driver composable per sensori RPLIDAR.'],
        ['LD06', 'ldlidar_stl_ros2', 'Driver per LiDAR LD06/LD19.'],
        ['ldlidar', 'ldlidar', 'Driver LiDAR alternativo configurato dal bringup.'],
        ['lidar_filter_180_node', 'marrtinorobot2_bringup', 'Filtra la scansione LiDAR nel settore utile al robot.'],
        ['pointcloud_to_laserscan', 'pointcloud_to_laserscan', 'Converte la nuvola di punti OAK-D in LaserScan.'],
        ['depthimage_to_laserscan', 'depthimage_to_laserscan', 'Converte immagini di profondità in LaserScan.'],
    ],
    'Visione e riconoscimento' => [
        ['usb_camera', 'v4l2_camera', 'Acquisizione della webcam USB.'],
        ['web_video_server', 'web_video_server', 'Espone i topic immagine come streaming web.'],
        ['rgb_node', 'depthai-ros', 'Flusso RGB della camera OAK-D.'],
        ['stereo_node', 'depthai-ros', 'Profondità stereo della camera OAK-D.'],
        ['rectify_color_node', 'image_proc', 'Rettifica le immagini a colori.'],
        ['image_grabber', 'marrtinorobot2_vision', 'Cattura immagini dai topic ROS.'],
        ['face_recognition_node', 'marrtinorobot2_vision', 'Riconoscimento facciale classico.'],
        ['face_recognition_and_tracking_node', 'marrtinorobot2_vision', 'Riconoscimento e inseguimento del volto.'],
        ['face_recognizer', 'marrtinorobot2_recognition', 'Registrazione e riconoscimento delle persone con InsightFace.'],
        ['person_follower', 'marrtinorobot2_recognition', 'Ricerca e avvicinamento controllato a una persona.'],
    ],
    'Mappatura e navigazione' => [
        ['cartographer_node', 'cartographer_ros', 'SLAM e costruzione della mappa.'],
        ['cartographer_occupancy_grid_node', 'cartographer_ros', 'Genera la griglia di occupazione da Cartographer.'],
        ['slam_toolbox', 'slam_toolbox', 'SLAM 2D e gestione delle mappe.'],
        ['map_server', 'nav2_map_server', 'Carica e pubblica una mappa salvata.'],
        ['amcl', 'nav2_amcl', 'Localizzazione del robot sulla mappa.'],
        ['bt_navigator', 'nav2_bt_navigator', 'Coordina le missioni di navigazione Nav2.'],
        ['controller_server', 'nav2_controller', 'Calcola e applica il controllo locale del percorso.'],
        ['planner_server', 'nav2_planner', 'Calcola il percorso globale.'],
        ['behavior_server', 'nav2_behaviors', 'Gestisce i comportamenti di recupero Nav2.'],
        ['waypoint_follower', 'nav2_waypoint_follower', 'Esegue percorsi composti da più punti.'],
        ['rtabmap', 'rtabmap_ros', 'SLAM visuale/RGB-D con RTAB-Map.'],
        ['rviz2', 'rviz2', 'Visualizzazione di robot, sensori, mappe e navigazione.'],
    ],
    'Web, diagnostica e infrastruttura' => [
        ['autostart', 'bringup', 'Avvia automaticamente i processi configurati.'],
        ['log_publisher', 'marrtinorobot2_webinterface', 'Esegue i comandi Bringup e pubblica stdout/stderr.'],
        ['command_executor', 'marrtinorobot2_webinterface', 'Esegue i comandi ricevuti dal pannello web.'],
        ['bridge_server', 'marrtinorobot2', 'Bridge ROS per il server comandi.'],
        ['node_counter', 'marrtinorobot2_webinterface', 'Conta e monitora i nodi ROS attivi.'],
        ['robot_state_publisher', 'robot_state_publisher', 'Pubblica le trasformazioni del modello URDF.'],
        ['joint_state_publisher', 'joint_state_publisher', 'Pubblica lo stato dei giunti del modello.'],
        ['rosbridge_websocket', 'rosbridge_server', 'Connessione WebSocket tra interfaccia web e ROS 2.'],
        ['vizanti_server', 'vizanti_server', 'Interfaccia web per visualizzazione e navigazione ROS.'],
    ],
];
$nodeCount = array_sum(array_map('count', $groups));
?>
<!DOCTYPE html>
<html lang="it">
<head>
  <meta charset="utf-8">
  <title>Help - Nodi del robot</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
  <link rel="stylesheet" href="../bootstrap/css/bootstrap.min.css">
  <script src="../js/jquery-3.4.1.min.js"></script>
  <script src="../bootstrap/js/bootstrap.min.js"></script>
  <script src="../js/roslib.min.js"></script>
  <style>
    body { background: #f5f6fa; }
    .nodes-shell { max-width: 1200px; padding-top: 28px; padding-bottom: 50px; }
    .summary, .node-group { border: 0; border-radius: 10px; box-shadow: 0 8px 28px rgba(20, 25, 40, .09); }
    .group-title { color: rgb(157, 32, 104); font-weight: 700; }
    .node-name { color: #172033; font-family: monospace; font-weight: 700; }
    .package-name { white-space: nowrap; color: #526079; font-family: monospace; font-size: .88rem; }
    .node-row[hidden], .node-group[hidden] { display: none !important; }
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
  <main class="container nodes-shell">
    <h1>Nodi del robot</h1>
    <p class="lead text-muted">Catalogo dei <?= $nodeCount ?> nodi ROS 2 disponibili nel software MARRtino.</p>
    <div class="card summary mb-4"><div class="card-body">
      <label for="nodeSearch" class="font-weight-bold">Cerca per nodo, pacchetto o funzione</label>
      <input id="nodeSearch" class="form-control" type="search" placeholder="Esempio: lidar, voce, navigazione">
      <small class="text-muted">Alcuni nodi sono alternative hardware o software e non vengono avviati contemporaneamente.</small>
    </div></div>

    <?php foreach ($groups as $title => $nodes): ?>
      <section class="card node-group mb-4">
        <div class="card-header bg-white"><h2 class="h5 group-title mb-0"><?= htmlspecialchars($title, ENT_QUOTES, 'UTF-8') ?></h2></div>
        <div class="table-responsive"><table class="table mb-0">
          <thead><tr><th>Nodo ROS</th><th>Pacchetto</th><th>Funzione</th></tr></thead>
          <tbody>
          <?php foreach ($nodes as $node): ?>
            <tr class="node-row">
              <td class="node-name">/<?= htmlspecialchars($node[0], ENT_QUOTES, 'UTF-8') ?></td>
              <td class="package-name"><?= htmlspecialchars($node[1], ENT_QUOTES, 'UTF-8') ?></td>
              <td><?= htmlspecialchars($node[2], ENT_QUOTES, 'UTF-8') ?></td>
            </tr>
          <?php endforeach; ?>
          </tbody>
        </table></div>
      </section>
    <?php endforeach; ?>
  </main>
  <script>
    document.getElementById('nodeSearch').addEventListener('input', function () {
      var query = this.value.toLocaleLowerCase('it').trim();
      document.querySelectorAll('.node-group').forEach(function (group) {
        var visible = 0;
        group.querySelectorAll('.node-row').forEach(function (row) {
          var matches = !query || row.textContent.toLocaleLowerCase('it').indexOf(query) !== -1;
          row.hidden = !matches;
          if (matches) visible++;
        });
        group.hidden = visible === 0;
      });
    });
  </script>
</body>
</html>
