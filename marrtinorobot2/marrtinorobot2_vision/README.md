# MARRtino Robot 2 Vision

Questo pacchetto raccoglie i nodi e i launch per usare webcam, AprilTag, face tracking e camera OAK-D Lite sul robot MARRtino. La parte principale per la navigazione usa la OAK-D Lite come sensore di profondita: la point cloud viene convertita in un `LaserScan` virtuale sul topic `/scan_oak`, poi Nav2 la usa nelle costmap per rilevare ostacoli frontali.

## Prerequisiti

Assicurati di avere un workspace ROS 2 gia configurato e compilabile:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/marrtinorobot2_ws/install/setup.bash
```

Pacchetti ROS consigliati:

```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-depthai-ros \
  ros-$ROS_DISTRO-pointcloud-to-laserscan \
  ros-$ROS_DISTRO-image-proc \
  ros-$ROS_DISTRO-v4l2-camera \
  ros-$ROS_DISTRO-apriltag-ros
```

Per verificare che la OAK-D Lite sia vista via USB:

```bash
lsusb | grep -i "03e7"
```

Se tutto e corretto, dovresti vedere un dispositivo Intel Movidius MyriadX.

## Build Del Pacchetto

Dal workspace:

```bash
cd ~/marrtinorobot2_ws
colcon build --packages-select marrtinorobot2_vision marrtinorobot2_navigation --symlink-install
source install/setup.bash
```

## OAK-D Lite Per Ostacoli

Avvia la OAK-D Lite e la conversione pointcloud-to-laserscan:

```bash
ros2 launch marrtinorobot2_vision oak_obstacles.launch.py
```

Il launch avvia:

- il driver DepthAI per la camera OAK-D Lite;
- il nodo `pointcloud_to_laserscan_node`;
- la conversione da `/oak/stereo/points` a `/scan_oak`.

Verifica i topic:

```bash
ros2 topic list | grep oak
ros2 topic hz /oak/stereo/points
ros2 topic hz /scan_oak
```

Se la point cloud della camera esce su un topic diverso, puoi cambiarlo:

```bash
ros2 launch marrtinorobot2_vision oak_obstacles.launch.py input_cloud:=/stereo/points
```

Puoi anche cambiare il topic di uscita:

```bash
ros2 launch marrtinorobot2_vision oak_obstacles.launch.py output_scan:=/scan_oak
```

## Uso Con Nav2

La configurazione `marrtinorobot2_navigation/config/navigation.yaml` e gia predisposta per leggere:

- `/scan`: lidar principale;
- `/base/scan`: eventuale scan base;
- `/scan_oak`: ostacoli rilevati dalla OAK-D Lite;
- `/oak/stereo/points`: point cloud 3D della OAK.

Avvio tipico:

```bash
ros2 launch marrtinorobot2_vision oak_obstacles.launch.py
ros2 launch marrtinorobot2_navigation navigation.launch.py
```

In RViz controlla:

- `LaserScan` su `/scan_oak`;
- `PointCloud2` su `/oak/stereo/points`;
- local costmap e global costmap di Nav2.

## Parametri OAK Laser Virtuale

La configurazione del laser virtuale si trova in:

```text
config/oak_pointcloud_to_laserscan.yaml
```

Parametri principali:

- `target_frame: base_link`: trasforma lo scan nel frame del robot.
- `min_height: 0.05`: ignora il pavimento.
- `max_height: 0.60`: considera ostacoli bassi e medi davanti al robot.
- `angle_min` / `angle_max`: campo visivo usato per lo scan frontale.
- `range_min: 0.25`: ignora punti troppo vicini alla camera.
- `range_max: 3.50`: distanza massima utile per ostacoli.

Se la costmap vede troppo rumore dal pavimento, aumenta `min_height`. Se ignora ostacoli bassi, abbassa `min_height` o alza `max_height`.

## Webcam USB

Per verificare le webcam disponibili:

```bash
v4l2-ctl --list-devices
```

Avvio manuale webcam:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

## Streaming Webcam E Saluto Di Fabo

Il launch `web_video_server.launch.py` avvia:

- camera USB con `v4l2_camera`;
- streaming MJPEG con `web_video_server` sulla porta `29090`;
- riconoscimento volto su `/camera/image_raw`;
- saluto TTS su `/tts_input` quando riconosce Fabo.

Prima di usarlo, aggiungi 3-8 foto del viso in:

```text
marrtinorobot2_vision/config/faces/fabo/
```

Poi ricompila:

```bash
cd ~/marrtinorobot2_ws
colcon build --packages-select marrtinorobot2_vision --symlink-install
source install/setup.bash
```

Assicurati anche che il nodo TTS sia attivo, perche il saluto viene pubblicato su `/tts_input`.

Avvio consigliato:

```bash
ros2 launch marrtinorobot2_vision web_video_server.launch.py
```

Se la webcam non e `/dev/webcam`:

```bash
ros2 launch marrtinorobot2_vision web_video_server.launch.py video_device:=/dev/video0
```

Puoi ottimizzare lo streaming abbassando risoluzione, FPS o qualita:

```bash
ros2 launch marrtinorobot2_vision web_video_server.launch.py image_width:=640 image_height:=480 fps:=15 stream_quality:=65
```

Stream originale:

```text
http://ROBOT_IP:29090/stream?topic=/camera/image_raw
```

Stream con riquadro riconoscimento:

```text
http://ROBOT_IP:29090/stream?topic=/face_recognition/image
```

## AprilTag

Avvio camera + AprilTag, adattando `camera_name` e `image_topic` ai topic reali:

```bash
ros2 launch apriltag_ros tag_realsense.launch.py camera_name:=/camera image_topic:=image_raw
```

Per generare marker AprilTag 36h11 puoi usare:

```text
https://chev.me/arucogen/
```

## Acquisizione Immagine

Se il nodo `getimage` e attivo, puoi richiedere uno scatto con:

```bash
ros2 topic pub -1 /getimage std_msgs/msg/String '{data: "get"}'
```

## Debug Rapido

Controlla che la camera pubblichi:

```bash
ros2 topic list
ros2 topic hz /oak/stereo/points
ros2 topic hz /scan_oak
```

Controlla i frame TF:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link oak_rgb_camera_frame
```

Se Nav2 non usa gli ostacoli OAK:

```bash
ros2 param get /local_costmap/local_costmap obstacle_layer.observation_sources
ros2 param get /global_costmap/global_costmap obstacle_layer.observation_sources
```

Verifica che tra le sorgenti sia presente `scan_oak`.

## Note Operative

La OAK-D Lite vede bene davanti al robot, ma non copre 360 gradi. Per navigazione affidabile e meglio usarla insieme al lidar principale. In questa configurazione il lidar rimane su `/scan`, mentre la OAK aggiunge ostacoli frontali su `/scan_oak`.
