
# Integrazione di una telecamera OAK-D con ROS 2 Humble per rilevamento ostacoli

## 1. Prerequisiti

Assicurati di avere:
- **ROS 2 Humble** installato.
- I driver e i pacchetti della telecamera **OAK-D** (DepthAI).
- Un robot configurato con il pacchetto di navigazione **Nav2**.
- Una workspace ROS 2 configurata e funzionante.

---

## 2. Installazione dei driver DepthAI

1. Clona il repository **depthai-ros** nella workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/luxonis/depthai-ros.git
   git clone https://github.com/luxonis/depthai-ros-examples.git
   ```

2. Installa le dipendenze:
   ```bash
   sudo apt update
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Compila la workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select depthai_ros depthai_ros_examples
   source install/setup.bash
   ```

---

## 3. Configurazione della telecamera OAK-D

1. Avvia la telecamera OAK-D con un esempio:
   ```bash
   ros2 launch depthai_ros_examples stereo_inertial_node.launch.py
   ```

2. I topic principali pubblicati sono:
   - `/stereo/depth`: Immagine di profondità.
   - `/stereo/points`: Point cloud 3D.
   - `/rgb/image_raw`: Immagini RGB.

3. Verifica i topic pubblicati:
   ```bash
   ros2 topic list
   ```

---

## 4. Integrazione con il robot

### A. Rilevamento degli ostacoli con LaserScan

1. **Installa il pacchetto per la conversione da PointCloud2 a LaserScan:**
   ```bash
   sudo apt install ros-humble-pointcloud-to-laserscan
   ```

2. **Esegui la conversione:**
   ```bash
   ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
       --ros-args -p input_cloud:=/stereo/points -p output_scan:=/scan
   ```

3. Configura Nav2 per utilizzare il topic `/scan` nel file di configurazione di Nav2 (`nav2_params.yaml`):
   ```yaml
   obstacle_layer:
     plugins: ["voxel_layer"]
     voxel_layer:
       enabled: true
       topic: "/scan"
       observation_sources: laser
   ```

---

## 5. Lancio e Test

1. **Avvia la telecamera:**
   ```bash
   ros2 launch depthai_ros_examples stereo_inertial_node.launch.py
   ```

2. **Avvia Nav2:**
   ```bash
   ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false
   ```

3. Verifica che il robot eviti ostacoli muovendo oggetti davanti alla telecamera.

---

## 6. Debug e Ottimizzazione

- Usa **RViz2** per visualizzare i dati della point cloud e del LaserScan.
- Ottimizza i parametri della telecamera (ad esempio, range di profondità).
- Modifica i parametri di Nav2 per migliorare le prestazioni.

---

Se hai bisogno di ulteriori dettagli, puoi personalizzare questo file per adattarlo al tuo sistema specifico!
