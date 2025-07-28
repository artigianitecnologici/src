#!/usr/bin/env python3
from flask import Flask
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ~/src/marrtinorobot2/marrtinorobot2_webinterface/marrtinorobot2_webinterface/robot_cmd_ros.py import RobotCmdROS  # Importa la libreria


# === Comandi fisici reali ===
def forward(n):
    print(f"Robot: avanti di {n}")
    # qui il vero codice per muovere i motori

def right(n):
    print(f"Robot: destra di {n}")
    # codice per ruotare destra
    #robot.right(30)

def alza_braccio_destro():
    print("Robot: alzo braccio destro")
    # codice servo destro

# === Inizializzazione ROS e Flask ===
rclpy.init()
robot = RobotCmdRos()
node = rclpy.create_node('server_flask_ros')
pub = node.create_publisher(String, '/comando_robot', 10)
app = Flask(__name__)

@app.route('/<comando>', methods=['GET'])
def ricevi_comando(comando):
    print(f"Ricevuto comando HTTP: {comando}")

    # Esecuzione reale (non solo ROS)
    if comando == "avanti":
        forward(1)
    elif comando == "destra":
        robot.right(30)
    elif comando == "alza_braccio_destro":
        alza_braccio_destro()
    else:
        print("Comando non riconosciuto.")

    # Facoltativo: pubblica anche su ROS
    msg = String()
    msg.data = comando
    pub.publish(msg)

    return f"Comando {comando} eseguito\n"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
