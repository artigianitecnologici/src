import os
import subprocess
import threading
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Classe TmuxSend aggiornata
class TmuxSend:
    def __init__(self, session_name, names):
        self.session_name = session_name
        self.names = names

    def cmd(self, index, command):
        try:
            print(f"Executing command '{command}' in panel {index} of session '{self.session_name}'")
            process = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print(f"Process started with PID: {process.pid}")
            return process  # Restituisce il processo
        except Exception as e:
            print(f"Error executing command '{command}': {e}")
            return None

    def Cc(self, index):
        print(f"Sending Ctrl+C to panel {index} of session '{self.session_name}'")


# Nodo ROS per i log
class LogPublisher(Node):
    def __init__(self):
        super().__init__('log_publisher')
        self.log_pub = self.create_publisher(String, 'log_msg', 10)

    def publish_log(self, message):
        log_msg = String()
        log_msg.data = message
        self.log_pub.publish(log_msg)
        self.get_logger().info(f'Published log: {message}')


# Funzione per monitorare il processo in background
def monitor_process(process, log_publisher):
    def read_output(pipe, log_type="INFO"):
        for line in iter(pipe.readline, b""):
            log_publisher.publish_log(f"{log_type}: {line.decode('utf-8').strip()}")

    stdout_thread = threading.Thread(target=read_output, args=(process.stdout, "INFO"))
    stderr_thread = threading.Thread(target=read_output, args=(process.stderr, "ERROR"))
    stdout_thread.start()
    stderr_thread.start()
    stdout_thread.join()
    stderr_thread.join()


# Esegue un comando
def execute_command(command, tmux, index, log_publisher):
    try:
        log_publisher.publish_log(f"Attempting to execute: {command}")
        process = tmux.cmd(index, command)
        if process:
            #threading.Thread(target=monitor_process, args=(process, log_publisher), daemon=True).start()
            log_publisher.publish_log(f"Command executed successfully: {command}. Process PID: {process.pid}")
            print(f"Command executed successfully: {command}. Process PID: {process.pid}")
            return process
        else:
            log_publisher.publish_log(f"Error executing command: {command}")
            print(f"Error executing command: {command}")
    except Exception as e:
        error_message = f"Exception occurred while executing '{command}': {str(e)}"
        log_publisher.publish_log(error_message)
        print(error_message)
    return None


# Funzione principale del server
def run_server(port, log_publisher):
    # Inizializza il socket TCP
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_address = ('', port)
    sock.bind(server_address)
    sock.listen(1)
    print(f"Server started on port {port}")
    log_publisher.publish_log(f"Server started on port {port}")

    panel_names = ['bringup', 'MicroROS', 'webcam', 'tts', 'rplidar','ldlidar','slam','explorer']
    tmux = TmuxSend('bringup', panel_names)

    processes = {}

    while True:
        connection, client_address = sock.accept()
        log_publisher.publish_log(f"Connection from {client_address}")
        data = connection.recv(320).decode('utf-8').strip()

        if data:
            pfolder = os.getenv('MARRTINOROBOT2_WS')
            if pfolder is None:
                log_publisher.publish_log("Error: MARRTINOROBOT2_WS environment variable not set.")
                continue

            if '_kill' in data:
                base_command = data.replace('_kill', '').replace('.sh', '')  # Normalizza il nome
                try:
                    index = panel_names.index(base_command)
                    if base_command in processes:
                        processes[base_command].terminate()
                        log_publisher.publish_log(f"Process for '{base_command}' terminated.")
                        del processes[base_command]
                    else:
                        log_publisher.publish_log(f"No running process for '{base_command}' to terminate.")
                except ValueError:
                    log_publisher.publish_log(f"Command '{base_command}' not found.")
            else:
                base_command = f"cd {pfolder} && " + './' + data
                try:
                    index = panel_names.index(data.replace('.sh', ''))  # Normalizza il nome
                    command_key = data.replace('.sh', '')  # Chiave per il dizionario dei processi
                    if command_key not in processes:
                        process = execute_command(base_command, tmux, index, log_publisher)
                        if process:
                            processes[command_key] = process
                    else:
                        log_publisher.publish_log(f"Command '{data}' is already running.")
                except ValueError:
                    log_publisher.publish_log(f"Command '{data}' not found in panel names.")


if __name__ == '__main__':
    rclpy.init()

    # Nodo ROS per i log
    log_publisher = LogPublisher()

    # Porta del server
    default_port = 9236
    log_publisher.publish_log(f"Server bringup on port '{default_port}' ")

    # Esegui il server
    server_thread = threading.Thread(target=run_server, args=(default_port, log_publisher))
    server_thread.start()

    try:
        rclpy.spin(log_publisher)
    except KeyboardInterrupt:
        print("Shutting down server.")
    finally:
        server_thread.join()
        log_publisher.destroy_node()
        rclpy.shutdown()
