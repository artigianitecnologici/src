import os
import json
import signal
import subprocess
import threading
import socket
import time
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Classe TmuxSend aggiornata
class TmuxSend:
    def __init__(self, session_name, names):
        self.session_name = session_name
        self.names = names

    def cmd(self, index, command, monitor=False):
        try:
            print(f"Executing command '{command}' in panel {index} of session '{self.session_name}'")
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            process = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE if monitor else subprocess.DEVNULL,
                # Errors must always reach the web UI.  "monitor" only controls
                # the usually much noisier stdout stream.
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                text=True,
                bufsize=1,
                env=env,
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
def monitor_process(process, title, log_publisher):
    ros_console_prefix = re.compile(
        r"^\s*\[(?:DEBUG|INFO|WARN(?:ING)?|ERROR|FATAL)\]\s*"
        r"(?:\[[0-9]+(?:\.[0-9]+)?\]\s*)?"
        r"\[[^\]]+\]:\s*",
        re.IGNORECASE,
    )

    def detect_level(text, stream):
        """Usa il livello nel messaggio, non il canale stdout/stderr."""
        levels = re.findall(r"\[(DEBUG|INFO|WARN(?:ING)?|ERROR|FATAL)\]", text, re.IGNORECASE)
        if levels:
            level = levels[-1].upper()
            if level == "WARNING":
                return "WARN"
            if level == "FATAL":
                return "ERROR"
            return level
        if re.search(r"\b(?:warning|warn)\b", text, re.IGNORECASE):
            return "WARN"
        if re.search(r"\b(?:error|exception|traceback|fatal|failed|fail)\b", text, re.IGNORECASE):
            return "ERROR"
        # ROS 2 e Vosk inviano anche i normali messaggi informativi su stderr.
        return "INFO"

    def read_output(pipe, stream="stdout"):
        if pipe is None:
            return
        for line in iter(pipe.readline, ""):
            text = line.rstrip()
            if text:
                level = detect_level(text, stream)
                # Il pannello aggiunge già servizio e livello. Rimuove quindi
                # l'eventuale prefisso console ROS (livello, timestamp e nodo)
                # per evitare righe duplicate e difficili da leggere.
                message = ros_console_prefix.sub("", text, count=1)
                message = re.sub(
                    r"^\[(?:DEBUG|INFO|WARN(?:ING)?|ERROR|FATAL)\]\s*",
                    "",
                    message,
                    count=1,
                    flags=re.IGNORECASE,
                )
                log_publisher.publish_log(f"[{title}] {level}: {message}")

    threads = []
    if process.stdout is not None:
        threads.append(threading.Thread(target=read_output, args=(process.stdout, "stdout"), daemon=True))
    if process.stderr is not None:
        threads.append(threading.Thread(target=read_output, args=(process.stderr, "stderr"), daemon=True))
    for thread in threads:
        thread.start()
    returncode = process.wait()
    for thread in threads:
        thread.join(timeout=1)
    level = "ERROR" if returncode else "INFO"
    log_publisher.publish_log(f"[{title}] {level}: exited with code {returncode}")


# Esegue un comando
def execute_command(command, tmux, index, log_publisher, title="", monitor=False):
    try:
        log_publisher.publish_log(f"Attempting to execute: {command}")
        process = tmux.cmd(index, command, monitor=monitor)
        if process:
            # Always monitor stderr and process termination.  With the Monitor
            # flag enabled stdout is captured as well.
            if process.stderr is not None or process.stdout is not None:
                if monitor:
                    log_publisher.publish_log(f"[{title}] monitor enabled: streaming stdout/stderr.")
                threading.Thread(
                    target=monitor_process,
                    args=(process, title or command, log_publisher),
                    daemon=True,
                ).start()
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


def commands_json_paths():
    candidates = [
        os.path.expanduser('~/src/marrtinorobot2/marrtinorobot2_webinterface/www/bringup/commands.json'),
        '/home/ubuntu/src/marrtinorobot2/marrtinorobot2_webinterface/www/bringup/commands.json',
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'www', 'bringup', 'commands.json')),
    ]
    for path in candidates:
        if os.path.exists(path):
            directory = os.path.dirname(path)
            return [
                os.path.join(directory, 'commands.json'),
                os.path.join(directory, 'navigation.json'),
            ]
    directory = os.path.dirname(candidates[0])
    return [os.path.join(directory, 'commands.json'), os.path.join(directory, 'navigation.json')]


def load_commands(log_publisher):
    commands = []
    for path in commands_json_paths():
        try:
            with open(path, 'r', encoding='utf-8') as file:
                dataset = json.load(file)
            if not isinstance(dataset, list):
                log_publisher.publish_log(f"Invalid commands file format: {path}")
                continue
            commands.extend(dataset)
        except Exception as exc:
            log_publisher.publish_log(f"Error loading commands from {path}: {exc}")
    return commands


def command_key(command_entry):
    return str(command_entry.get('title', '')).strip()


def build_command_maps(commands):
    start_map = {}
    stop_map = {}
    for entry in commands:
        key = command_key(entry)
        start = str(entry.get('startCommand', '')).strip()
        stop = str(entry.get('stopCommand', '')).strip()
        if key and start:
            start_map[start] = entry
        if key and stop:
            stop_map[stop] = entry
    return start_map, stop_map


def stop_process(process, title, log_publisher):
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait(timeout=2)
        log_publisher.publish_log(f"Process for '{title}' terminated.")
    except ProcessLookupError:
        log_publisher.publish_log(f"Process for '{title}' was already stopped.")
    except Exception as exc:
        log_publisher.publish_log(f"Error stopping '{title}': {exc}")


def start_command(entry, pfolder, tmux, index, processes, log_publisher):
    title = command_key(entry)
    start = str(entry.get('startCommand', '')).strip()
    monitor = bool(entry.get('monitor', False))
    if not title or not start:
        log_publisher.publish_log("Invalid command entry: missing title or startCommand.")
        return

    if title in processes and processes[title].poll() is None:
        log_publisher.publish_log(f"Command '{title}' is already running.")
        return

    command = f"cd {pfolder} && ./{start}"
    process = execute_command(command, tmux, index, log_publisher, title=title, monitor=monitor)
    if process:
        processes[title] = process


def execute_autostart(commands, pfolder, tmux, processes, log_publisher):
    for index, entry in enumerate(commands):
        if entry.get('selected', False):
            start_command(entry, pfolder, tmux, index, processes, log_publisher)
            time.sleep(2)


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

    commands = load_commands(log_publisher)
    start_map, stop_map = build_command_maps(commands)
    panel_names = [command_key(entry) for entry in commands if command_key(entry)]
    tmux = TmuxSend('bringup', panel_names)

    processes = {}
    pfolder = os.getenv('MARRTINOROBOT2_WS')
    if pfolder is None:
        log_publisher.publish_log("Error: MARRTINOROBOT2_WS environment variable not set.")
    elif os.getenv('MARRTINOROBOT2_SKIP_AUTOSTART', '').lower() in ('1', 'true', 'yes', 'on'):
        log_publisher.publish_log("Autostart skipped by MARRTINOROBOT2_SKIP_AUTOSTART.")
    else:
        execute_autostart(commands, pfolder, tmux, processes, log_publisher)

    while True:
        connection, client_address = sock.accept()
        try:
            log_publisher.publish_log(f"Connection from {client_address}")
            data = connection.recv(320).decode('utf-8').strip()

            if not data:
                continue

            # Preferences can be changed from bringup.php while this server is
            # running. Reload them for every action so a newly enabled Monitor
            # flag is applied the next time that command is started.
            commands = load_commands(log_publisher)
            start_map, stop_map = build_command_maps(commands)
            panel_names = [command_key(entry) for entry in commands if command_key(entry)]

            pfolder = os.getenv('MARRTINOROBOT2_WS')
            if pfolder is None:
                log_publisher.publish_log("Error: MARRTINOROBOT2_WS environment variable not set.")
                continue

            if data in stop_map:
                entry = stop_map[data]
                title = command_key(entry)
                if title in processes:
                    stop_process(processes[title], title, log_publisher)
                    del processes[title]
                else:
                    log_publisher.publish_log(f"No running process for '{title}' to terminate.")
                continue

            if data in start_map:
                entry = start_map[data]
                title = command_key(entry)
                index = panel_names.index(title) if title in panel_names else 0
                start_command(entry, pfolder, tmux, index, processes, log_publisher)
                continue

            log_publisher.publish_log(f"Command '{data}' not found in commands.json.")
        finally:
            connection.close()


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
