#!/usr/bin/env python3
#
# Copyright 2025 robotics-3d.com 
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Ferrarini Fabio
# Email : ferrarini09@gmail.com
# File  : websocket_robot.py

import rclpy

import os
import time
import asyncio
from datetime import datetime
from threading import Thread
import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web

from robot_cmd_ros import RobotCmdROS  # Importa la libreria


# Global variables
websocket_server = None  # WebSocket handler
run = True  # Main loop run flag
server_port = 9913  # Web server port
code = None
status = "Idle"  # Global robot status
list_ws = []
logdir = os.getenv('HOME') + '/log/'  # Directory for writing log files (programs, images,...)

class MyWebSocketServer(tornado.websocket.WebSocketHandler):
    def open(self):
        global list_ws
        self.client_ip = self.request.remote_ip
        print(f'New connection from {self.client_ip}')
        list_ws.append(self)
        print(f'Num connected clients: {len(list_ws)}')

    def on_message(self, message):
        global code, status
        if message == 'stop':
            print('Stop code and robot')
            if status != 'Idle':
                status = 'Stop'
            robot.stop()
        elif message.startswith('event'):
            print(f'Received event {message}')
        else:
            print('Code received')
            save_program(message)
            if status == 'Idle':
                run_code(message)
            else:
                print('Program running. This code is discarded.')
                self.write_message('Program running. This code is discarded.')

    def on_close(self):
        global list_ws
        print(f'Connection closed from {self.client_ip}')
        list_ws.remove(self)
        print(f'Num connected clients: {len(list_ws)}')

    def check_origin(self, origin):
        return True

# def display(text):
#     global list_ws
#     for ws in list_ws:
#         try:
#             ws.write_message(f'display {text}')
#         except tornado.websocket.WebSocketClosedError:
#             print('Cannot write to closed WebSocket')
async def display(text):
    global list_ws
    for ws in list_ws:
        try:
            ws.write_message(f'display {text}')  # ✅ Usa await
        except tornado.websocket.WebSocketClosedError:
            print('Cannot write to closed WebSocket')

def main_loop():
    global run, status
    asyncio.set_event_loop(asyncio.new_event_loop())  # Crea un nuovo ciclo eventi per questo thread
    while run:
        time.sleep(1)
        for ws in list_ws:
            try:
                ws.write_message(status)
            except tornado.websocket.WebSocketClosedError:
                pass


fncode_running = False


# def deffunctioncode(code):
#     r = "def fncode():\n"
#     #r += "  robot.begin()\n"
#     for line in code.splitlines():
#         #line = line.strip()
#         if line and not line.startswith("#"):  # Ignora i commenti
#             r += f"  {line}\n"  
            
#     #r += "  robot.end()\n"
#     print("Generated function code:\n", r)  # Log del codice generato
#     return r
def deffunctioncode(code):
    r = "async def fncode():\n"  # ✅ Deve essere async per usare await!
    for line in code.splitlines():
        if line and not line.startswith("#"):  # Ignora i commenti
            if line.strip().startswith("display("):  # Se è una chiamata a display, aggiungi await
                r += f"  await {line.strip()}\n"  
            else:
                r += f"  {line}\n"
    print("Generated function code:\n", r)  # Log del codice generato
    return r


# def fncodeexcept(local_context):
#     global fncode_running
#     fncode_running = True
#     try:
#         loop = asyncio.new_event_loop()  # Crea un nuovo event loop
#         asyncio.set_event_loop(loop)  # Imposta il nuovo loop come corrente
#         local_context['fncode']()
#     except Exception as e:
#         print(f"CODE EXECUTION ERROR: {e}")
#         loop.run_until_complete(display(str(e)))  # Usa il loop per eseguire display()
#     fncode_running = False

def fncodeexcept(local_context):
    global fncode_running
    fncode_running = True
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(local_context['fncode']())  # ✅ Esegui la coroutine
    except NameError as e:
        error_msg = f"SYNTAX ERROR: {e}"
        print(error_msg)
        loop.run_until_complete(display(error_msg))  # ✅ Usa run_until_complete
    except Exception as e:
        error_msg = f"CODE EXECUTION ERROR: {e}"
        print(error_msg)
        loop.run_until_complete(display(error_msg))  # ✅ Usa run_until_complete
    fncode_running = False




# def exec_thread(code):
#     global fncode_running, robot
#     fncodestr = deffunctioncode(code)
#     local_context = {'robot': robot}
#     try:
#         print("Executing with context:", local_context)
#         exec(fncodestr, globals(), local_context)  # Passa il contesto globale e locale
#     except Exception as e:
#         print(f"FN CODE DEFINITION ERROR: {e}")
#         display(str(e))
#         return

#     # Assicurati che la funzione fncode esista
#     if 'fncode' not in local_context:
#         print("ERROR: fncode not defined")
#         return

#     thread = Thread(target=fncodeexcept, args=(local_context,))
#     thread.start()
#     while fncode_running and status != "Stop":
#         time.sleep(0.5)
#     thread.join()

import asyncio

def exec_thread(code):
    global fncode_running, robot
    fncodestr = deffunctioncode(code)
    local_context = {'robot': robot}

    try:
        print("Executing with context:", local_context)
        exec(fncodestr, globals(), local_context)  # Passa il contesto globale e locale
    except NameError as e:
        error_msg = f"SYNTAX ERROR: {e}"
        print(error_msg)
        send_error_to_display(error_msg)  # ✅ Usa una funzione sicura per WebSocket
        return
    except Exception as e:
        error_msg = f"FN CODE DEFINITION ERROR: {e}"
        print(error_msg)
        send_error_to_display(error_msg)
        return

    # Assicurati che la funzione fncode esista
    if 'fncode' not in local_context:
        print("ERROR: fncode not defined")
        return

    thread = Thread(target=fncodeexcept, args=(local_context,))
    thread.start()
    while fncode_running and status != "Stop":
        time.sleep(0.5)
    thread.join()


def send_error_to_display(error_msg):
    """Gestisce correttamente la visualizzazione dell'errore su WebSocket"""
    try:
        loop = asyncio.get_event_loop()
        if loop.is_running():
            asyncio.ensure_future(display(error_msg))
        else:
            loop.run_until_complete(display(error_msg))
    except RuntimeError:
        asyncio.run(display(error_msg))


def save_program(code):
    try:
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = os.path.join(logdir, f'{timestamp}.prg')
        with open(filename, 'w') as f:
            f.write(code)
    except Exception as e:
        print(f"ERROR saving program: {e}")

def run_code(code):
    global status
    if not code:
        return
    print("Executing code:")
    print(code)
    status = "Executing program"
    exec_thread(code)
    status = "Idle"
 
if __name__ == "__main__":
    os.makedirs(logdir, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = '/tmp/ros_logs'

    # Robot initialization
    rclpy.init()
    robot = RobotCmdROS()
    robot.begin()

    # Start WebSocket server
    application = tornado.web.Application([(r'/websocketserver', MyWebSocketServer)])
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(server_port)
    print(f"WebSocket server listening on port {server_port}")

    main_thread = Thread(target=main_loop)
    main_thread.start()

    try:
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down")

    # Cleanup
    robot.end()
    run = False
    main_thread.join()
    print("Web server and robot control stopped")
    rclpy.shutdown()
