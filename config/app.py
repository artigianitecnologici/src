from flask import Flask, render_template, request, redirect, url_for, jsonify
import subprocess

app = Flask(__name__)

# Utility function to run shell commands
def run_command(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, shell=True)
        return result.stdout.strip()
    except Exception as e:
        return str(e)

@app.route('/')
def index():
    return render_template('index.html')

# Bluetooth routes
@app.route('/bluetooth')
def bluetooth():
    paired_devices = run_command("echo -e 'paired-devices\nquit' | bluetoothctl")
    parsed_devices = []
    for line in paired_devices.split('\n'):
        if "Device" in line:
            parsed_devices.append(line.strip())
    return render_template('bluetooth.html', devices=parsed_devices)

@app.route('/bluetooth/scan')
def bluetooth_scan():
    devices = run_command("echo -e 'scan on\nsleep 10\ndevices\nquit' | bluetoothctl")
    parsed_devices = []
    for line in devices.split('\n'):
        if "Device" in line:
            parsed_devices.append(line.strip())
    return render_template('bluetooth_scan.html', devices=parsed_devices)

@app.route('/bluetooth/connect', methods=['POST'])
def bluetooth_connect():
    mac_address = request.form.get('mac_address')
    if mac_address:
        output = run_command(f"echo -e 'connect {mac_address}\nquit' | bluetoothctl")
        return jsonify(output=output)
    return jsonify(output="MAC Address is required."), 400

# WiFi routes
@app.route('/wifi')
def wifi():
    networks = run_command("nmcli -t -f SSID dev wifi")
    return render_template('wifi.html', networks=networks.split('\n'))

@app.route('/wifi/connect', methods=['POST'])
def wifi_connect():
    ssid = request.form.get('ssid')
    password = request.form.get('password')
    if ssid:
        command = f"nmcli dev wifi connect '{ssid}' password '{password}'"
        output = run_command(command)
        return jsonify(output=output)
    return jsonify(output="SSID is required."), 400

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5400)
