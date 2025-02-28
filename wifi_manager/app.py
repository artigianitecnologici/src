from flask import Flask, render_template, request, redirect, url_for, flash
import subprocess

app = Flask(__name__)
app.secret_key = 'supersecretkey'  # Necessario per usare i messaggi flash

def run_command(cmd):
    """
    Funzione helper per eseguire un comando e restituire output o eventuali errori.
    """
    try:
        output = subprocess.check_output(cmd, stderr=subprocess.STDOUT).decode()
        return output, None
    except subprocess.CalledProcessError as e:
        return None, e.output.decode()

@app.route('/')
def index():
    """
    Pagina iniziale con link per gestire la connessione della scheda locale e visualizzare le regole di routing.
    """
    return render_template('base.html')

@app.route('/local_networks')
def local_networks():
    """
    Elenca le reti Wi‑Fi disponibili utilizzando la scheda wlan1 (quella che si connette alla rete locale).
    """
    # Il comando specifica l'interfaccia "wlan1" (modifica se necessario)
    cmd = ["nmcli", "-t", "-f", "SSID,SIGNAL,SECURITY", "device", "wifi", "list", "ifname", "wlan1"]
    output, error = run_command(cmd)
    networks = []
    if error:
        flash("Errore nella lettura delle reti locali: " + error, "danger")
    else:
        for line in output.strip().split("\n"):
            parts = line.split(":")
            if len(parts) >= 3:
                ssid, signal, security = parts[0], parts[1], parts[2]
                networks.append({
                    'ssid': ssid,
                    'signal': signal,
                    'security': security
                })
    return render_template('local_index.html', networks=networks)

@app.route('/local/select', methods=['POST'])
def local_select():
    """
    Riceve l'SSID della rete locale selezionata e mostra il form per inserire la password.
    """
    ssid = request.form.get('ssid')
    if not ssid:
        flash("Devi selezionare una rete.", "danger")
        return redirect(url_for('local_networks'))
    return render_template('local_connect.html', ssid=ssid)

@app.route('/local/connect', methods=['POST'])
def local_connect():
    """
    Connette la scheda wlan1 alla rete locale utilizzando ssid e password.
    Dopo la connessione, reindirizza alla pagina per abilitare il routing.
    """
    ssid = request.form.get('ssid')
    password = request.form.get('password')
    if not ssid or not password:
        flash("SSID e password sono necessari.", "danger")
        return redirect(url_for('local_networks'))

    # Comando per connettere wlan1 alla rete locale
    cmd = ["nmcli", "device", "wifi", "connect", ssid, "password", password, "ifname", "wlan1"]
    output, error = run_command(cmd)
    if error:
        flash("Errore durante la connessione alla rete locale: " + error, "danger")
        return redirect(url_for('local_networks'))
    
    flash("Connessione alla rete locale riuscita: " + output, "success")
    return redirect(url_for('routing_info'))

@app.route('/routing_info')
def routing_info():
    """
    Abilita l'IP forwarding e configura il NAT (routing) per instradare il traffico dalla scheda AP (wlan0)
    verso la scheda collegata alla rete locale (wlan1).
    Visualizza le regole NAT attive.
    """
    # Abilita l'IP forwarding (impostazione temporanea, per modificare in modo persistente
    # bisogna modificare /etc/sysctl.conf)
    subprocess.call("sysctl -w net.ipv4.ip_forward=1", shell=True)
    
    # Configura il NAT: in questo esempio, impostiamo come uscita l'interfaccia wlan1
    # Per sicurezza, prima svuotiamo la chain POSTROUTING della tabella nat
    subprocess.call("iptables -t nat -F", shell=True)
    subprocess.call("iptables -t nat -A POSTROUTING -o wlan1 -j MASQUERADE", shell=True)
    
    # (Opzionale) Aggiungi regole di FORWARD per accettare il traffico tra le interfacce
    subprocess.call("iptables -F FORWARD", shell=True)
    subprocess.call("iptables -A FORWARD -i wlan1 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT", shell=True)
    subprocess.call("iptables -A FORWARD -i wlan0 -o wlan1 -j ACCEPT", shell=True)
    
    # Mostra le regole NAT attive
    cmd = ["iptables", "-t", "nat", "-L", "-n", "--line-numbers"]
    output, error = run_command(cmd)
    if error:
        flash("Errore nella lettura delle regole NAT: " + error, "warning")
        output = "Nessuna regola disponibile."
    
    return render_template('routing_info.html', routing_info=output)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
