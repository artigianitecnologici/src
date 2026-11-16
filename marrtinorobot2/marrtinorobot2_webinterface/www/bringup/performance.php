<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8" />
    <title>MARRtino Performance Monitor</title>
    <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
    <link rel="stylesheet" href="../bootstrap/css/bootstrap.min.css">
    <script src="../js/jquery-3.4.1.min.js"></script>
    <script src="../bootstrap/js/bootstrap.min.js"></script>
    <script type="text/javascript" src="../js/roslib.min.js"></script>
    <style>
        :root {
            --accent: rgb(157, 32, 104);
            --accent-2: #38d6ff;
            --accent-3: #42f59b;
            --bg-0: #070a12;
            --bg-1: #0d1322;
            --panel: rgba(14, 20, 35, 0.92);
            --line: rgba(255, 255, 255, 0.13);
            --text: #eef3ff;
            --muted: #9aa7bd;
        }

        body {
            min-height: 100vh;
            color: var(--text);
            background:
                radial-gradient(circle at 16% 12%, rgba(157, 32, 104, 0.28), transparent 31%),
                radial-gradient(circle at 86% 18%, rgba(56, 214, 255, 0.15), transparent 28%),
                linear-gradient(135deg, var(--bg-0), var(--bg-1) 58%, #190d1a);
        }

        .perf-shell {
            padding: 24px;
        }

        .perf-heading {
            display: flex;
            justify-content: space-between;
            align-items: flex-end;
            gap: 16px;
            margin-bottom: 20px;
        }

        .perf-heading h1 {
            margin: 0;
            font-weight: 800;
            letter-spacing: 0;
        }

        .perf-heading p {
            margin: 6px 0 0;
            color: var(--muted);
        }

        .btn-cyber {
            border-radius: 6px;
            border: 1px solid rgba(255,255,255,0.18);
            color: #fff;
            background: rgba(255,255,255,0.07);
        }

        .btn-accent {
            color: #fff;
            border-color: var(--accent);
            background: var(--accent);
            box-shadow: 0 0 18px rgba(157, 32, 104, 0.34);
        }

        .btn-kill {
            color: #fff;
            border: 1px solid rgba(255, 91, 91, 0.75);
            border-radius: 5px;
            background: rgba(190, 35, 45, 0.82);
            white-space: nowrap;
        }

        .btn-kill:hover,
        .btn-kill:focus {
            color: #fff;
            background: #dc3545;
        }

        .action-status {
            min-height: 24px;
            margin: 10px 16px;
            color: var(--accent-3);
        }

        .metric-grid {
            display: grid;
            grid-template-columns: repeat(3, minmax(160px, 1fr));
            gap: 12px;
            margin-bottom: 16px;
        }

        .metric {
            border: 1px solid var(--line);
            border-radius: 8px;
            padding: 14px;
            background: linear-gradient(180deg, rgba(255,255,255,0.08), rgba(255,255,255,0.035));
        }

        .metric span {
            display: block;
            color: var(--muted);
            font-size: 0.78rem;
            text-transform: uppercase;
        }

        .metric strong {
            display: block;
            margin-top: 4px;
            font-size: 1.55rem;
            line-height: 1;
        }

        .control-panel {
            border: 1px solid var(--line);
            border-radius: 8px;
            background: var(--panel);
            box-shadow: 0 24px 80px rgba(0, 0, 0, 0.34), 0 0 0 1px rgba(157, 32, 104, 0.12);
            overflow: hidden;
        }

        .panel-title {
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 14px 16px;
            border-bottom: 1px solid var(--line);
            background: linear-gradient(90deg, rgba(157, 32, 104, 0.24), rgba(56, 214, 255, 0.08));
        }

        .panel-title h2 {
            margin: 0;
            font-size: 1rem;
            font-weight: 700;
        }

        .table-console {
            margin: 0;
            color: var(--text);
        }

        .table-console thead th {
            border: 0;
            color: #cbd5e1;
            background: rgba(255, 255, 255, 0.055);
            font-size: 0.78rem;
            text-transform: uppercase;
        }

        .table-console td {
            border-top: 1px solid rgba(255,255,255,0.08);
            vertical-align: middle;
        }

        .bar {
            width: 100%;
            min-width: 90px;
            height: 8px;
            border-radius: 999px;
            background: rgba(255,255,255,0.08);
            overflow: hidden;
        }

        .bar-fill {
            height: 100%;
            background: linear-gradient(90deg, var(--accent), var(--accent-2));
        }

        .raw-output {
            margin: 0;
            padding: 14px;
            color: #d7fff2;
            background: #050914;
            border-top: 1px solid var(--line);
            max-height: 260px;
            overflow-y: auto;
            white-space: pre-wrap;
        }

        @media (max-width: 720px) {
            .perf-shell {
                padding: 16px 10px;
            }

            .perf-heading {
                display: block;
            }

            .metric-grid {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
<?php include "../nav.php" ?>
<div class="container-fluid perf-shell">
    <div class="perf-heading">
        <div>
            <h1>Performance Monitor</h1>
            <p>Processi ordinati per CPU nel container ROS2, aggiornati automaticamente.</p>
        </div>
        <div>
            <a href="bringup.php" class="btn btn-cyber">Back to Bringup</a>
            <button type="button" class="btn btn-accent" onclick="requestPerformance()">Refresh</button>
        </div>
    </div>

    <div class="metric-grid">
        <div class="metric">
            <span>Total CPU</span>
            <strong id="totalCpu">--</strong>
        </div>
        <div class="metric">
            <span>Total MEM</span>
            <strong id="totalMem">--</strong>
        </div>
        <div class="metric">
            <span>Last Update</span>
            <strong id="lastUpdate">--</strong>
        </div>
    </div>

    <div class="control-panel">
        <div class="panel-title">
            <h2>Process Matrix</h2>
            <span id="processCount">0 processi</span>
        </div>
        <div class="table-responsive">
            <table class="table table-console">
                <thead>
                <tr>
                    <th>PID</th>
                    <th>CPU</th>
                    <th>MEM</th>
                    <th>Process</th>
                    <th>Command</th>
                    <th>Action</th>
                </tr>
                </thead>
                <tbody id="processRows">
                <tr><td colspan="6">Waiting for data...</td></tr>
                </tbody>
            </table>
        </div>
        <div id="actionStatus" class="action-status" role="status"></div>
        <pre id="rawOutput" class="raw-output"></pre>
    </div>
</div>

<script>
    var ros = new ROSLIB.Ros({
        url: 'ws:' + window.location.hostname + ':9090'
    });

    ros.on('connection', function () {
        document.getElementById("status").innerHTML = "Connected";
        requestPerformance();
        setInterval(requestPerformance, 5000);
    });

    ros.on('error', function () {
        document.getElementById("status").innerHTML = "Error";
    });

    ros.on('close', function () {
        document.getElementById("status").innerHTML = "Closed";
    });

    var commandTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/shell_command',
        messageType: 'std_msgs/String'
    });

    var outputTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/command_output',
        messageType: 'std_msgs/String'
    });

    var processIndex = {};
    var refreshTimer = null;

    function requestPerformance() {
        var command = "ps -eo pid,ppid,pcpu,pmem,comm,args --sort=-pcpu | head -30";
        commandTopic.publish(new ROSLIB.Message({data: command}));
    }

    function parseProcessLine(line) {
        var match = line.trim().match(/^(\d+)\s+(\d+)\s+([\d.]+)\s+([\d.]+)\s+(\S+)\s+(.*)$/);
        if (!match) {
            return null;
        }
        return {
            pid: match[1],
            ppid: match[2],
            cpu: parseFloat(match[3]) || 0,
            mem: parseFloat(match[4]) || 0,
            comm: match[5],
            args: match[6]
        };
    }

    function renderPerformance(output) {
        var lines = output.trim().split(/\n+/);
        var rows = [];
        processIndex = {};
        for (var i = 1; i < lines.length; i++) {
            var row = parseProcessLine(lines[i]);
            if (row) {
                rows.push(row);
                processIndex[row.pid] = row;
            }
        }

        var totalCpu = rows.reduce(function (sum, row) { return sum + row.cpu; }, 0);
        var totalMem = rows.reduce(function (sum, row) { return sum + row.mem; }, 0);
        document.getElementById("totalCpu").innerHTML = totalCpu.toFixed(1) + "%";
        document.getElementById("totalMem").innerHTML = totalMem.toFixed(1) + "%";
        document.getElementById("lastUpdate").innerHTML = new Date().toLocaleTimeString();
        document.getElementById("processCount").innerHTML = rows.length + " processi";
        document.getElementById("rawOutput").textContent = output;

        var html = rows.map(function (row) {
            var width = Math.min(100, row.cpu).toFixed(1);
            return "<tr>" +
                "<td>" + row.pid + "</td>" +
                "<td><strong>" + row.cpu.toFixed(1) + "%</strong><div class=\"bar\"><div class=\"bar-fill\" style=\"width:" + width + "%\"></div></div></td>" +
                "<td>" + row.mem.toFixed(1) + "%</td>" +
                "<td>" + escapeHtml(row.comm) + "</td>" +
                "<td>" + escapeHtml(row.args) + "</td>" +
                "<td><button type=\"button\" class=\"btn btn-sm btn-kill\" onclick=\"killProcess(" + row.pid + ")\">Kill</button></td>" +
                "</tr>";
        }).join("");

        document.getElementById("processRows").innerHTML = html || "<tr><td colspan=\"6\">No process data.</td></tr>";
    }

    function killProcess(pid) {
        pid = Number(pid);
        var row = processIndex[String(pid)];
        if (!Number.isInteger(pid) || pid <= 2 || !row) {
            document.getElementById("actionStatus").textContent = "PID non valido o non più presente.";
            return;
        }

        var description = row.comm + " (PID " + pid + ")";
        if (!window.confirm("Terminare " + description + "?")) {
            return;
        }

        document.getElementById("actionStatus").textContent = "Invio SIGTERM a " + description + "...";
        commandTopic.publish(new ROSLIB.Message({data: "kill -TERM -- " + pid}));

        window.clearTimeout(refreshTimer);
        refreshTimer = window.setTimeout(function () {
            document.getElementById("actionStatus").textContent = "Richiesta inviata a " + description + ".";
            requestPerformance();
        }, 1000);
    }

    function escapeHtml(value) {
        return String(value).replace(/[&<>"']/g, function (char) {
            return ({
                "&": "&amp;",
                "<": "&lt;",
                ">": "&gt;",
                "\"": "&quot;",
                "'": "&#039;"
            })[char];
        });
    }

    outputTopic.subscribe(function (message) {
        if (message.data.indexOf("PID") !== -1 && message.data.indexOf("%CPU") !== -1) {
            renderPerformance(message.data);
        }
    });
</script>
</body>
</html>
