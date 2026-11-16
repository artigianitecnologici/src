<!DOCTYPE html>
<html>
<head>
  <meta http-equiv="Content-Type" content="text/html; charset=UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1.0"/>
  <title>Python robot</title>

  <!-- CSS -->
  <link href="../css/materialize.css" type="text/css" rel="stylesheet" media="screen,projection"/>
  <link href="../css/style.css" type="text/css" rel="stylesheet" media="screen,projection"/>

  <!-- Bootstrap CSS -->
  <link rel="stylesheet" href="bootstrap/css/bootstrap.min.css">
  <script src="bootstrap/js/bootstrap.min.js"></script>
  <script src="websocket_robot.js"></script>  

  <style>
    #display {
      width: 500px;
      height: 200px;
      background-color: black;
      color: #33FF33;
      font-family: 'Courier New', monospace;
      font-size: 20px;
      border: 2px solid #33FF33;
      padding: 10px;
      resize: none;
      overflow: hidden;
      text-shadow: 0px 0px 5px #33FF33;
    }
  </style>
</head>

<body>
  <?php include "../nav.php" ?>
  <h1>Tile robot</h1>
  <p>Program robot with predefined command tiles.</p>

  <table border=0>
    <tr height=40>
      <td>Robot IP</td>
      <script>
        document.write("<td><input type='text' name='IP' id='IP' value='" + window.location.hostname + "' width=240></td>");
      </script>
      <td width=100><button onclick="connect()">Connect</button></td>
      <td width=120><button onclick="disconnect()">Disconnect</button></td>
      <td><div id="connection"><font color='red'>Not Connected</font></div></td>
    </tr>
    <tr height=40>
      <td>Status</td>
      <td><div id="status" style="color: blue;">Idle</div></td>
    </tr>
  </table>

  <div class="row">
    <div class="col-md-6">
      <table border=0>
        <tr><th>Tiles workspace</th><th></th><th>Python code</th></tr>
        <tr>
          <td>
           
              <table border=1>
                <tr>
                  <td><button id="bip_btn" onclick="command('A')"><img src="img/bip.png"></button></td>
                  <td><button id="f_btn" onclick="command('F')"><img src="img/up.png"></button></td>
                  <td><button id="boom_btn" onclick="command('C')"><img src="img/boom.png"></button></td>
                </tr>
                <tr>
                  <td><button id="l_btn" onclick="command('L')"><img src="img/rotleft.png"></button></td>
                  <td><button id="run_btn" onclick="runCode()"><img src="img/run.png"></button></td>
                  <td><button id="r_btn" onclick="command('R')"><img src="img/rotright.png"></button></td>
                </tr>
                <tr>
                  <td><button id="clr_btn" onclick="clearCode()"><img src="img/clear.png"></button></td>
                  <td><button id="d_btn" onclick="command('B')"><img src="img/down.png"></button></td>
                  <td><button id="stop_btn" onclick="stopCode()"><img src="img/stop.png"></button></td>
                </tr>
              </table>
            
          </td>
          <td width=20>
            <textarea id="display" readonly></textarea>
          </td>
          <td>
            <div id="codeDiv" style="height: 400px; width: 240px; background-color: #DDDDDD; font-size: 120%;"></div>
          </td>
          <table border=1>
                <tr>
                  <td><button id="bip_btn" onclick="command('A')"><img src="img/bip.png"></button></td>
                  <td><button id="f_btn" onclick="command('F')"><img src="img/up.png"></button></td>
                  <td><button id="boom_btn" onclick="command('C')"><img src="img/boom.png"></button></td>
                </tr>
                <tr>
                  <td><button id="l_btn" onclick="command('L')"><img src="img/rotleft.png"></button></td>
                  <td><button id="run_btn" onclick="runCode()"><img src="img/run.png"></button></td>
                  <td><button id="r_btn" onclick="command('R')"><img src="img/rotright.png"></button></td>
                </tr>
                <tr>
                  <td><button id="clr_btn" onclick="clearCode()"><img src="img/clear.png"></button></td>
                  <td><button id="d_btn" onclick="command('B')"><img src="img/down.png"></button></td>
                  <td><button id="stop_btn" onclick="stopCode()"><img src="img/stop.png"></button></td>
                </tr>
              </table>

        </tr>
      </table>
    </div>
  </div>
  <hr>

  <!-- ****** SCRIPTS ****** -->
  <script>
    var program = "";
    var last_command = "";
    var last_index = 0;

    const commandMap = {
      F: "robot.forward",
      B: "robot.backward",
      L: "robot.left",
      R: "robot.right",
      A: "robot.bip",
      C: "robot.boom"
    };

    function decode(c) {
      return commandMap[c] || "robot.unknown";
    }

    function clearCode() {
      program = ""; last_command = ""; last_index = 0;
      document.getElementById("codeDiv").innerHTML = "<pre>" + currentcode() + "</pre>";        
      document.getElementById("display").value = "";
    }

    function command(c) {
      if (c != last_command) {
        if (last_command !== "") {
          program += decode(last_command) + "(" + last_index + ")\n";
        }
        last_command = c;
        last_index = (c === 'L' || c === 'R') ? 30 : 0.1;
      } else {
        last_index += (c === 'L' || c === 'R') ? 30 : 0.1;
      }

      var code = currentcode();
      document.getElementById("codeDiv").innerHTML = "<pre>" + code + "</pre>";
      document.getElementById("display").value = code;
    }

    function currentcode() {
      let code = program;
      if (last_command !== "") {
        code += decode(last_command) + "(" + last_index + ")\n";
      }
      return code;
    }

    function runCode() {
      let code = currentcode();
      wsrobot_send(code);
      document.getElementById("status").innerText = "Running";
    }

    function stopCode() {
      wsrobot_send("stop");
      document.getElementById("status").innerText = "Stopped";
    }

    function check_connection() {
      if (wsrobot_connected()) {
        document.getElementById("connection").innerHTML = "<font color='green'>Connected</font>";
        document.getElementById("run_btn").disabled = false;
        document.getElementById("stop_btn").disabled = false;
      } else {
        document.getElementById("connection").innerHTML = "<font color='red'>Not Connected</font>";
        document.getElementById("run_btn").disabled = true;
        document.getElementById("stop_btn").disabled = true;
      }
    }

    function connect() {
      wsrobot_init(9913);
      setTimeout(check_connection, 1000);
    }

    function disconnect() {
      wsrobot_quit();
      setTimeout(check_connection, 1000);
    }

    // Inizializzazione
    document.getElementById("run_btn").disabled = true;
    document.getElementById("stop_btn").disabled = true;
    clearCode();
  </script>

</body>
</html>
