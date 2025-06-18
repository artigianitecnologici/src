<!DOCTYPE html>
<html>
<head>
  <meta http-equiv="Content-Type" content="text/html; charset=UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1.0"/>
  <title>Python robot</title>

  <!-- CSS  -->
  <link href="../css/materialize.css" type="text/css" rel="stylesheet" media="screen,projection"/>
  <link href="../css/style.css" type="text/css" rel="stylesheet" media="screen,projection"/>
  
  <!-- Bootstrap CSS -->
  <link rel="stylesheet" href="bootstrap/css/bootstrap.min.css">
  <!--<script src="js/jquery-3.4.1.min.js"></script>-->
  <script src="bootstrap/js/bootstrap.min.js"></script>
  <script src="websocket_robot.js"></script>  
  <style>
      #display {
        width: 500px;
        height: 200px;
        background-color: black;
        color: #33FF33; /* Fosfori verdi */
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
<!-- <head>
  <meta charset="utf-8">
  <title>Tile Robot</title>
  <script src="websocket_robot.js"></script>
  <style>
    body {
      background-color: #fff;
      font-family: sans-serif;
    }
    h1 {
      font-weight: normal;
      font-size: 140%;
    }
  </style>
</head> -->
<body>
  <?php include "../nav.php" ?>
  <h1>Tile robot</h1>

  <p>Program robot with predefined command tiles.</p>

  <p>
    <table border=0>
    <tr height=40>
      <td>Robot IP</td>
<script>
      document.write("<td><input type=\"text\" name=\"IP\" id=\"IP\" value=\"" + 
            window.location.hostname + "\" width=240></td>")
</script>
      <td width=100><button onclick="connect()">Connect</button></td>
      <td width=120><button onclick="disconnect()">Disconnect</button></td>
      <td><div id="connection"><font color='red'>Not Connected</font></div></td>
    </tr>
    <tr height=40>
      <td>Status</td>
      <td><div id="status" style="color: blue;" >Idle</div></td>
    </tr>
    </table>
  </p>

  <p>
  </p>

 


</div>
  <div class="row">
     <div class="col-md-6">

     <table border=0>
        <tr>  <th>Tiles workspace</th> <th></th> <th>Python code</th> </tr>

        <tr>
        <td>
        <div class="container-fluid">
      <table border=1>

          <tr>
            <td><button id="bip_btn" onclick="mycommand('A')"><img src="img/bip.png"></button></td>
            <td><button id="f_btn" onclick="mycommand('F')"><img src="img/up.png"></button></td>
            <td><button id="boom_btn" onclick="mycommand('C')"><img src="img/boom.png"></button></td>
          </tr>
          <tr>
            <td><button id="l_btn" onclick="mycommand('L')"><img src="img/rotleft.png"></button></td>
            <td><button id="run_btn" onclick="runCode()"><img src="img/run.png"></button></td>
            <td><button id="r_btn" onclick="mycommand('R')"><img src="img/rotright.png"></button></td>
          </tr>
          <tr>
            <td><button id="clr_btn" onclick="clearCode()"><img src="img/clear.png"></button></td>
            <td><button id="d_btn" onclick="mycommand('B')"><img src="img/down.png"></button></td>
            <td><button id="stop_btn" onclick="stopCode()"><img src="img/stop.png"></button></td>
          </tr>
      </table>
    
  </td>

  <td width=20>  <textarea id="display" style="display:none;" readonly></textarea>
  </td>

  <td>
    <div id="codeDiv" style="height: 400px; width: 240px; background-color: #DDDDDD; font-size: 120%;"></div>
    
  </td>
  <td>
          <table border=1>
                <tr>
                  <td><button id="laup_btn" onclick="mycommand('LAUP')"><img src="img/bip.png"></button></td>
                  <td><button id="tf_btn" onclick="mycommand('TU')"><img src="img/up.png"></button></td>
                  <td><button id="ladn_btn" onclick="mycommand('LADN')"><img src="img/boom.png"></button></td>
                </tr>
                <tr>
                  <td><button id="pl_btn" onclick="mycommand('PL')"><img src="img/rotleft.png"></button></td>
                  <td><button id="run_btn" onclick="runCode()"><img src="img/run.png"></button></td>
                  <td><button id="pr_btn" onclick="mycommand('PR')"><img src="img/rotright.png"></button></td>
                </tr>
                <tr>
                  <td><button id="raup_btn" onclick="mycommand('RAUP')"><img src="img/clear.png"></button></td>
                  <td><button id="td_btn" onclick="mycommand('TD')"><img src="img/down.png"></button></td>
                  <td><button id="radn_btn" onclick="mycommand('RADN')"><img src="img/stop.png"></button></td>
                </tr>
              </table>
            </td>

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

    document.getElementById("run_btn").disabled = true;
    document.getElementById("stop_btn").disabled = true;
    clearCode();

    function runCode() {
      wsrobot_send(currentcode());
    }

    function stopCode() {
      // quit the program and stop the robot
      wsrobot_send("stop"); 
    }

    function decode(c) {
        if (c === 'F') return "robot.forward";
        if (c === 'B') return "robot.backward";
        if (c === 'L') return "robot.left";
        if (c === 'R') return "robot.right";
        if (c === 'A') return "robot.bip";
        if (c === 'C') return "robot.boom";
        if (c === 'TU' || c === 'TD') return "robot.tilt";
        if (c === 'PL' || c === 'PR') return "robot.pan";
        if (c === 'LADN' || c === 'LAUP') return "robot.left_arm";
        if (c === 'RADN' || c === 'RAUP') return "robot.right_arm";
        return "robot.unknown";
      }

    function clearCode() {
       program = ""; last_command = ""; last_index = 0;
       document.getElementById("codeDiv").innerHTML = "<pre>" + currentcode() + "</pre>";        
    }

    function mycommand(c) {
        let increment;

        if (c === 'L' || c === 'R') {
          increment = 15;
        } else if (c === 'TU') {
          program += "robot.tilt(-15)\n";
          program += "robot.wait(2)\n";
          updateCodeView();
          return;
        } else if (c === 'TD') {
          program += "robot.tilt(15)\n";
          program += "robot.wait(2)\n";
          updateCodeView();
          return;
        } else if (c === 'PL') {
          program += "robot.pan(-15)\n";
          program += "robot.wait(2)\n";
          updateCodeView();
          return;
        } else if (c === 'PR') {
          program += "robot.pan(15)\n";
          program += "robot.wait(2)\n";
          updateCodeView();
          return;
        }  else if (c === 'LAUP') {
          program += "robot.left_arm(30)\n";
          program += "robot.wait(2)\n";
          updateCodeView();
          return;
        } else if (c === 'LADN') {
          program += "robot.left_arm(-30)\n";
          program += "robot.wait(2)\n";
          updateCodeView();
          return;
        }  else if (c === 'RAUP') {
          program += "robot.right_arm(30)\n";
          program += "robot.wait(2)\n";
          updateCodeView();
          return;
        } else if (c === 'RADN') {
          program += "robot.right_arm(-30)\n";
          program += "robot.wait(2)\n";
          updateCodeView();
          return;
        } else {
          increment = 0.1;
        }

        if (c !== last_command) {
          if (last_command !== "") {
            program += decode(last_command) + "(" + last_index + ")\n";
          }
          last_command = c;
          last_index = increment;
        } else {
          last_index += increment;
        }

        updateCodeView();
      }

function updateCodeView() {
  document.getElementById("codeDiv").innerHTML = "<pre>" + currentcode() + "</pre>";
}

    function currentcode() {
        //#var code = "begin()\n" + program; 
        var code =  program; 
        if (last_command!="") {
            code += decode(last_command)+"("+last_index+")\n";
        }
        // code += "end()\n";
        return code;
    }

    function check_connection() {
        console.log("check connection")
        if (wsrobot_connected()) {
            console.log("wsrobot_connected true")
            document.getElementById("connection").innerHTML = "<font color='green'>Connected</font>";
            document.getElementById("run_btn").disabled = false;
            document.getElementById("stop_btn").disabled = false;
        }
        else {
            console.log("wsrobot_connected false")
            document.getElementById("connection").innerHTML = "<font color='red'>Not Connected</font>";
            document.getElementById("run_btn").disabled = true;
            document.getElementById("stop_btn").disabled = true;
        }
    }

    function connect() {
        wsrobot_init(9913);  // init websocket robot
        setTimeout(check_connection, 1000);
    }

    function disconnect() {
        wsrobot_quit();  // init websocket robot
        setTimeout(check_connection, 1000);
    }



  </script>

</body>
</html>
