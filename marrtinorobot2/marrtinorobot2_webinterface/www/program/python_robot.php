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
<body>

<?php include "../nav.php" ?>

<div class="container-fluid">

<div class="row">
       <!-- <div class="col-md-6">
        <button class="btn btn-outline-success" id="hide_code" onclick="hide_blockly() ">Python</button>
        <button class="btn btn-outline-success" id="hide_code" onclick="show_blockly() ">Blockly</button>
        <button class="btn btn-outline-primary" onclick="showCode()">Show code</button>
        <button class="btn btn-outline-primary" id="run_btn" onclick="runCode()">Run</button>
        <button class="btn btn-outline-danger" id="stop_btn" onclick="stopCode()">Stop</button>
        <button class="btn btn-outline-warning" id="loadfile_btn" onclick="load_from_file()">Load file</button>
        <button class="btn btn-outline-success"  id="savefile_btn" onclick="save_to_file()">Save file</button>
        
        </div> -->
       <div class="col-md-6">
          Robot IP 
          <script>
           document.write("<td><input type=\"text\" name=\"IP\" id=\"IP\" value=\"" + 
                window.location.hostname + "\" width=240></td>")
        </script>
         <button onclick="connect()">Connect</button> 
          <button onclick="disconnect()">Disconnect</button> 
           
          <div id="connection"><font color='red'>Not Connected</font> </div>
     
          Status 
          <div id="status" style="color: blue;" >Idle</div>   
      </div>

    </div>
    





  <br>

  <p>
    <button id="run_btn" onclick="runCode()" class="btn waves-effect waves-light blue" style="margin-right:10px">Run</button>
    <button id="stop_btn" onclick="stopCode()" class="btn waves-effect waves-light blue">Stop</button>
  </p>
    
  <div class="row">
       <div class="col-md-9">
            <textarea id="code" rows=20 cols=60 style="height:400px"># Write your robot program here
            </textarea>
            <table border=0>
              <tr>  <th>Display Monitor</th>  </tr>
              <tr>
              <td>
                <textarea id="display" readonly></textarea>

              <br>
              <!-- <button onclick="clearDisplay()" style="
                background-color: black;
                color: #33FF33;
                border: 2px solid #33FF33;
                padding: 10px;
                font-family: 'Courier New', monospace;
                font-size: 16px;
                cursor: pointer; 
              ">Clear</button>-->
              </td>
              </tr>
            </table>
      </div>
   
      <div class="col-md-3">Face<div class="iframe-container"><iframe loading="lazy" src="/social/marrtina04.html"></iframe>
       <div class="image-container">
         <img height=300 id='image' src="lastimage/lastimage.jpg" alt=""/>  
    
     </div>
  </div>

       </div>
<h3>Commands</h3>

<listing style="font-size:125%;">




# motion
# ==============================
robot.forward(m)
robot.backward(m)
robot.left(deg)
robot.right(deg)
robot.setSpeed(lx,az,tm,stopend=False)


# range distance
# ==============================
d = robot.obstacleDistance(dir_deg=0)

 
# audio
# ==============================
robot.sound(name)
robot.say(text, language='en')

# vision
# ==============================
img = robot.getImage() # dysplay image
n = robot.faceDetection(img)

# tag 
# ==============================
b = robot.tagTrigger()  # boolean
id = robot.tagID()      # id
d = robot.tagDistance() # [m]
a = robot.tagAngle()    # [deg]
robot.tagClean()
display((robot.tagID()))


# utils
# ==============================
robot.wait(sec)		    # Wait for X seconds

# function

robot.display(x) 		    # Print the content of 'x' or the result of function 'x' in the Display area 

# Social

</listing>

<br>
<hr>
<br>





<br><br><br>

</div>


                <!-- ****** SCRIPTS ****** -->

<script src="../js/jquery-2.1.1.min.js"></script>
<script src="../js/materialize.js"></script>
<script src="../js/init.js"></script>

  <script>

    document.getElementById("run_btn").disabled = true;
    document.getElementById("stop_btn").disabled = true;

    function runCode() {
      var code = document.getElementById("code").value;
      wsrobot_send(code);
    }

    function stopCode() {
      // quit the program and stop the robot
      wsrobot_send("stop"); 
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


    window.setInterval(function()
    {
        document.getElementById('image').src = "/viewer/img/lastimage.jpg?random="+new Date().getTime();
    }, 5000);

  </script>

</body>
</html>

