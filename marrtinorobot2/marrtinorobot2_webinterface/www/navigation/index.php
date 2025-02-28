<!-- * -->
<html>
<head>
  <meta http-equiv="Content-Type" content="text/html; charset=UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1.0"/>
  <title>SOCIAL BRINGUP v.2.001</title>
  <link rel="stylesheet" href="bootstrap/css/bootstrap.min.css">
  <script src="js/jquery-3.4.1.min.js"></script>
  <script src="bootstrap/js/bootstrap.min.js"></script>
  <!-- CSS  
  <link href="css/materialize.css" type="text/css" rel="stylesheet" media="screen,projection"/>
  <link href="css/style.css" type="text/css" rel="stylesheet" media="screen,projection"/>
  -->
  <script src="js/websocket_robot.js"></script>
 

 

</head>

<body>

<!-- Nav Bar -->
<?php include "../social/nav.php" ?>
<!-- Eof Nav Bar -->

<div class="container-fluid">
  <div class="row">
 
    <div class="col-md-12"  style="background-color:yellow;" align="center" ><h2>MARRTINO ROBOT2 BRINGUP</h2></div>

  </div>
  <div class="row">
  <div class="col-md-1"></div>
    <div class="col-md-4">
    IP:
    <script>
      document.write("<td><input type=\"text\" name=\"IP\" id=\"IP\" value=\"" + 
            window.location.hostname + "\" width=240>")
    </script>
   
    </div>
    <div class="col-md-4">
    <div id="connection"><font color='red'>Not Connected</font></div>
    <!-- /
    <div id="status" style="color: blue;" >Idle</div>
   <p class="text-white">Connection status: <span id="status"></span></p>-->
     </div>
  </div>

  
</div>



<div class="row">
	<div class="col-md-1"></div>
    <div class="col-md-10">
    <table>
        <tr height=40>
        <td width=150><div id="ros" style="color: grey;" >ROS</div></td>
        <td width=150><div id="robot" style="color: grey;" >MARRtino</div></td>

  </tr>
  <tr height=40>
    <td><span id="odom" style="color: grey;">odom</span>     
        <span id="odom_status" style="color: grey;">OFF</span>   </td>
    <td><span id="laser" style="color: grey;">laser</span>   
        <span id="laser_status" style="color: grey;">OFF</span> </td>
    <td><span id="sonar" style="color: grey;">sonar</span>
        <span id="sonar_status" style="color: grey;">OFF</span>   </td>
  </tr>
  <tr height=40>
    <td><span id="rgb" style="color: grey;">RGB</span>
        <span id="rgb_status" style="color: grey;">OFF</span>        </td>
    <td><span id="depth" style="color: grey;">Depth</span>
        <span id="depth_status" style="color: grey;">OFF</span>      </td>
  </tr>
</table>


<table>
  <tr><td>TF</td></tr>
  <tr><td><div id="tf_map_odom" style="color: grey;">map -&gt; odom</div></td></tr>
  <tr><td><div id="tf_odom_base" style="color: grey;">odom -&gt; base_frame</div></td></tr>

  <tr><td><div id="tf_base_laser" style="color: grey;">base_frame -&gt; laser_frame</div></td></tr>
  <tr><td><div id="tf_base_rgb" style="color: grey;">base_frame -&gt; rgb_camera_frame</div></td></tr>
  <tr><td><div id="tf_base_depth" style="color: grey;">base_frame -&gt; depth_camera_frame</div></td></tr>
</table>

<p>
<button onclick="send_cmd('check')" class="btn waves-effect waves-light blue" style="margin-right:10px">Check status</button>
<button onclick="send_cmd('ros_quit')" class="btn waves-effect waves-light blue" style="margin-right:10px">Quit all</button>
<button id="shutdown_btn" onclick="send_cmd('shutdown')" class="btn waves-effect waves-light blue">Shutdown</button>
</p>
</div>
</div>




<div class="row">
<div class="col-md-1"></div>
<div class="col-md-11">

 
<table>

<tr height=40>
<td width=280>Bringup </td> 
<td width=80 align='center'>  
<td><button id="robot_start_btn" onclick="send_cmd('robot_start')" class="btn btn-primary ">Robot Start</button></td>
<td><button id="robot_quit_btn" onclick="send_cmd('robot_kill')" class="btn btn-light ">Robot quit</button></td>
</td>
</tr>
<tr height=40>
<td width=280>MicroROS </td> 
<td width=80 align='center'>  
<td><button id="robot_start_btn" onclick="send_cmd('microros_start')" class="btn btn-primary ">Social Start</button></td>
<td><button id="robot_quit_btn" onclick="send_cmd('microros_kill')" class="btn btn-light ">Social quit</button></td>
</td>
</tr>
<tr height=40>
<td width=280>WebCam</td> 
<td width=80 align='center'>
<td><button id="social_start_btn" onclick="send_cmd('webcam_start')" class="btn btn-primary ">Social Start</button></td>
<td><button id="social_quit_btn" onclick="send_cmd('webcam_kill')" class="btn btn-light ">Social quit</button></td></td>
</tr>
<tr height=40>
<td width=280>Pan Tilt</td> 
<td width=80 align='center'>
<td><button id="social_start_btn" onclick="send_cmd('pantilt_start')" class="btn btn-primary ">Social Start</button></td>
<td><button id="social_quit_btn" onclick="send_cmd('pantilt_kill')" class="btn btn-light ">Social quit</button></td></td>
</tr>

<tr height=40>
<td width=280>SLAM </td> 
<td width=80 align='center'>
<td><button id="speech_start_btn" onclick="send_cmd('slam_start')" class="btn btn-primary ">Start interactive mode</button></td>
<td><button id="speech_quit_btn" onclick="send_cmd('slam_kill')" class="btn btn-primary ">Stop interactive mode</button></td></td>
</tr>

<tr height=40>
<td width=280>Mapping </td> 
<td width=80 align='center'>
<td><button id="speech_start_btn" onclick="send_cmd('interactive_start')" class="btn btn-primary ">Start interactive mode</button></td>
<td><button id="speech_quit_btn" onclick="send_cmd('interactive_kill')" class="btn btn-primary ">Stop interactive mode</button></td></td>
</tr>

<tr height=40>
<td width=280>RViz</td> 
<td width=80 align='center'>
<td><button id="gazebo_start_btn" onclick="send_cmd('rviz_start')" class="btn btn-primary ">Start Interactive Off line</button></td>
<td><button id="gazebo_quit_btn" onclick="send_cmd('rviz_kill')" class="btn btn-primary ">Stop Interactive Off line</button></td></td>
</tr>

<tr height=40>
<td width=280>Gazebo</td> 
<td width=80 align='center'>
<td><button id="gazebo_start_btn" onclick="send_cmd('gazebo_start')" class="btn btn-primary ">Start Interactive Off line</button></td>
<td><button id="gazebo_quit_btn" onclick="send_cmd('gazebo_kill')" class="btn btn-primary ">Stop Interactive Off line</button></td></td>
</tr>

<tr height=40>
<td width=280>ASR OFF line</td> 
<td width=80 align='center'>
<td><button id="asr_start_btn" onclick="send_cmd('asroffline_start')" class="btn btn-primary ">Start ASR Off line</button></td>
<td><button id="asr_quit_btn" onclick="send_cmd('asroffline_kill')" class="btn btn-primary ">Stop  Off line</button></td></td>
</tr>

<tr height=40>
<td width=280>ASR with APP </td> 
<td width=80 align='center'>
<td><button id="asr_start_btn" onclick="send_cmd('asr_app_start')" class="btn btn-primary ">Start ASR with APP</button></td>
<td><button id="asr_quit_btn" onclick="send_cmd('asr_app_kill')" class="btn btn-primary ">Stop  ASR with APP</button></td></td>
</tr>

</table>


</div>

<!--

  <h2>GESTIONE SOCIAL </h2>
  
 
  <div class="row">
    <div class="col">
      <button id="face_start" onclick="send_cmd('face_start')" class="btn btn-outline-warning ">Face Start </button>  
   </div>
   <div class="col">
    <button id="face_start" onclick="send_cmd('face_stop')" class="btn btn-outline-warning ">Face Stop </button>  
  </div>
   <div class="col">
      <button id="face_reset" onclick="send_cmd('face_reset')" class="btn btn-outline-warning ">Face Reset</button> 
    </div>
 </div>



  <div class="row">
    <div class="col">
      <button id="social_stop" onclick="send_cmd('shutdown')" class="btn btn-outline-success ">Shutdown </button>  
   </div>
   <div class="col">
    <button id="social_reset" onclick="send_cmd('social_reset')" class="btn btn-outline-danger ">Social Reset </button>  
   </div>
  
  </div>
 -->
 
   
  
 </div>






<br><br><br>


<!-- The Modal -->
<div id="waitModal" class="modal">

  <!-- Modal content -->
  <div class="modal-content">
    <h2 align='center'>Please wait..</h2>
  </div>

</div>

</div>

</body>


<!-- ****** SCRIPTS ****** -->
<!--
<script src="js/jquery-2.1.1.min.js"></script>
<script src="js/materialize.js"></script>
<script src="js/init.js"></script>
-->
<script>


    function waitwindow_on() {
        var modal = document.getElementById('waitModal');
        modal.style.display = "block";
        console.log("waitwindow_on")
    }

    function waitwindow_off() {
        var modal = document.getElementById('waitModal');
        modal.style.display = "none";
        console.log("waitwindow_off")
    }

    // setTimeout(waitwindow_on(), 4000);
    

    //document.getElementById("simrobot_start_btn").disabled = false;


    // messages received from wsserver
    eventproc = function(event){
      v = event.data.split(' ');

      if (v[0]=='STATUS') {
          vs = '';
          for (i=1; i<v.length ; i++)
             vs += v[i]+' '
          vs = vs.trim();
          document.getElementById("status").innerHTML = vs;
          var d = new Date();
          console.log(d+" - status ["+vs+"]");
          if (vs=="Idle")
            waitwindow_off();
          else
            waitwindow_on();
      }
      else if (v[0]=='RESULT') {
          cv = v[2];
          console.log("log: "+ v[1] + " " + v[2])
          if (v[2]=='True' || parseFloat(v[2])>0) {
            //cv = "<font color='green'>"+v[2]+"</font>"
            d = document.getElementById(v[1])
            if (d!=null)
                d.style.color = 'green';
            //document.getElementById(v[1]+"_btn").style.color = 'green';

            //document.getElementById(v[1]+"_start_btn").disabled = false;
            //document.getElementById(v[1]+"_quit_btn").disabled = false;

            vst = v[1]+"_status"; // status label
            d = document.getElementById(vst);
            if (d!=null) {
                if (v[2]=='True')
                    d.innerHTML = 'OK';
                else
                    d.innerHTML = v[2];
                d.style.color = 'green';
            }



          }
          else if (v[2]=='False' || parseFloat(v[2])==0.0) {
            //cv = "<font color='red'>"+v[2]+"</font>"
            d = document.getElementById(v[1]);
            if (d!=null)
                d.style.color = 'red'
            //document.getElementById(v[1]+"_start_btn").disabled = false;
            //document.getElementById(v[1]+"_quit_btn").disabled = false;

            vst = v[1]+"_status"; // status label
            d = document.getElementById(vst);
            if (d!=null) {
                d.innerHTML = 'OFF';
                d.style.color = 'red';
            }
          }


      }
      else if (v[0]=='VALUE') {
          vs = '';
          for (i=2; i<v.length ; i++)
             vs += v[i]+' '
          document.getElementById(v[1]).innerHTML = vs;
      }
    }

    function connect() {
        wsrobot_init(9912);  // init websocket robot
        setTimeout(check_connection, 1000);
        websocket.onmessage = eventproc;
    }

    function disconnect() {
        wsrobot_quit();  // init websocket robot
        setTimeout(check_connection, 1000);
    }

    function check_connection() {
        console.log("check connection")
        if (wsrobot_connected()) {
            console.log("wsrobot_connected true")
            document.getElementById("connection").innerHTML = "<font color='green'>Connected</font>";
            //document.getElementById("run_btn").disabled = false;
            //document.getElementById("stop_btn").disabled = false;
        }
        else {
            console.log("wsrobot_connected false")
            document.getElementById("connection").innerHTML = "<font color='red'>Not Connected</font>";
            //document.getElementById("run_btn").disabled = true;
            //document.getElementById("stop_btn").disabled = true;
        }
    }

    function send_cmd(action) {
        wsrobot_send(action);
    }

 


</script>

</html>

