<!DOCTYPE html>
<html>

<head>
  <meta charset="utf-8" />
  <title>MARRtino Social Interface</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
  
  <!-- Bootstrap CSS -->
  <link rel="stylesheet" href="bootstrap/css/bootstrap.min.css">
  <script src="js/jquery-3.4.1.min.js"></script>
  <script src="bootstrap/js/bootstrap.min.js"></script>
  <script type="text/javascript" src="js/roslib.min.js"></script>
  <script type="text/javascript" src="js/eventemitter2.min.js"></script>
  <script type="text/javascript" type="text/javascript">
    
    var ros = new ROSLIB.Ros({
      url: 'ws:' + window.location.hostname +':9090'
    });

    ros.on('connection', function () {
      document.getElementById("status").innerHTML = "Connected";
    });

    ros.on('error', function (error) {
      document.getElementById("status").innerHTML = "Error";
    });

    ros.on('close', function () {
      document.getElementById("status").innerHTML = "Closed";
    });
    // 
    var txt_listener = new ROSLIB.Topic({
      ros: ros,
      name: '/txt_msg',
      messageType: 'std_msgs/String'
    });

    txt_listener.subscribe(function (m) {
      document.getElementById("msg").innerHTML = m.data;
      
    });
    
    

   //* A topic for messaging.
var emotionTopic = new ROSLIB.Topic({
  ros: ros,
  name : '/social/emotion',
  messageType: 'std_msgs/String'
});

   //* A topic for messaging.
   
var speechTopic = new ROSLIB.Topic({
  ros: ros,
  name : '/speech/to_speak',
  messageType: 'std_msgs/String'
});

function speak( testo){
var msg_speak = new ROSLIB.Message({
      data: testo
 });
 start_speak();
 speechTopic.publish(msg_speak,''); // error here als
 console.log(msg_speak);
 normal();
 console.log("speech");   
}

function normal(){
var msg_happy = new ROSLIB.Message({
      data :  'happy'   
 });
 emotionTopic.publish(msg_happy); // error here als
 console.log("happy");   
}

function start_speak(){
var msg_speak = new ROSLIB.Message({
      data :  'speak'   

 });
 emotionTopic.publish(msg_speak); // error here als
 console.log("start speak");   
}


function FaceExpression( face){
  var mymsg = new ROSLIB.Message({
       data :  face   
  });
  emotionTopic.publish(mymsg); // error here als

  console.log(face);   
}




function normal(){
  var mymsg = new ROSLIB.Message({
       data :  'normal'   
  });
  emotionTopic.publish(mymsg); // error here als

  console.log("normal");   
}


function startBlinking(){
  var mymsg = new ROSLIB.Message({
       data :   'startblinking'   
  });
  emotionTopic.publish(mymsg); // error here als

  console.log("startblinking");   
}

function LeggiMurphy(){
 num = Math.floor(Math.random() * 2) + 1;
 //document.getElementById("demo").innerHTML = num;

if (num == 1) {
   speak("la probabilità che qualcosa accada è inversamente proporzionale alla sua desiderabilità");
} else if (num == 2) {
    speak("le cose vengono danneggiate in proporzione al loro valore");
} else {
 speak(",se sei di buon umore, non ti preoccupare. Ti passera");
 }
}

function Barzelletta(){
 num = Math.floor(Math.random() * 2) + 1;
 //document.getElementById("demo").innerHTML = num;

if (num == 1) {
   speak("sapete quale é la differenza tra una formìca e la televisione? , , nessuna , , hanno tutte e due le antenne");
} else if (num == 2) {
    speak(" Scusate.. ma per aprire un buon vino dòc, mi è sufficiente aprire  WORD?");
} else {
 speak(" Che cosa dice un uccello specializzato in elettronica? , , , cip cip");
 }
}

function Presentazione(){
 
   speak(" ciao ragazzi, io sono Martina e sono un robot sociale, sono nata dalla collaborazione tra robotics 3d e l'università la sapienza di Roma. dopo anni di programmazione e miglioramenti estetici, eccomi qua! Posso parlare, cantare, ballare.  posso rispondere alle domande ed interagire con gli essere umani. Vi aspetto piu tardi per divertirci insieme!.");

}
     
    window.onload = function () {
	  
      
	  
    }

  </script>
 
  

</head>

<body>
   
 <!-- Nav Bar -->
 <?php include "../nav.php" ?>
<!-- Eof Nav Bar -->
<div class="container-fluid">
  <div class="row">
    <div class="col-md-12"  style="background-color:yellow;" align="center" ><h2>DEMO INTERFACE </h2></div>

  </div>
  <div class="row">
    <div class="col-md-12"><h1></h1></div>
  </div>
  <div class="row">
    <div class="col-md-1"></div>
    <div class="col-md-4">
      <div class="thumbnail">
        <h1> </h1>
        <h1> </h1>
        <a href="https://www.robotics-3d.com">
          <img style="display: block;margin-left: auto;margin-right: auto;" src="image/LogoRobotics.png" alt="robotics-3d.com"width="40%">

        </a>
      </div>
    </div>
    <div class="col-md-2">  </div>
    <div class="col-md-4">
      <div class="thumbnail">
        <a href="https://robotics.surfweb.eu" target="_blank">
          <img style="display: block;margin-left: auto;margin-right: auto;" src="image/LogoCircolareATNero.png" alt="artigianitecnologici.it" width="40%">
          
        </a>
      </div>
    </div>
    <div class="col-md-1">  </div>
  </div>
  <div class="row">
    <div class="col-md-12">. </div>

  </div>
  <div class="row">
	
		<div class="col-md-3"><button  class="btn btn-outline-danger btn-lg" onclick="Presentazione()">Presentation</button></div>
 	  <div class="col-md-3">
     <a href="../quiz/index.html"  class="btn btn-outline-danger btn-lg" role="button">
        <img src="image/quiz.jpeg" class="img-rounded" alt="laggi di murphy" width="60%">
  			</a>
			
		</div>
    <div class="col-md-3"> 
      <button  class="btn btn-outline-danger btn-lg" onclick="LeggiMurphy()">
  		<img src="image/murphy.jpeg" class="img-rounded" alt="Murpy" width="60%">

			</button>
		</div>
    <div class="col-md-3">
               
				    <button  class="btn btn-outline-danger btn-lg" onclick="Barzelletta()">
                                            <img src="image/barzellette.jpeg" class="img-rounded" alt="Risate" width="80%">
                                     </button>
                   
 	        </div>
 
				
			 
    </div>

   <div class="row">
	
    <div class="col-md-3">
      <a href="../3domande/index.html"   class="btn btn-danger btn-lg btn-block" role="button">Measure Your Intelligence</a>
       
	  </div> 
    <div class="col-md-3">
      <a href="../quiz/index.html"  class="btn btn-danger btn-lg btn-block" role="button">Quiz </a>
      <a href="../quiz/editorquiz.php"  class="btn btn-danger btn-lg btn-block" role="button">Modify Quiz </a>
    </div>

    <div class="col-md-3">
 		    <button  class="btn btn-danger btn-lg btn-block " onclick="LeggiMurphy()">Murphy'S Law</button>
 	  </div>
    <div class="col-md-3">
         <button  class="btn btn-danger btn-lg btn-block " onclick="Barzelletta()">Joke</button>
    </div>
    
  </div>     
  
            
</diV>

 

      <p style="color: bisque;">Connection status: <span id="status"></span></p>
      <p style="color: bisque;"> on /emotion: <span id="emotion"></span></p>
</body>

</html>



