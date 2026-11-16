<style>
  :root {
    --marrtino-accent: rgb(157, 32, 104);
    --marrtino-accent-soft: rgba(157, 32, 104, 0.22);
    --marrtino-panel: rgba(17, 24, 39, 0.88);
    --marrtino-line: rgba(255, 255, 255, 0.14);
    --marrtino-cyan: #38d6ff;
  }

  .marrtino-nav {
    background:
      linear-gradient(90deg, rgba(12, 16, 28, 0.98), rgba(52, 13, 42, 0.96)),
      var(--marrtino-accent);
    border-bottom: 1px solid rgba(255, 255, 255, 0.16);
    box-shadow: 0 10px 34px rgba(157, 32, 104, 0.28);
  }

  .marrtino-nav .navbar-brand {
    font-weight: 700;
    letter-spacing: 0;
  }

  .marrtino-nav .nav-link {
    color: rgba(255, 255, 255, 0.78) !important;
    border-radius: 6px;
    margin: 0 2px;
  }

  .marrtino-nav .nav-link:hover,
  .marrtino-nav .nav-item.active .nav-link {
    color: #fff !important;
    background: rgba(255, 255, 255, 0.1);
  }

  .connection-pill {
    display: inline-flex;
    align-items: center;
    gap: 8px;
    margin: 0;
    padding: 7px 10px;
    color: #fff;
    border: 1px solid rgba(255, 255, 255, 0.18);
    border-radius: 999px;
    background: rgba(0, 0, 0, 0.22);
    font-size: 0.86rem;
    white-space: nowrap;
  }

  .connection-pill::before {
    content: "";
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: var(--marrtino-cyan);
    box-shadow: 0 0 12px var(--marrtino-cyan);
  }
</style>

<!-- Nav Bar -->
<nav class="navbar navbar-expand-md navbar-dark marrtino-nav">
  <a class="navbar-brand" href="#">MARRtino Robot ROS2</a>
  <button class="navbar-toggler" type="button" data-toggle="collapse" data-target="#navbarsExampleDefault" aria-controls="navbarsExampleDefault" aria-expanded="false" aria-label="Toggle navigation">
    <span class="navbar-toggler-icon"></span>
  </button>

  <div class="collapse navbar-collapse" id="navbarsExampleDefault">
    <ul class="navbar-nav mr-auto">
      <li class="nav-item active">
        <a class="nav-link" href="/index.php">Home <span class="sr-only">(current)</span></a>
      </li>
	   <!-- <li class="nav-item">
        <a class="nav-link" href="../bringup/bringup.php" aria-haspopup="true"  >Bringup</a>
      </li> -->
      
      <li class="nav-item">
        <a class="nav-link" href="../social/facerobot.php" aria-haspopup="true"  >Social Interface</a>
      </li>
       <!-- <li class="nav-item dropdown">
        <a class="nav-link dropdown-toggle" href="#" id="navbarDropdown" role="button" data-toggle="dropdown" aria-haspopup="true" aria-expanded="false">
          Facerobot
        </a>
        <div class="dropdown-menu" aria-labelledby="navbarDropdown">
          <a class="dropdown-item" href="../social/facerobot.php">facerobot IT</a>
          <a class="dropdown-item" href="../social/faceroboten.php">facerobot EN</a>
        
      </li> -->
     
      <!-- <li class="nav-item">
        <a class="nav-link" href="../social/facerobot.php" aria-haspopup="true"  >Social Interface</a>
      </li> -->
     
      <li class="nav-item">
        <a class="nav-link" href="../program/blockly_robot.php" aria-haspopup="true"  >Blocky</a>
      </li>
      <li class="nav-item">
        <a class="nav-link" href="../program/python_robot.php" aria-haspopup="true"  >Python</a>
      </li>
      <li class="nav-item">
        <a class="nav-link" href="/config.php" aria-haspopup="true">Config</a>
      </li>
      <li class="nav-item dropdown">
        <a class="nav-link dropdown-toggle" href="#" id="helpDropdown" role="button" data-toggle="dropdown" aria-haspopup="true" aria-expanded="false">
          Help
        </a>
        <div class="dropdown-menu" aria-labelledby="helpDropdown">
          <a class="dropdown-item" href="/help/nodes.php">Nodi del robot</a>
          <a class="dropdown-item" href="/help/voice_interaction.php">Interazione vocale</a>
        </div>
      </li>
      
      <!--
      <li class="nav-item dropdown">
        <a class="nav-link dropdown-toggle" href="#" id="navbarDropdown" role="button" data-toggle="dropdown" aria-haspopup="true" aria-expanded="false">
          Games
        </a>
        <div class="dropdown-menu" aria-labelledby="navbarDropdown">
          <a class="dropdown-item" href="../chess">Chess</a>
          <a class="dropdown-item" href="#">Another action</a>
          <div class="dropdown-divider"></div>
          <a class="dropdown-item" href="#">Something else here</a>
        </div>
      </li>
      <li class="nav-item dropdown">
        <a class="nav-link dropdown-toggle" href="#" id="navbarDropdown" role="button" data-toggle="dropdown" aria-haspopup="true" aria-expanded="false">
          Quiz
        </a>
        <div class="dropdown-menu" aria-labelledby="navbarDropdown">
          <a class="dropdown-item" href="../quizapp">Quiz</a>
          <a class="dropdown-item" href="../quizapp/adminpanel">Admin panel</a>
        
      </li>-->
	  <!-- <li class="nav-item">
        <a class="nav-link" href="../social/demorobotics.php" tabindex="-1" >Demo Social</a>
      </li>
      <li class="nav-item">
        <a class="nav-link" href="../tombola/index.html" tabindex="-1" >Tombola !!</a>
      </li> -->
      <li class="nav-item">
        <a class="nav-link" href="../social/navigation.php" tabindex="-1" >Telepresence</a>
      </li>
	   <li class="nav-item">
        <a class="nav-link" href="../social/vision.php" tabindex="-1" >Vision</a>
      </li>

      <!-- <li class="nav-item">
        <a class="nav-link" href="../bringup/config.php" tabindex="-1" >Configuration</a>
      </li> -->

	 
    </ul>
	<p class="connection-pill">ROS Bridge <span id="status">Connecting</span></p>
	
	
   
  </div>
</nav>
<!-- Eof Nav Bar -->
