// Define some global variables
var rbServer = null;
var cmdVelTopic = null;
var ip_addr = null;
var user = null;
var direction = null;
var flag_connected = false;


//Some initializations after the page has been shown
$(document).ready(function(){
  document.getElementById("log").value = 'Default text\n';
});

// Define some functions
function connectROS() {
  // This function connects to the rosbridge server
  ip_addr = document.getElementById("ip_adresse").value;
  user = document.getElementById("username").value;

  rbServer = new ROSLIB.Ros({
      // Assuming ros server IP is 10.42.0.1
    url : 'ws://'+ ip_addr +':9090'
  });

  rbServer.on('connection', function(){
      flag_connected = true;
      document.getElementById("conn_msg").innerHTML = 'Connected to : ' + user + '@' + ip_addr + '\n';
      $('#Connexion_modal').modal({
        backdrop: 'static',
        keyboard: false,
        show: true
      });

      $('#navbarToggleExternalContent').collapse('hide');

      document.getElementById("log").value += 'Connected to : ' + user + '@' + ip_addr + '\n';

      // These lines create a topic object as defined by roslibjs
      cmdVelTopic = new ROSLIB.Topic({
          ros : rbServer,
          name : '/racecar/cmd_vel_abtr_2',
          messageType : 'geometry_msgs/Twist'
      });

      camTopic = new ROSLIB.Topic({
          ros : rbServer,
          name: '/racecar/raspicam_node/image/compressed',
          messageType: 'sensor_msgs/CompressedImage'
      });

      lidarTopic = new ROSLIB.Topic({
          ros : rbServer,
          name: '/racecar/scan',
          messageType: 'sensor_msgs/LaserScan'
      });

      camTopic.subscribe(function(message) {
        getElementById("camera_feed").src = 'data:image/jpeg;base64,' + message.data;
      });

      lidarTopic.subscribe(function(message) {
        getElementById("lidar_feed").src = 'data:image/jpeg;base64,' + message.data;
  });
});

  rbServer.on('error', function(error) {
    flag_connected = false;
    console.log('Error connecting to websocket server: ', error);
  });

  rbServer.on('close', function() {
    flag_connected = false;
    console.log('Connection to websocket server closed.');
  });
}

function disconnectROS() {
  rbServer.close();
  document.getElementById("log").value += 'Disconnected from : ' + user + '@' + ip_addr + '\n';
  document.getElementById("ip_adresse").value = '';
  document.getElementById("username").value = '';
  document.getElementById("camera_feed").src = 'icons/disconnected.png';

  document.getElementById("disconn_msg").innerHTML = 'Disconnected from : ' + user + '@' + ip_addr + '\n';
  $('#Disconnection_modal').modal({
    backdrop: 'static',
    keyboard: false,
    show: true
  });
  $('#navbarToggleExternalContent').collapse('show');
}

function eraseStatus() {
  document.getElementById('log').value = '';
  console.log('Clearing logs...');
}


// These lines create a message that conforms to the structure of the Twist defined in our ROS installation
// It initalizes all properties to zero. They will be set to appropriate values before we publish this message.
var twist = new ROSLIB.Message({
      linear : {
          x : 0.0,
          y : 0.0,
          z : 0.0
      },
      angular : {
          x : 0.0,
          y : 0.0,
          z : 0.0
      }
});


  function scale(value, lim_in_min, lim_in_max, lim_out_min, lim_out_max) {
    return (value - lim_in_min) * (lim_out_max - lim_in_max) / (lim_out_min - lim_in_min) + lim_in_max;
  }
  var speed = 5.0; // Volts

//Publishing loop cmd_vel at 5 Hz
setInterval(function(){
    // Ajouter logic pour atteindre valeurs minimum
      if(cmdVelTopic != null)
      {
        twist.linear.x = parseFloat(localStorage.getItem("vitesse"));
        twist.angular.z = parseFloat(localStorage.getItem("angle"));
        cmdVelTopic.publish(twist);
      }
}, 200);



          