// Main javascript code for the racecar web interface (APP3)
// Define some global variables
var rbServer = null;
var cmdVelTopic = null;
var racecar_ip_adress = null;
var racecar_username = null;  
var direction = null;   // 'foward', 'backward', 'left', 'right' with arrows
document.getElementById("connStatus").style.color = "red";
document.getElementById("connStatus").innerHTML = "Déconnecté";
document.getElementById("statusIcon").style.fill = "red";
var flag_connected = false;
localStorage.setItem("flag_connected",flag_connected);
// Some initializations after the page has been shown
$(document).ready(function(){
    // document.getElementById("console").value = 'Default text\n'
});
// Define some functions:
function connectROS() {
    // This function connects to the rosbridge server
    racecar_ip_adress = document.getElementById("racecar_ip_adress").value;
    racecar_username = document.getElementById("racecar_username").value;
    rbServer = new ROSLIB.Ros({
        // Assuming ros server IP is 10.42.0.1
        url : 'ws://' + racecar_ip_adress + ':9090'             //'ws://localhost:9090'          //10.42.0.1:9090'
    });
    rbServer.on('connection', function(){
        flag_connected = true;
        // Ouverture du popup:
        document.getElementById("conn_msg").innerHTML = 'Connecté à : ' + racecar_username + " avec l'adresse : " + racecar_ip_adress;
        $('#Connexion_modal').modal({
            backdrop: 'static',
            keyboard: false, 
            show: true
        }); 
        // Collapse navbar when connected:
        $('#navbarToggleExternalContent').collapse('hide');
        document.getElementById("console").innerHTML = 'Connecté à : ' + racecar_username + " avec l'adresse : " + racecar_ip_adress;
        console.log('Connecté à : ' + racecar_username + " avec l'adresse : " + racecar_ip_adress);
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

        camTopic.subscribe(function(message) {
            document.getElementById("videoStream").src = 'data:image/jpeg;base64,' + message.data;
          });

        // document.getElementById("videoStream").src = "http://localhost:8080/stream?topic=/racecar/raspicam_node/image&type=ros_compressed";
        document.getElementById("connStatus").style.color = "green";
        document.getElementById("connStatus").innerHTML = "Connecté";
        document.getElementById("statusIcon").style.fill = "green";
    });
    rbServer.on('error', function(error) {
        flag_connected = false;
        console.log('Error connecting to websocket server: ', error);
        document.getElementById("connStatus").style.color = "red";
        document.getElementById("connStatus").innerHTML = "Déconnecté";
        document.getElementById("statusIcon").style.fill = "red";
    });
    rbServer.on('close', function() {
        flag_connected = false;
        console.log('Connection to websocket server closed');
        document.getElementById("connStatus").style.color = "red";
        document.getElementById("connStatus").innerHTML = "Déconnecté";
        document.getElementById("statusIcon").style.fill = "red";
    });
}
function disconnectROS() {
    // This function disconnects from the rosbridge server
    rbServer.close();
    document.getElementById("console").innerHTML = "Non connecté";
    document.getElementById('racecar_username').value = '';
    document.getElementById('racecar_ip_adress').value = '';
    document.getElementById("videoStream").src = 'icons/no_camera.png';
    console.log('Déconnecté à : ' + racecar_username + " avec l'adresse : " + racecar_ip_adress);
    // Ouverture du popup:
    document.getElementById("disconn_msg").innerHTML = 'Déconnecté à : ' + racecar_username + " avec l'adresse : " + racecar_ip_adress;
    $('#Disconnection_modal').modal({
        backdrop: 'static',
        keyboard: false, 
        show: true
    }); 
    // Ouverture de la navbar:
    $('#navbarToggleExternalContent').collapse('show');
}
// These lines create a message that conforms to the structure of the Twist defined in our ROS installation
// It initalizes all properties to zero. They will be set to appropriate values before we publish this message.
var twist = new ROSLIB.Message({
    linear : {
        x : 0.0,
        y : 0.0,
        z : 1.0
    },
    angular : {
        x : 0.0,
        y : 0.0,
        z : 0.0
    }
});
//Publishing loop cmd_vel at 5 Hz
setInterval(function(){
    if(cmdVelTopic != null)
    {
      twist.linear.x=parseFloat(localStorage.getItem("vitesse"));
      twist.angular.z=parseFloat(localStorage.getItem("angle"));
        cmdVelTopic.publish(twist);
    }
}, 200);
