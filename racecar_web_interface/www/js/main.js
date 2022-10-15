// Main javascript code for the racecar web interface (APP3)

// Define some global variables
var rbServer = null;
var cmdVelTopic = null;
var racecar_ip_adress = null;

// Some initializations after the page has been shown
$(document).ready(function(){
    document.getElementById("log").value = 'Default text\n'
});


// Define some functions:

function connectROS() {
    // This function connects to the rosbridge server
    racecar_ip_adress = document.getElementById("racecar_ip_adress").value;

    rbServer = new ROSLIB.Ros({
        // Assuming ros server IP is 10.42.0.1
        url : 'ws://' + racecar_ip_adress + ':9090'             //'ws://localhost:9090'          //10.42.0.1:9090'
    });

    rbServer.on('connection', function(){
        document.getElementById("console").value += 'Connected to websocket server via adress : ' + racecar_ip_adress + '\n\n';
        console.log('Connected to websocket server with adress : ' + racecar_ip_adress);

        document.getElementById("videoStream").src = "http://localhost:8080/stream?topic=/racecar/raspicam_node/image&type=ros_compressed";


        // These lines create a topic object as defined by roslibjs
        cmdVelTopic = new ROSLIB.Topic({
            ros : rbServer,
            name : '/racecar/cmd_vel_abtr_2',
            messageType : 'geometry_msgs/Twist'
        });
    });

    rbServer.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
    });

    rbServer.on('close', function() {
        console.log('Connection to websocket server closed');
    });
}

function text_area_manager(arg) {
    // This function manages the text area
    if (arg == 'clear') {
        document.getElementById("console").value = "";
    }
    else if (arg == 'foward') {
        document.getElementById("console").value += "En avant!\n";
        twist.linear.x = 1;
    }
    else if (arg == 'stop') {
        document.getElementById("console").value += "Arret!\n";
        twist.linear.x = 0;
    }
}

function disconnectROS() {
    // This function disconnects from the rosbridge server
    rbServer.close();
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

//Publishing loop cmd_vel at 5 Hz
setInterval(function(){
    if(cmdVelTopic != null)
    {
        cmdVelTopic.publish(twist);
    }
}, 200);