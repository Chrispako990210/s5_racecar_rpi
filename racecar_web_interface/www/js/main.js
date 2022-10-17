
// Main javascript code for the racecar web interface (APP3)

// Define some global variables
var rbServer = null;
var cmdVelTopic = null;
var racecar_ip_adress = null;
var racecar_username = null;  
var direction = null;   // 'foward', 'backward', 'left', 'right' with arrows



var flag_connected = false;

// Some initializations after the page has been shown
$(document).ready(function(){
    document.getElementById("log").value = 'Default text\n'
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
        document.getElementById("conn_msg").innerHTML = 'Connected to : ' + racecar_username + ' with adresss : ' + racecar_ip_adress;
        $('#Connexion_modal').modal({
            backdrop: 'static',
            keyboard: false, 
            show: true
        }); 

        // Collapse navbar when connected:
        $('#navbarToggleExternalContent').collapse('hide');

        document.getElementById("console").value += 'Connected to : ' + racecar_username + '\n';
        document.getElementById("console").value += 'Connected to websocket server with adress : ' + racecar_ip_adress + '\n\n';
        document.getElementById("videoStream").src = "http://localhost:8080/stream?topic=/racecar/raspicam_node/image&type=ros_compressed";

        console.log('Connected to websocket server with adress : ' + racecar_ip_adress);

        // These lines create a topic object as defined by roslibjs
        cmdVelTopic = new ROSLIB.Topic({
            ros : rbServer,
            name : '/racecar/cmd_vel_abtr_2',
            messageType : 'geometry_msgs/Twist'
        });
    });


    rbServer.on('error', function(error) {
        flag_connected = false;
        console.log('Error connecting to websocket server: ', error);
    });


    rbServer.on('close', function() {
        flag_connected = false;
        console.log('Connection to websocket server closed');
    });
}

function disconnectROS() {
    // This function disconnects from the rosbridge server
    rbServer.close();

    text_area_manager('clear');
    document.getElementById('racecar_username').value = '';
    document.getElementById('racecar_ip_adress').value = '';
    document.getElementById("videoStream").src = 'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAOEAAADhCAMAAAAJbSJIAAAAb1BMVEX///8AAACXl5fQ0NA4ODhMTEzt7e329vb8/PwuLi7n5+eioqLCwsJjY2OdnZ0FBQXR0dFGRkZ1dXVqamrZ2dkkJCS6urq0tLR7e3uqqqpVVVWGhoZdXV3f398UFBSRkZEdHR0YGBhQUFA9PT1wcHAuDD5UAAAD3UlEQVR4nO2daXuqMBBGibu4tHWtW7W2//833mqrNwPDAC6QTN7zSQnk4SgmJJKZKAIAAAAAAAAAAAAAAAAAAAAAAAAAAACETTxcjRs1MV42Ow+yWDVWXFXd5cjUzmR4v9/6XNM4uX33XbPblc193+T4Uk+DbO5s63RKsund7Lfr/68m5rxd4dZrdWNXsrlu7r3VJZLN+y1+rx+kjv5le+elJguRt/KCk2Qdf9vjOs6/AH3RJs0wXcVvQbf6cy/IVxm/LtfT/Rb1mRJHmBYXXLIVnItS165LrAr6ddr88aeyZrWnXJZiff8g6/BT4Udy42gwbNbEopHqtUYF/OJW5gf0U7pKbBrcfjPxEDrviROa5R6yFy6Bn2LaE667z3fII6Yt3zZn99lcEDTJLuSm24jHQ1sN+ZeY/MpThuR2+7sig1zI7yo1BrLIbSajnv1uXplBHh37tFqZu/WmnNTI/nzoZ/BaoUMOZKST1TYs2G9tQa4AUtMNN7rPwz5nfhzVXXN+0x69xiN7VL+sWELEbkDYH2Kylzvz0jwVEUO71XrQHNBjsNt4poWP2fHs347E8GC/cQl7QJe+rWlwfvPLvQExtPr7soOx52I38utE2Y4dDO2v5cTQep3dKNeCdWZtWkImYq5nb805eW7YTI0UTgzsXbw27LGD2SNtJH02ZCZiTLqb89ewy85ZT1PDPW8N+YkY5obHU0N+IoYd7vlpyE7EHPjxv4+G/ETMPuNQ/wyP7L9GrV3Wof4ZsgyyD1VheJQGQhoM5aGs/4ajnClP7w1z/xz23HCSf6jXhp9F5gN9Nszq4yn+GvYz+3iKt4bS7D7BU8N28clOPw3LTFf7aLgt9bemh4YlHwDzz/BY8lD/DNv5exNg6AgwFIChI8BQAIaOAEMBGDoCDAVg6AgwFIChI8BQAIaOAEMBGDoCDAVg6AgwFIChI8BQAIaOAEMBGDoCDAVg6AgwFIChI8BQAIaOEJSh/mcT9T9fGsAzwkb/c94mgGf1jf71FiaANTNG/7onE8DaNaN//eEJ7WtITQDrgI3+tdwmgPX4RllMBf1xMfTHNgkhPk0UQIyhSH+cqCiAWF+R/nhtUQAx9yL9cRNPlIt9qT9+6cF+4xIPi0GrP46w/ljQ+uN564/Jrj+uvv7cCAHkt3AxRwlNOnJvjpIA8sykcwVtleUKcj3fU7GfjZjvye2cXUVvtKScXQHkXQsgd57+/IcB5LDUm4fUngDRmUuWpE/UmA/4M1mkLqdznCpSk5f7PHDM+r9fR271aDZzYJgEAAAAAAAAAAAAAAAAAAAAAAAAAAAAqJZ/5vND2HdQ9r4AAAAASUVORK5CYII=';

    console.log('Disconnected from :' + racecar_ip_adress);
    
    // Ouverture du popup:
    document.getElementById("disconn_msg").innerHTML = 'Disconnected to : ' + racecar_username + ' with adresss : ' + racecar_ip_adress;
    $('#Disconnection_modal').modal({
        backdrop: 'static',
        keyboard: false, 
        show: true
    }); 

    // Ouverture de la navbar:
    $('#navbarToggleExternalContent').collapse('show');
}


function text_area_manager(arg) {
    // This function manages the text area
    // TODO: remove switch case if not necessary
    switch (arg) {
        case 'clear':
            document.getElementById("console").value = "";
            break;        
    }
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
      twist.linear.x=parseFloat(localStorage.getItem("vitesse"));
      twist.angular.z=parseFloat(localStorage.getItem("angle"));
        cmdVelTopic.publish(twist);
    }
}, 200);
