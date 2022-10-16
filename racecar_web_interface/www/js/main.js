 // Define some global variables
 var rbServer = null;
 var cmdVelTopic = null;
 var ip="localhost";

 //Some initializations after the page has been shown

 $(document).ready(function(){
   document.getElementById("log").value = 'Default text\n'
 });

 // Define some functions
 function connectROS() {
   // This function connects to the rosbridge server
   //ip=document.getElementById("adressIP").value;
   rbServer = new ROSLIB.Ros({
     // Assuming ros server IP is 10.42.0.1
     url : 'ws://'+ip+':9090'
     
   });

   

   rbServer.on('connection', function(){
       console.log('Connected to websocket server.');
       document.getElementById("videoStream").src="http://localhost:8080/stream?topic=/racecar/raspicam_node/image&type=ros_compressed";
       

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
     console.log('Connection to websocket server closed.');
   });
 }

 var twist = new ROSLIB.Message({
      linear : {
           x : 0.0,
           y : 0.0,
           z : 0.0
       },
       angular : {
           x : 0.0,
           y : 0.0,
           z : 20.0
       }
 });
 
 //Publishing loop cmd_vel at 5 Hz
 setInterval(function(){
      if(cmdVelTopic != null)
      {
        twist.linear.x=parseFloat(localStorage.getItem("vitesse"));
        twist.angular.z=parseFloat(localStorage.getItem("angle"));
         cmdVelTopic.publish(twist);
         console.log(twist);
         
      }
 }, 200);

