      // Define some global variables
      var rbServer = null;
      var cmdVelTopic = null;
      
        // Textarea for logging
        const textarea = document.getElementById("log");
        textarea.scrollTop = textarea.scrollHeight;

        // Buttons and event handlers
        const conn_btn = document.getElementById("connect_button");
        const clear_btn = document.getElementById("clear_button");
        var arrow = {
            'up': document.getElementById("arrow_up"),
            'down': document.getElementById("arrow_down"),
            'left': document.getElementById("arrow_left"),
            'right': document.getElementById("arrow_right")
        };

      //Some initializations after the page has been shown
      $(document).ready(function(){
      });

      // Define some functions
      function connectROS() {
        // This function connects to the rosbridge server
        var ip_addr = document.getElementById("ip_address").value;
        console.log(ip_addr);

	    rbServer = new ROSLIB.Ros({
          // Assuming ros server IP is 10.42.0.1
	      url : 'ws://'+ ip_addr +':9090'
	    });

        rbServer.on('connection', function(){
            console.log('Connected to websocket server.');
            textarea.value += 'Connected to websocket server with address' + ip_addr;

            // These lines create a topic object as defined by roslibjs
            cmdVelTopic = new ROSLIB.Topic({
                ros : rbServer,
                name : '/racecar/cmd_vel_abtr_2',
                messageType : 'geometry_msgs/Twist'
            });

            camTopic = new ROSLIB.Topic({
                ros : rbServer,
                name: '/racecar/camera/rgb/image_raw'
            });
	    });

        rbServer.on('error', function(error) {
          console.log('Error connecting to websocket server: ', error);
	    });

	    rbServer.on('close', function() {
          console.log('Connection to websocket server closed.');
	    });
      }

      function eraseStatus() {
        document.getElementById('log').value = '';
        console.log('Clearing log');
      }

      function move(key) {
        arrow[key].style.backgroundColor = "red";
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
