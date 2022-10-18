//<canvas id="canvas" name="game"></canvas>
var canvas, ctx;
var flag_connected=false;         
// var cmd = {
//     linear : {
//         x : 0.0,
//         y : 0.0,
//         z : 0.0
//     },
//     angular : {
//         x : 0.0,
//         y : 0.0,
//         z : 0.0
//     }
// };
window.addEventListener('load', () => {
    canvas = document.getElementById('canvas');
    ctx = canvas.getContext('2d');          
    resize(); 
    document.addEventListener('mousedown', startDrawing);
    document.addEventListener('mouseup', stopDrawing);
    document.addEventListener('mousemove', Draw);
    document.addEventListener('touchstart', startDrawing);
    document.addEventListener('touchend', stopDrawing);
    document.addEventListener('touchcancel', stopDrawing);
    document.addEventListener('touchmove', Draw);
    window.addEventListener('resize', resize);
});
var width, height, radius, x_orig, y_orig;
var coordHeight=2;
var coordWidth=2;
function resize() {
    width = window.innerWidth/2;
    heigth = window.innerHeight;
    radius = 50;
    height = radius* 6.5;
    ctx.canvas.width = width;
    ctx.canvas.height = height;
    background();
    joystick(width / coordWidth, height / coordHeight);
}
function updateCanvas() {
    flag_connected=localStorage.getItem("flag_connected");
    paint=flag_connected;
    console.log(flag_connected);
}
function background() {
    x_orig = width / coordWidth;
    y_orig = height / coordHeight;
    ctx.beginPath();
    ctx.arc(x_orig, y_orig, radius + 20, 0, Math.PI * 2, true);
    ctx.fillStyle = '#ECE5E5';
    ctx.fill();
}
function joystick(width, height) {
    ctx.beginPath();
    ctx.arc(width, height, radius, 0, Math.PI * 2, true);
    if(flag_connected){
    ctx.fillStyle = '#F08080';
    ctx.fill();
    ctx.strokeStyle = '#F6ABAB';
    }else{
    ctx.fillStyle = '#747981';
    ctx.fill();
    ctx.strokeStyle = '#787d85';
    }
    ctx.lineWidth = 8;
    ctx.stroke();
}
let coord = { x: 0, y: 0 };
let paint = false;
function getPosition(event) {
    var mouse_x = event.clientX || event.touches[0].clientX;
    var mouse_y = event.clientY || event.touches[0].clientY;
    var rect = canvas.getBoundingClientRect();
    coord.x = mouse_x - (rect.x);
    coord.y = mouse_y - (rect.y);
}
function is_it_in_the_circle() {
    var current_radius = Math.sqrt(Math.pow(coord.x - x_orig, 2) + Math.pow(coord.y - y_orig, 2));
    if (radius >= current_radius) return true
    else return false
}
function joystickToTwist(speed,angle_in_degrees){
    if(angle_in_degrees<=90){
        vitesse=(speed/100)*5;
        angle=((angle_in_degrees)/90-1);
    }else if(angle_in_degrees<=180){
            vitesse=(speed/100)*5;
            angle=-1*(90/(angle_in_degrees)-1);
        }else{
            vitesse=(speed/100)*5*-1;
            angle=270/(angle_in_degrees)-1;
        }
            localStorage.setItem("vitesse",vitesse);
            localStorage.setItem("angle",angle);
        }
function startDrawing(event) {
    if(flag_connected){
        paint=true;
        getPosition(event);
        if (is_it_in_the_circle()) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            background();
            joystick(coord.x, coord.y);
            Draw();
        }
    }
}
function stopDrawing() {
    paint = false;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    background();
    joystick(width / coordWidth, height / coordHeight);
    joystickToTwist(0,90)
}
function Draw(event) {
    if (paint) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        background();
        var angle_in_degrees,x, y, speed;
        var angle = Math.atan2((coord.y - y_orig), (coord.x - x_orig));
        if (Math.sign(angle) == -1) {
            angle_in_degrees = Math.round(-angle * 180 / Math.PI);
        }
        else {
            angle_in_degrees =Math.round( 360 - angle * 180 / Math.PI);
        }
        if (is_it_in_the_circle()) {
            joystick(coord.x, coord.y);
            x = coord.x;
            y = coord.y;
        }
        else {
            x = radius * Math.cos(angle) + x_orig;
            y = radius * Math.sin(angle) + y_orig;
            joystick(x, y);
        }
        getPosition(event);
        var speed =  Math.round(100 * Math.sqrt(Math.pow(x - x_orig, 2) + Math.pow(y - y_orig, 2)) / radius);
        var x_relative = Math.round(x - x_orig);
        var y_relative = Math.round(y - y_orig);
        joystickToTwist(speed,angle_in_degrees)
    }
}
