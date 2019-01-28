# Balloon Popping Rover
In this project, we are required to combine a Raspberry Pi and a PIC microcontroller to let the Dagu Rover 5 search for a red balloon and move forward to pop it.
* The Raspberry Pi will act as the eyes and brain of the system, processing each
picture and detecting what should the rover do next
* Once a picture is taken, the processing starts and F, L, and R mapping to forward,
left and right is sent to the PIC serially so that it can move closer to a balloon
* When forward is active, the rover should know when to stop using an ultrasonic
sensor so that the arm can pop the balloon
* After popping a balloon, the rover continues the search to pop all 3 balloons

## Tools Used
* Raspberry Pi 3+ Model B
* PIC18F4550
* Dagu Rover 5 (4 motors, 4 encoders)
* L298 H-Bdridge
* MG995 Tower Pro Servo Motor
* Micro Servo 99
* HC-SR04 Ultrasonic Sensor
* PicKit 3
* MPLAB X IDE
* SimpleCV
