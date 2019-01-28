#!/usr/bin/env python3
########################################################################
# Filename    : Project.py
# Description : Balloon Popper Rover
# Author      : Elliott
########################################################################
import subprocess
import SimpleCV
import time
import serial
import RPi.GPIO as GPIO
from multiprocessing import Process

ser=serial.Serial("/dev/ttyS0")
ser.baudrate=9600
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(3,GPIO.IN, pull_up_down= GPIO.PUD_UP)
global moved        # variable to check whether the arm has moved or not


    
def search():
    ser.write('P'.encode('UTF-8'))
    time.sleep(0.4)
    ser.write('S'.encode('UTF-8'))
        
def stop():
    ser.write('S'.encode('UTF-8'))
    
OFFSE_DUTY = 0.5        #define pulse offset of servo
SERVO_MIN_DUTY = 2.5+OFFSE_DUTY     #define pulse duty cycle for minimum angle of servo
SERVO_MAX_DUTY = 12.5+OFFSE_DUTY    #define pulse duty cycle for maximum angle of servo
smallServoPin = 12
bigServoPin = 32

def map( value, fromLow, fromHigh, toLow, toHigh):
    return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow

trigPin = 16
echoPin = 18
MAX_DISTANCE = 220          #define the maximum measured distance
timeOut = MAX_DISTANCE*60   #calculate timeout according to the maximum measured distance

def pulseIn(pin,level,timeOut): # function pulseIn: obtain pulse time of a pin
    t0 = time.time()
    while(GPIO.input(pin) != level):
        if((time.time() - t0) > timeOut*0.000001):
            return 0;
    t0 = time.time()
    while(GPIO.input(pin) == level):
        if((time.time() - t0) > timeOut*0.000001):
            return 0;
    pulseTime = (time.time() - t0)*1000000
    return pulseTime
    
def getSonar():     #get the measurement results of ultrasonic module,with unit: cm
    GPIO.output(trigPin,GPIO.HIGH)      #make trigPin send 10us high level 
    time.sleep(0.00001)     #10us
    GPIO.output(trigPin,GPIO.LOW)
    pingTime = pulseIn(echoPin,GPIO.HIGH,timeOut)   #read plus time of echoPin
    distance = pingTime * 340.0 / 2.0 / 10000.0     # the sound speed is 340m/s, and calculate distance
    return distance

def setup():
    global p
    global w
    print ('Program is starting...')
    GPIO.setmode(GPIO.BOARD)       #numbers GPIOs by physical location
    GPIO.setup(trigPin, GPIO.OUT)   #
    GPIO.setup(echoPin, GPIO.IN)    #
    GPIO.setup(smallServoPin, GPIO.OUT)   # Set smallServoPin's mode is output
    GPIO.setup(bigServoPin, GPIO.OUT) # Set bigServoPin's mode as output
    GPIO.output(smallServoPin, GPIO.LOW)  # Set smallServoPin to low
    GPIO.output(bigServoPin, GPIO.LOW) # Set bigServoPin to Low

    p = GPIO.PWM(smallServoPin, 20)     # set Frequency to 40Hz of small servo to rotate while keeping the ultrasonic healthy to catch distances
    p.start(0)                     # Duty Cycle = 0
    w = GPIO.PWM(bigServoPin, 50)   # frequency of the servo that is mnoving the arm
    w.start(0)
    
def servoWriteP(angle):      # make the ultrasonic servo rotate to specific angle (0-180 degrees)
    if(angle<0):
        angle = 0
    elif(angle > 90):
        angle = 90
    p.ChangeDutyCycle(map(angle,0,90,SERVO_MIN_DUTY,SERVO_MAX_DUTY))#map the angle to duty cycle and output it

def servoWriteW(angle):  # make the ultrasonic servo rotate to specific angle (0-180 degrees)
    if(angle<0):
        angle = 0
    elif(angle > 180):
        angle = 180
    w.ChangeDutyCycle(map(angle,0,180,SERVO_MIN_DUTY,SERVO_MAX_DUTY))

def moveSensorServoUp(angle): 
    distance = getSonar()
    print("The distance is : %.2f cm"%(distance))
    for dc in range(0,angle+1, 1):
        #print("Up")
        servoWriteP(dc)
        time.sleep(0.001)
        if(distance<32 and distance>2): # default sensor value 0
            global moved
            moveArm()
            moved = 1
            print(moved)
            break
        
    time.sleep(0.5)

def moveSensorServoDown(angle):
    distance = getSonar()
    print("The distance is : %.2f cm"%(distance))
    for dc in range(angle+1, -1, -1): #make servo rotate from 180 to 0 deg
        servoWriteP(dc)
        time.sleep(0.001)
        #print("Down")
        if(distance<32 and distance>2):
            global moved
            moveArm()
            moved = 1
            print(moved)
            break
        
    time.sleep(0.5)
    
def moveArm():
    stop()
    for i in range (2):
        for dc in range(0, 181, 1):   #make servo rotate from 0 to 180 deg
            servoWriteW(dc)     # Write to servo
            time.sleep(0.001)
        time.sleep(0.5)
        for dc in range(181, -1, -1): #make servo rotate from 180 to 0 deg
            servoWriteW(dc)
            time.sleep(0.001)
        time.sleep(0.5)

def loop():
    GPIO.setup(11,GPIO.IN)
    print("post: " + str(moved))
    while(moved == 0):
        print("new " + str(moved))
        distance = getSonar()
        #if(distance>32):
        moveSensorServoDown(60)
        #if(distance>32):
        moveSensorServoUp(60)
        
     

if __name__ == '__main__':     #program start from here
    print("Jello")
    setup()
    x=GPIO.input(3)
    while(1):
        
        
                  
        subprocess.call("raspistill -n -w %s -h %s -o project.bmp" % (640, 480), shell=True)
        img = SimpleCV.Image("project.bmp")
        dist = -img.hueDistance(SimpleCV.Color.RED)
        filt= dist.stretch(225,255)
        erode=filt.erode(2)
        blobs=erode.findBlobs()
        erode.save("project_red.bmp")
        if blobs:
                    circles=blobs.filter([b.isCircle(0.65) for b in blobs])
                    if circles:
                                x= circles[-1].x
                                print (x)                  
                                if x<285:
                                                ser.write('R'.encode('UTF-8')) # because of crossed wiring we changed the left and right
                                                d=320-x
                                                d=d/35
                                                m=str(d)
                                                ser.write(m.encode('UTF-8'))
                                                print("Left")
                                                
                                                
                                else:
                                        if x>355:
                                                ser.write('L'.encode('UTF-8'))
                                                d=x-320
                                                d=d/35
                                                m=str(d)
                                                ser.write(m.encode('UTF-8'))
                                                print("Right")
                                        else:
                                                ser.write('F'.encode('UTF-8'))
                                                print("pre")
                                                moved = 0
                                                loop()
                                            
                            
                                                
                                                print("post")
                                                
                                                
                    else:
                                print ('no')
                                search()
        
                                
