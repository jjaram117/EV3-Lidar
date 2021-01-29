#!/usr/bin/env python3
import serial, time, struct
import csv
import math
from operator import itemgetter 
import statistics
import sys
    
#Serial communication protocols for Lidar 
Start_Scan = b"\xA5\x20" #Begins scanning
Force_Scan = b"\xA5\x21" #Overrides anything preventing a scan
Health = b"\xA5\x52" #Returns the state of the Lidar
Stop_Scan = b"\xA5\x25" #Stops the scan
RESET = b"\xA5\x40" #Resets the device
Motor = b"\xA5\xF0" #Sets motor speed
MotorFast = b'\xa5\xf0\x02\x94\x02\xc1'
Motor0 = b"\xA5\xF0\x02\x00\x00\x57" #A5F0 0200 0057
Rate = b'\xA5\x59'

##--------------Lidar Serial Communication
def chksm(payload):
    cksm = 0
    for elem in payload:
        cksm ^= elem
    return struct.pack('=B',cksm)

def serialComm(cmd, num= 0):
    payload = cmd + chksm(cmd)
    ser.write(payload)
    print(payload)
    reply = b''
    for i in range(num):
        reply += ser.readline()
    if num > 0:
        print(reply)
    return reply

def translate(payload):
    quality = payload[0]>>2
    S = payload[0] & 0b1
    notS = payload[0] & 0b10
    C = payload[1] & 0b1
    angle = struct.unpack('<H',payload[1:3])[0]/2/64  # divide by 2 gets rid of C bit
    dist = struct.unpack('<H',payload[3:5])[0]/4000
    return quality,S,notS,C,angle,dist

def GrabPts(num): #Request Lidar to collect "num" number of points
    run = 0 #Tracker variable

    #Preallocation for the 4 variables
    x = [0]*num
    y = [0]*num
    a = [0]*num
    d = [0]*num

    #Wiping Lidar buffer
    ser.flushInput()
    ser.flushOutput()

    while run < num: #Run Lidar for desired number of points. Generate all angle, distance, and calculated x and y data       
        reading = ser.read(5)
        try: 
            (quality, S, notS, C, angle, dist) = translate(reading)
        except Exception as e:
            print(e) 
            continue 

        if quality > 10:
            if dist == 0: #Eliminating noise calculated to be at distance (d) = 0
                continue
            else:
                #Append polar coordinates to running variables
                a[run] = (angle)
                d[run] = (dist)
                
                #Converting from polar to cartesian coordinates
                ang = angle*3.14/180 #radians to degrees
                x_calc = dist*math.cos(ang) 
                y_calc = dist*math.sin(ang) 

                #Append cartesian coordinates to running variables
                x[run] = (x_calc)
                y[run] = (y_calc)

                run += 1 #Update Run variable
    return (x,y,a,d) #Return all variables upon completion

def LidarComm(): #Compact method of initiating communication with Lidar
    serialComm(Start_Scan)
    while ser.read() != b'\xA5':
        time.sleep(0.01)
    reply = ser.read(6)
    if (reply == b'\x5A\x05\x00\x00\x40\x81'):
        print('starting...')
    else:
        print('incorrect reply')

def SaveData():
    name = 'currData' #Adjustable name for temporary csv file
    
    #Saving all data to the temporary .csv file
    filename = "/home/robot/" + name + ".csv" #Rename directory address on EV3 as desired 
    with open(filename, 'w', newline='') as f:
            writer = csv.writer(f, delimiter = ',')
            writer.writerows(zip(x,y,a,d))
    time.sleep(.1)

def opt1(num): #Have lidar collect desired number of points
    global x,y,a,d

    LidarComm() #Initiating Lidar

    (x,y,a,d) = GrabPts(num)

    serialComm(Stop_Scan)
    ser.read(ser.inWaiting()) #Flush Lidar buffer
    SaveData() #Save data to currData.csv


##----------------------------SETUPS----------------------------##
port = "/dev/ttyUSB0" #USB Port for EV3. Rename as necessary

ser = serial.Serial(port, 115200, timeout = 1)
ser.setDTR(False)

#Initial Serial commands
serialComm(RESET,3)
serialComm(Health,1)
serialComm(Rate,1)

#Set Motor speed
speed = 800  #Speed between 1 and 1023
motorSpeed = Motor + b'\x02'+struct.pack('<H',speed)  #2 byte payload, little endian of the desired speed

#Assigning arguments recieved from main script to communicate with Lidar
endComm= int(sys.argv[1])
num = int(sys.argv[2])

##------MAIN LOOP------##
while True: 
    if endComm == 1: #Turn off lidar and end this script
        serialComm(Motor0)
        break

    elif endComm == 0: #Start up lidar and collect data, then end this script (lidar will keep spinning)
        serialComm(motorSpeed)
        time.sleep(2) #Spin up period
        opt1(num)

        print(num, "points have been collected")
        break

    else: 
        print("endComm argument was not a valid input")
        break