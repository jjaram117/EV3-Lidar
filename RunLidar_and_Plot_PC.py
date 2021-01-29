import serial, time, struct
import csv
import math
import matplotlib.pyplot as plt
from operator import itemgetter 
import statistics
    
Start_Scan = b"\xA5\x20" #Begins scanning
Force_Scan = b"\xA5\x21" #Overrides anything preventing a scan
Health = b"\xA5\x52" #Returns the state of the Lidar
Stop_Scan = b"\xA5\x25" #Stops the scan
RESET = b"\xA5\x40" #Resets the device
Motor = b"\xA5\xF0" #Sets motor speed
MotorFast = b'\xa5\xf0\x02\x94\x02\xc1'
Motor0 = b"\xA5\xF0\x02\x00\x00\x57" #A5F0 0200 0057
Rate = b'\xA5\x59'

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

def GrabPts(num):
    run = 0

    #Preallocation for our 4 variables
    x = [0]*num
    y = [0]*num
    a = [0]*num
    d = [0]*num

    #Wiping the buffer
    ser.flushInput()
    ser.flushOutput()

    #Run Lidar and save desired number of angle and distance points to a file
    while run < num:
        reading = ser.read(5)   
        try: 
            (quality, S, notS, C, angle, dist) = translate(reading)
        except Exception as e:
            print(e) 
            continue 

        if quality > 10:
            #Eliminating noise calculated to be at distance (d) = 0
            if dist == 0:
                continue
            else:
                #Append polar coordinates to their running variables
                a[run] = (angle)
                d[run] = (dist)
                
                #Converting from polar to cartesian coordinates
                ang = angle*3.14/180 #radians to degrees
                x_calc = dist*math.cos(ang) 
                y_calc = dist*math.sin(ang)

                #Append cartesian coordinates to their running variables
                x[run] = (x_calc)
                y[run] = (y_calc)

                #Update Run variable
                run += 1
    return (x,y,a,d)


def plotter(x,y):
#Creating circular outline to represent lidar shape and direction being faced
    circle1 = plt.Circle((0, 0), 0.05, color='r', fill=False) 
    ax = plt.gca()
    ax.add_artist(circle1)
    plt.arrow(0,0,0.07,0, shape='full', lw=.5, length_includes_head=True, head_width=.015, color='r') #Arrow to show lidar direction 

    #Plotting data
    plt.scatter(x, y)

    #Formatting plot
    plt.title('Lidar Mapping Data')
    plt.ylabel('Y-axis')
    plt.xlabel('X-axis')
    plt.axis('equal') #Equalizing the axis ratios
    
    xmin = min(x)
    xmax = max(x)
    ymin = min(y)
    ymax =max(y)
    ax.set_xlim([-xmax*1.1,xmax*1.1]) #Setting axis limits based on data. Keeps plot consistent after inverting axis
    ax.set_ylim([-ymax*1.1,ymax*1.1])

    plt.gca().invert_yaxis() #Flips y axis to match physical orientation to the lidar

    plt.show(block=False)
    time.sleep(1)

def LidarComm(): #Compact method of starting the communication with the Lidar
    serialComm(Start_Scan)
    while ser.read() != b'\xA5':
        time.sleep(0.01)
    reply = ser.read(6)
    if (reply == b'\x5A\x05\x00\x00\x40\x81'):
        print('starting...')
    else:
        print('incorrect reply')

def MedianDist(a,d,tol):
    ##-90 degree distance calculations-##
    Ind_90 = [i for i, x in enumerate(a) if x >= 90-tol and x <= 90+tol] #Gives indices of all values near 90 degrees within the tolerance "tol". ie if tol=2 the range is from 88° to 92°
    
    medDis_90 = statistics.median(itemgetter(*Ind_90)(d)) #"itemgetter" gives list of values in "a" for the calculated indices in "ind". "statistics.median()"" calculates the median of those values and gives the median distance
    time.sleep(.1)

    ##-0 degree distance calculations-##
    Ind_0 = [i for i, x in enumerate(a) if x >= 360-tol] #or x <= 0+tol]

    medDis_0 = statistics.median(itemgetter(*Ind_0)(d))
    time.sleep(.1)

    return (medDis_90, medDis_0)


def SaveData():
    name = str(input('What would you like to name your file? (Do not inlude the file type, just the file name)\n')) #
    
    #Saving all data to the named .csv file
    filename = "C:/Users/Owner/Desktop" + name + ".csv" #Rename folder address as desired 
    with open(filename, 'a', newline='') as f:
            writer = csv.writer(f, delimiter = ',')
            writer.writerows(zip(x,y,a,d))
    time.sleep(.1)

    print('Your file has been saved as', name,'\n')
    time.sleep(2)


def opt1(): #Grab Data
    global x,y,a,d

    num = int(input("How many points do you want?\n"))

    LidarComm()

    (x,y,a,d) = GrabPts(num)

    serialComm(Stop_Scan)
    ser.read(ser.inWaiting()) #Flush the buffer

    plotter(x,y)
    print("Your points have been plotted\n")

def opt2(): #Close Current figure
    print('Are you sure you want to clear the current figure? y/n\n')
    act = input()

    if act == 'y' or 'Y':
        plt.close()
    
    elif act == 'n' or 'N':
        print('\nNo')
        #continue

    else:
        print("Not an available option\n")

def opt3(): #Save current data to a .csv file
    SaveData()


def opt4(): #Calculate distance after move
    num = int(input("How many points do you want? The more points (i.e. 1500+) the better\n"))

    tol = int(input("What degree tolerance do you want? (The higher the tolerance the better. i.e: 7)\n"))

    ##--Collecting the first set of points--##
    print('Now collecting your first set of points\n\n')
    LidarComm()
    
    (x,y,a,d) = GrabPts(num) #Grab all the points. x,y used for the plots, a,d used for the median distance calculations
    
    serialComm(Stop_Scan)
    ser.read(ser.inWaiting())
    time.sleep(.5)

    plotter(x,y) #Plotting the data

    (med1_90,med1_0) = MedianDist(a,d,tol) #Assigns first pair of calculated medians to med1_90 and med1_0
    time.sleep(.5)

    print('Your first distance at 90° has been calculated to be =', "%.3f" % med1_90, '\nYour first distance at 0° has been calculated to be =', "%.3f" % med1_0)
    time.sleep(2)


    ##--Collecting the Second set of points--##
    print('\nPlease move your sensor to the new location. Press "Enter/Return" when ready to continue\n')
    time.sleep(2.5)
    input("Press Enter to continue...\n")

    #Collecting the second set of points:
    print('Now collecting your second set of points')
    LidarComm()

    (x,y,a,d) = GrabPts(num)

    serialComm(Stop_Scan)
    ser.read(ser.inWaiting())
    time.sleep(.5)

    plotter(x,y)

    (med2_90,med2_0) = MedianDist(a,d,tol) #Assigns second pair of calculated medians to med2_90 and med2_0
    
    #Calculating the differences between the calculated distances
    distMoved_90 = med1_90-med2_90
    distMoved_0 = med1_0-med2_0  

    print('Your second distance at 90° has been calculated to be =', "%.3f" % med2_90, '\nYour second distance at 0° has been calculated to be =', "%.3f" % med2_0)
    time.sleep(2)
    
    #Printing out the final results of the estimated moves
    print('\n   Your estimated total distance moved in the 90° direction is =', "%.3f" % distMoved_90, 'meters')
    print('\n   Your estimated total distance moved in the 0° direction is =', "%.3f" % distMoved_0, 'meters')
    time.sleep(5)

def opt5(): #End program and stop lidar
    print('\nAre you sure you want to end the program? y/n\n')
    act = input()

    if act == 'y' or 'Y':
        serialComm(Motor0)
        exit()
    
    elif act == 'n' or 'N':
        print('\nNo')
        #continue

    else:
        print("Not an available option\n")


##----------------------------SETUPS----------------------------##
port = "COM Port" #Enter your COM port within the quotations

ser = serial.Serial(port, 115200, timeout = 1)
ser.setDTR(False)

serialComm(RESET,3)
serialComm(Health,1)
serialComm(Rate,1)

#Set Motor speed
speed = 800  # between 1 and 1023
motorSpeed = Motor + b'\x02'+struct.pack('<H',speed)  # 2 byte payload, little endian of the desired speed
serialComm(motorSpeed)
time.sleep(2) #Motor spin up

inp = 0 #setting up the variable for the input


##----------------------------MAIN LOOP----------------------------##
while True:
    inp = input('\nWhat would you like to do? \n1 = Grab Data \n2 = Close Current Figure \n3 = Save current (x,y,a,d) data to a file \n4 = Calculate distance after move \n5 = End Program and Stop Lidar \n\n')
    
    time.sleep(.5)

    if inp == '1': #Grab Data
        opt1()        

    elif inp == '2': #Close Current figure
        opt2()

    elif inp == '3': #Save current data to a .csv file
        opt3()
        
    elif inp == '4': #Calculate distance after move
        opt4()

    elif inp == '5': #End program and stop lidar
        opt5()
    
    else:
        print("Not an available option\n")