import time
import math
import csv
import paramiko
import operator
import statistics
from scp import SCPClient
from shutil import copyfile
import matplotlib.pyplot as plt
from operator import itemgetter 

##--------------SSH communication functions
def sshEV3(hostname, username, password): #Initiate ssh with the EV3
    global client

    #Initialize the SSH client
    client = paramiko.SSHClient()
    #Add to known hosts
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(hostname=hostname, username=username, password=password)
    except:
        print("[!] Cannot connect to the SSH Server")
        exit()

def RunLidar(endComm, num): #Function communicates commands to the Lidar mounted on the EV3. 
                            #"endComm" parameter indicates whether to stop lidar or not via a 1 or 0 respectively, and "num" parameter is number of points lidar grabs
    command = 'python3 /home/robot/RunLidar.py ' + str(endComm) + ' ' + str(num) #Convert "endComm" and "num" into string to be passed as terminal argument. Alter address as necessary
    (stdin, stdout, stderr) = client.exec_command(command) #Paramiko format to execute the desired command

    #Display any lines printed from the RunLidar script
    for line in stdout.readlines():
        print(line) 

def scpData(): #scp the temporary data file to local computer. Read it in and assign respective x,y,a,d variables
    with SCPClient(client.get_transport()) as scp:
        scp.get('/home/robot/currData.csv', 'C:/Users/Owner/Desktop/currData.csv') #scp.get("source", "destination")

    global x,y,a,d

    x = []
    y = []
    a = []
    d = []

    with open('C:/Users/Owner/Desktop/currData.csv') as f:
        reader = csv.reader(f)
        for row in reader:
            x.append(float(row[0]))
            y.append(float(row[1]))
            a.append(float(row[2]))
            d.append(float(row[3]))

def RunEV3(rotAng, distMove): #Function communicates movement commands to the EV3. "rotAng" is rotation angle for the robot, "distMove" is the distance to move
    command = 'python3 /home/robot/RunEV3.py ' + str(rotAng) + ' ' + str(distMove) #Extra space string needed to distinguish between arguments
    (stdin, stdout, stderr) = client.exec_command(command)

    #Display any lines printed from the RunEV3 script
    for line in stdout.readlines():
        print(line) 

##--------------Functions impacting only local computer
def plotter(x,y): #Plot all data collected by the lidar
    
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

def copyData(): #Creates copy of the "currData" file under a new name in order to save current data set 
    name = str(input('What would you like to name your file? (Do not inlude the file type, just file name with no spaces)\n')) #User input
    
    #Saving all data to the named .csv file
    filename = "C:/Users/Owner/Desktop" + name + ".csv" #Adjust folder address as desired
    
    copyfile('C:/Users/Owner/Desktop/currData.csv', filename)

    print('\nYour current data set has been saved\n')
    time.sleep(.1)

def MedianDist(a,d,tol): #Calculates median distance of angles within a given tolerance "tol"  
    ##---90 degree distance calculations---##
    Ind_90 = [i for i, x in enumerate(a) if x >= 90-tol and x <= 90+tol] #Gives indices of all values near 90 degrees within the tolerance "tol". (ie if tol=2, the range is from 88° to 92°)

    medDis_90 = statistics.median(itemgetter(*Ind_90)(d)) #"itemgetter" gives list of values in "a" for the calculated indices in "ind". Then "statistics.median()" calculates the median of those values to give distance at that angle

    ##---0 degree distance calculations---##
    Ind_0 = [i for i, x in enumerate(a) if x >= 360-tol or x <= 0+tol]

    medDis_0 = statistics.median(itemgetter(*Ind_0)(d))

    return (medDis_90, medDis_0)


##--------------User Interface Options - Options Table 
def opt1(): #Request lidar to collect data
    num = int(input("    How many points do you want?\n"))

    RunLidar(0, num) #Communicate to run Lidar through the EV3 (endComm=0) and grab "num" number of points
    time.sleep(0.2)

    scpData() #Grab the newly created "currData.csv" file from the EV3. Will output global x,y,a,d variables

    plotter(x,y) #Plot data for the user to see
    print("    Your data has been plotted\n")

def opt2(): #Rotate or move the EV3
    print('    \nPositive angles rotate clockwise, negatives counterclockwise. Positive values move forwards, negatives move backwards.     \nUnits are in degrees and meters.     \nThe robot will rotate first, then move.')
    time.sleep(2)
    
    rotAng = float(input('    \nHow much would you like to rotate? (90, 80, 167, etc.)\n'))
    distMove = float(input('    \nHow much would you like to move? (1, -3, 0.5, etc.)\n'))

    RunEV3(rotAng, distMove) #SSh into EV3 to get EV3 going

def opt3(): #Close Current figure
    print('    Are you sure you want to clear the current figure? y/n\n')
    act = input()

    if act == 'y' or 'Y':
        plt.close()
    
    elif act == 'n' or 'N':
        print('\nNo')
        #continue

    else:
        print("    Not an available option\n")

def opt4(): #Copy current "currData" file to a new local .csv file 
    copyData()

def opt5(): #Calculate distance after move
    ##--------Phase 1--------##
    num = int(input("    How many points do you want to grab?\n"))

    tol = int(input("    What degree tolerance do you want? (i.e: 1, 2,, etc)\n"))

    ##-----Communicating with EV3 to collect the first set of points-----##
    print('    Now collecting your first set of points\n\n')
    
    RunLidar(0, num) #Running lidar to collect first set of points. Should print out "points have been collected", which comes from the EV3 RunLidar script
    time.sleep(.2)

    scpData() #Grabbing the first set of data and saving to local computer 

    plotter(x,y) #Plot the current set of points
    time.sleep(.2)

    (med1_90,med1_0) = MedianDist(a,d,tol) #Assigns calculated distances for 90° (med1_90) and 0° (med1_0) for the first set of data

    print('    Your first distance at 90° has been calculated to be =', "%.3f" % med1_90, '    \nYour first distance at 0° has been calculated to be =', "%.3f" % med1_0)
    time.sleep(3)

    distMove = float(input('    \nwhat distance would you like to move?     \nOnly linear travel is allowed in this option (no rotation). Positive values move forwards, negatives move backwards. Units are in meters.\n'))
    time.sleep(2)

    RunEV3(0, distMove) #No rotation allowed here!


    ##--------Phase 2--------##
    print('Now collecting your second set of points\n\n')
    
    RunLidar(0, num) #Running lidar to collect second set of points.
    time.sleep(.2)

    scpData() #Grabbing the second set of data and saving to local computer 

    plotter(x,y) #Overlay the second set of points on the plot
    time.sleep(.2)

    (med2_90,med2_0) = MedianDist(a,d,tol) #Second set of points median distance being calculated and saved to variables

    print('    Your second distance at 90° has been calculated to be =', "%.3f" % med2_90, '    \nYour second distance at 0° has been calculated to be =', "%.3f" % med2_0)
    time.sleep(3)


    ##--------Final Outputs--------##
    #Calculating the differences between the sets of calculated distances
    distMoved_90 = med1_90-med2_90 #90 degree direction calculations
    distMoved_0 = med1_0-med2_0 #0 degree direction calculations
  
    #Printing out the final results of the estimated moves
    print('\n    Your estimated total distance moved in the 90° direction is =', "%.3f" % distMoved_90, 'meters')
    print('\n    Your estimated total distance moved in the 0° direction is =', "%.3f" % distMoved_0, 'meters')
    time.sleep(3)

def opt6(): #Stop lidar, end SSH connection, end the program 
    print('    \nAre you sure you want to end communication to the EV3 and end this program? (y/n)\n')
    act = input()

    if act == 'y' or 'Y':
        RunLidar(1, 0) #Runs argument to EV3 that turns off the lidar
        client.close() #End the ssh connection with the EV3
        exit() #End the program
    
    elif act == 'n' or 'N':
        print('\nNo')
        #continue

    else:
        print("    Error. Returning you to Options Table\n")


##-----------------------------MAIN CODE-----------------------------##
#SSH server login info - EV3
hostname = "EV3 IP Address" #PC and EV3 must be on the same network
username = "EV3 Username"
password = "EV3 Password"

#SSh into the client
sshEV3(hostname, username, password)


##----------------------------MAIN LOOP----------------------------##

while True:
    inp = input('    \nOPTIONS TABLE: What would you like to do? \n1 = Grab Data \n2 = Rotate or Move EV3 \n3 = Close Current Figure \n4 = Save current (x,y,a,d) data to a local file \n5 = Calculate distance after move \n6 = Stop Lidar and end remote communication \n\n')
    
    time.sleep(.5)

    if inp == '1': #Request lidar to collect data
        opt1()        

    if inp == '2': #Rotate or move the EV3
        opt2()

    elif inp == '3': #Close Current figure
        opt3()

    elif inp == '4': #Copy current "currData" file to a new local .csv file 
        opt4()
        
    elif inp == '5': #Calculate distance after move
        opt5()

    elif inp == '6': #Stop lidar, end SSH connection, end the program 
        opt6()
    
    else:
        print("Error. Returning you to Options Table\n")
        time.sleep(2)