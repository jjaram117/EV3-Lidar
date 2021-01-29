#!/usr/bin/env python3
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_D
import time
import math
import sys

#Functions Setup
def Convert(DesDist, diam): #Conversion from DesDist (Desired Distance) to number of rotations based on the physical diameter of the wheels
    NumRot = DesDist/(math.pi*diam) 
    return NumRot

def RotVari(diam, WheelDist): #Calculates number of rotations to achieve 360° EV3 spin. Based on wheel diameter and distance between the wheels
    Circumf = math.pi*diam #Calculate circumference of the wheels
    TurnCircumf = WheelDist*math.pi #Circumference of the invisible circle the EV3 wheels travel upon when rotating 
    NeccRot = TurnCircumf/Circumf #Rotations required for to achieve full 360° rotation of circle in "TurnCircumf"  
    
    return NeccRot
    
    
def RotateEV3(DesAng, NeccRot): #Rotate the EV3 to the desired angle. Calculates based on the necessary rotations determined from the "RotVari" function
    AngMove = DesAng/360 #Ratio between desired turning angle and 360° of a full circle
    NumRot = AngMove*NeccRot #Multiply necessary rotations and turn ratio to determine number of rotations for specific desired input

    tank_pair.on_for_rotations(left_speed=-MotorSpeed/1.5, right_speed=MotorSpeed/1.5, rotations=NumRot) #Rotates at lower speed than default to reduce slippage.


##----------------------------SETUPS----------------------------##
tank_pair = MoveTank(OUTPUT_A, OUTPUT_D) #Assign Tank pairs based on physical EV3 build
MotorSpeed = 40 #Default program speed for motors 

diam = 0.055 #Diameter of the wheels, in meters
WheelDist = 0.1 #Distance between the centers of the wheels (axle length)
CalibAng = 1.3 #Calibration value to rotate more/less based on terrain, slipage, etc. Increasing the value increases the rotation

NeccRot = RotVari(diam, WheelDist) #Calculate variables for EV3 rotation

#Assigning arguments recieved from main script to communicate EV3 movement
rotAng = float(sys.argv[1])*CalibAng #Angle to rotate can be positive (clockwise) or negative (counterclockwise). Adjust CalibAng as necessary
distMove = float(sys.argv[2]) #Distance to move. Also positive or negative

#---------Main Code---------#
NumRot = Convert(distMove, diam) #Calculate number of rotations necessary to move desired distance

RotateEV3(rotAng, NeccRot) #Rotate the EV3
time.sleep(.2)

tank_pair.on_for_rotations(left_speed=MotorSpeed, right_speed=MotorSpeed, rotations=NumRot) #Move EV3 linearly

print("\n    Your EV3 has completed the desired movement\n")