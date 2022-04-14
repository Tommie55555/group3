#!/usr/bin/env pybricks-micropython
from msilib import sequence
import sys

# from project import __init__
# from project import pick_up as pu



# from pybricks.pupdevices import ForceSensor
import time
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.hubs import EV3Brick
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks import robotics
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.robotics import DriveBase

ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
crane_motor = Motor(Port.A)

left_light = ColorSensor(Port.S3)

ultra_sensor = UltrasonicSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=47, axle_track=128)

#globala variablar
speed = 0
angle = 0


def CollisionAvoidance():
    sequence = False
    if ultra_sensor.distance() < 200:
        sequence = True  #initialize avoid sequence
    
    if sequence:
            angle += 20
            robot.straight(100)
            angle-=40
            robot.straight(100)
            robot.turn #look for line again
    
    return


def movement():
    fortsatt = 0
    while fortsatt == 0:
        robot.drive(speed, angle)
        CollisionAvoidance()

        if (40,40,40) <= left_light.rgb() <= (60,60,60): #mellan svart och vit
            speed = 100
            angle = 0
        elif (60,60,60) < left_light.rgb() <= (100,100,100): #vit
            speed = 50
            angle = -20
        elif (0,0,0) <= left_light.rgb() < (40,40,40): #svart
            speed = 50
            angle = 20
        elif left_light.rgb == 0 #designatet värde:
            speed = 20
            angle = 90

        else:
            fortsatt = 1


        

def main():
    
        loopcontinue = 0

        while loopcontinue == 0:
            """ihuiedhcid"""
            


if __name__ == '__main__':
    sys.exit(main())