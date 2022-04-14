# Pybricks imports
from pybricks.hubs import EV3Brick
from pybricks.pupdevices import ForceSensor
from pybricks.robotics import DriveBase
from pybricks.ev3devices import Motor
from pybricks.parameters import Port


#left_motor = Motor(Port.A)
#right_motor = Motor(Port.C)
#crane_motor = Motor(Port.B)
#height = 50
#touch_sensor =ForceSensor(Port._1)

#robot = DriveBase(left_motor, right_motor, wheel_diameter = 47, axle_track = 128)


def pick_up_pallet(robot,crane_motor,touch_sensor):
    """The function makes the robot pick upp a pallet."""
    #robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
    robot.straight(50)
    while check_if_pallet_corect(touch_sensor) is False:
        robot.straight(1)
    robot.stop()
    crane_motor.run(10)
    robot.straight(-50)


def elevated_pick_up(robot, crane_motor ,height,touch_sensor):
    """lifts the crane to pick upp a pallet"""
    crane_motor.run(height)
    pick_up_pallet(robot, crane_motor,touch_sensor)
    crane_motor.run(-height)

def check_if_pallet_corect(touch_sensor):
    """Checks if the pallet is on the fork"""
    return touch_sensor.pressed(3)

#elevated_pick_up(robot, crane_motor, height, touch_sensor )

