#!/usr/bin/env pybricks-micropython
import math
from pybricks.ev3devices import (ColorSensor, GyroSensor, InfraredSensor,
                                 Motor, TouchSensor, UltrasonicSensor)
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import DataLog, StopWatch, wait

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()

class Level:
    LINES, BRIDGE = range(2)


# Level 1: line following
def lines(robot):
    robot.log()
    LIGHT, DARK, K = 66, 13, 1.2
    mid = (LIGHT + DARK) / 2

    brightness = robot.color_sensor.reflection()
    delta = brightness - mid
    robot.turn_rate = K * delta

    # if brightness >= 4 and brightness <= 6:
    if brightness >= DARK:
        robot.log("Driving")
        robot.robot.drive(Robot.DRIVE_SPEED, robot.turn_rate)
        wait(10)
        # robot.robot.straight(50)
    else:
        robot.log("Turning")
        robot.robot.turn(math.copysign(1, robot.turn_rate))
    robot.log("T:" + str(robot.turn_rate) + ", L:" + str(robot.color_sensor.reflection()))


class Robot:
    DRIVE_SPEED = 50

    def __init__(self):
        self.level, self.turn_rate = Level.LINES, 0
        left_motor = Motor(Port.A)
        right_motor = Motor(Port.D)
        self.robot = DriveBase(left_motor=left_motor, right_motor=right_motor, wheel_diameter=36, axle_track=128)
        self.color_sensor = ColorSensor(Port.S2)
        # self.pressure_sensor =

    def log(self, msg=""):
        # ev3.display.print(f"[Level {self.level}] {msg}")
        ev3.screen.print(msg)

    def loop(self):
        level_fn = {Level.LINES: lines}
        while True:
            # Level function must set turn_rate
            level_fn[self.level](self)
            # self.robot.drive(Robot.DRIVE_SPEED, self.turn_rate)
            # self.robot.turn(self.turn_rate)


# robot.turn(90)

# Line following algorithm
# Pseudocode:
# light: 100
# dark: 0


Robot().loop()
