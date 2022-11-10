import math

from pybricks.ev3devices import (ColorSensor, GyroSensor, InfraredSensor,
                                 Motor, TouchSensor, UltrasonicSensor)
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from main import Level


class Robot:
    DRIVE_SPEED = 50

    def __init__(self, game):
        self.game = game
        self.turn_rate = 0
        left_motor = Motor(Port.A)
        right_motor = Motor(Port.D)
        self.robot = DriveBase(
            left_motor=left_motor, right_motor=right_motor, wheel_diameter=36, axle_track=128
        )
        self.color_sensor = ColorSensor(Port.S2)
        self.level_fns = {Level.LINE: Robot.lines}
        # self.pressure_sensor =

    def start_level(self, level):
        self.level_fns[level]()

    # def log(self, msg=""):
        # ev3.display.print(f"[Level {self.level}] {msg}")
        # ev3.screen.print(msg)

    # Level 1: line following
    def lines(self):
        while not self.game.should_abort():
            LIGHT, DARK, K = 66, 13, 1.2
            mid = (LIGHT + DARK) / 2

            brightness = self.color_sensor.reflection()
            delta = brightness - mid
            self.turn_rate = K * delta

            # if brightness >= 4 and brightness <= 6:
            if brightness >= DARK:
                self.log("Driving")
                self.self.drive(self.DRIVE_SPEED, self.turn_rate)
                wait(10)
                # self.self.straight(50)
            else:
                self.log("Turning")
                self.self.turn(math.copysign(1, self.turn_rate))
            self.log("T:" + str(self.turn_rate) + ", L:" + str(self.color_sensor.reflection()))
