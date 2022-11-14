#!/usr/bin/env pybricks-micropython
import math

from pybricks.ev3devices import (ColorSensor, GyroSensor, InfraredSensor,
                                 Motor, TouchSensor, UltrasonicSensor)
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()


class Game:
    def __init__(self):
        self.running = False

    def should_abort(self):
        abort = Button.LEFT in ev3.buttons.pressed()
        if abort:
            self.running = False
        print("ABort:", abort)
        return abort


class Level:
    COUNT = 4
    LINE, OBSTACLE, BRIDGE, FIELD = range(COUNT)

    def to_string(level):
        return {
            Level.LINE: "Follow lines",
            Level.OBSTACLE: "Move obstacle",
            Level.BRIDGE: "Cross bridge",
            Level.FIELD: "Color field",
        }[level]


class Menu:
    screen_width, screen_height = ev3.screen.width, ev3.screen.height

    def __init__(self, game):
        self.selected = Level.LINE
        self.last_pressed = None
        self.game = game
        self.redraw()

    def update(self):
        pressed_buttons = ev3.buttons.pressed()
        # Same button as before is still pressed, do nothing
        if self.last_pressed in pressed_buttons:
            return
        else:
            self.last_pressed = None

        if Button.UP in pressed_buttons:
            self.select_prev()
            self.last_pressed = Button.UP
        elif Button.DOWN in pressed_buttons:
            self.select_next()
            self.last_pressed = Button.DOWN
        elif Button.CENTER in pressed_buttons:
            self.game.running = True
        wait(100)

    def redraw(self):
        ev3.screen.clear()
        for i in range(Level.COUNT):
            ev3.screen.draw_text(3, i * (Menu.screen_height / Level.COUNT), Level.to_string(i))
        ev3.screen.draw_box(
            1,
            self.selected * (Menu.screen_height / Level.COUNT),
            Menu.screen_width - 1,
            (self.selected + 1) * (Menu.screen_height / Level.COUNT) - 8,
        )

    def select_next(self):
        self.selected = (self.selected + 1) % Level.COUNT
        self.redraw()

    def select_prev(self):
        self.selected = (self.selected - 1) % Level.COUNT
        self.redraw()


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
        self.level_fns = {
            Level.LINE: Robot.lines,
            Level.OBSTACLE: Robot.obstacle,
            Level.BRIDGE: Robot.bridge,
            Level.FIELD: Robot.field,
        }
        # self.pressure_sensor =

    def start_level(self, level):
        ev3.screen.clear()
        ev3.screen.print("Level started")
        self.level_fns[level](self)

    def log(self, msg=""):
        ev3.screen.print(msg)

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
                # self.robot.drive(self.DRIVE_SPEED, self.turn_rate)
                wait(10)
                # self.self.straight(50)
            else:
                self.log("Turning")
                self.robot.turn(math.copysign(1, self.turn_rate))
            # self.log("T:" + str(self.turn_rate) + ", L:" + str(self.color_sensor.reflection()))

    def obstacle(self):
        while not self.game.should_abort():
            self.log("Obstacle")

    def bridge(self):
        while not self.game.should_abort():
            self.log("Bridge")

    def field(self):
        while not self.game.should_abort():
            self.log("Field")


game = Game()
robot = Robot(game)
menu = Menu(game)

while True:
    menu.update()
    if game.running:
        robot.start_level(menu.selected)
        menu.redraw()


# robot.loop()
