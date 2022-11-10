#!/usr/bin/env pybricks-micropython
import math

from pybricks.ev3devices import (ColorSensor, GyroSensor, InfraredSensor,
                                 Motor, TouchSensor, UltrasonicSensor)
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from levels import lines
from robot import Robot

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()


class Game:
    def __init__(self):
        self.running = False

    def should_abort(self):
        abort = Button.CENTER in ev3.buttons.pressed()
        if abort:
            self.running = False
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
        self.game = game

    def update(self):
        pressed_buttons = ev3.buttons.pressed()
        if Button.UP in pressed_buttons:
            self.select_next()
        elif Button.DOWN in pressed_buttons:
            self.select_prev()
        elif Button.CENTER in pressed_buttons:
            self.game.running = True

    def redraw(self):
        ev3.screen.clear()
        for i in range(Level.COUNT):
            ev3.screen.draw_text(1, i * (Menu.screen_height / Level.COUNT), Level.to_string(i))
        ev3.screen.draw_box(
            1,
            self.selected * (Menu.screen_height / Level.COUNT),
            Menu.screen_width,
            (self.selected + 1) * (Menu.screen_height / Level.COUNT),
        )

    def select_next(self):
        self.selected = (self.selected + 1) % Level.COUNT
        self.redraw()

    def select_prev(self):
        self.selected = (self.selected - 1) % Level.COUNT
        self.redraw()


game = Game()
robot = Robot(game)
menu = Menu(game)

while True:
    menu.update()
    if game.running:
        robot.start_level(menu.selected)


# robot.loop()
