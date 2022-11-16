#!/usr/bin/env pybricks-micropython
import math

from pybricks.ev3devices import (ColorSensor, GyroSensor, InfraredSensor,
                                 Motor, TouchSensor, UltrasonicSensor)
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait

ev3 = EV3Brick()

#TODO separate classes into files


class Course:
    def __init__(self):
        self.running = False

    def should_abort(self):
        abort = Button.LEFT in ev3.buttons.pressed()
        if abort:
            self.running = False
        return abort
    
    #TODO real enum?
    class Stage:
        COUNT = 4
        LINE, OBSTACLE, BRIDGE, FIELD = range(COUNT)

        def to_string(Stage):
            return {
                Stage.LINE: "Follow lines",
                Stage.OBSTACLE: "Move obstacle",
                Stage.BRIDGE: "Cross bridge",
                Stage.FIELD: "Color field",
            }[Stage]


class Menu:
    SCREEN_WIDTH = ev3.screen.width
    SCREEN_HEIGHT = ev3.screen.height

    def __init__(self, course):
        self.course = course
        self.selected = course.Stage.LINE
        self.last_pressed = None
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
            self.course.running = True
        #TODO decide what to do
        # Probably not needed anymore because of last_pressed-query
        # wait(100)

    def redraw(self):
        ev3.screen.clear()
        # A few magic numbers here to make the menu look good
        for i in range(course.Stage.COUNT):
            ev3.screen.draw_text(3, i * (Menu.SCREEN_HEIGHT / course.Stage.COUNT), course.Stage.to_string(i))
        ev3.screen.draw_box(
            1,
            self.selected * (Menu.SCREEN_HEIGHT / course.Stage.COUNT),
            Menu.SCREEN_WIDTH - 1,
            (self.selected + 1) * (Menu.SCREEN_HEIGHT / course.Stage.COUNT) - 8,
        )

    def select_next(self):
        self.selected = (self.selected + 1) % course.Stage.COUNT
        self.redraw()

    def select_prev(self):
        self.selected = (self.selected - 1) % course.Stage.COUNT
        self.redraw()


class Robot:
    DRIVE_SPEED = 50

    def __init__(self, course):
        self.course = course

        left_motor = Motor(Port.A)
        right_motor = Motor(Port.D)
        # Since we used tracks, axle_track cannot be measured in a sensible way
        # We set it to the value where it does a 90 degree turn on the parcour floor if we tell it to (other angles do not work therefore).
        self.robot = DriveBase(
            left_motor=left_motor, right_motor=right_motor, wheel_diameter=56, axle_track=166.5
        )

        self.color_sensor = ColorSensor(Port.S2)
        self.us_sensor = UltrasonicSensor(Port.S4)

        self.stage_fns = {
            course.Stage.LINE: Robot.lines,
            course.Stage.OBSTACLE: Robot.obstacle,
            course.Stage.BRIDGE: Robot.bridge,
            course.Stage.FIELD: Robot.field,
        }

    def start_stage(self, Stage):
        ev3.screen.clear()
        ev3.screen.print("Stage started")
        self.stage_fns[Stage](self)

    def log(self, msg=""):
        ev3.screen.print(msg)

    def check_blueline(self):
        r, g, b = self.color_sensor.rgb()
        #TODO remove debug loop
        #while True:
        #    r, g, b = self.color_sensor.rgb()
        #    self.log(str(r) + " " + str(g) + " " + str(b))
        #    wait(100)
        return b >= 25 and r < 25 and g < 25

    # Stage 1: Following a white line with gaps and an obstacle
    def lines(self):
        LIGHT, DARK, K = 66, 13, 1.2
        mid = (LIGHT + DARK) / 2

        obstacle_distance = 100

        start_angle = 0
        search_angle = 2
        gap_threshold_angle = 84 # depends on search_angle
        gap_turnback_angle = 120

        self.log("Following line")
        while not self.course.should_abort():
            self.log(str(self.us_sensor.distance())) #TODO remove debug log
            if self.check_blueline():
                self.robot.stop()
                self.log("Reached end of line")
                course.running = False
                return
            if self.us_sensor.distance() < obstacle_distance:
                self.robot.stop()
                self.log("Obstacle found, circumnavigating...")
                continue
            
            #TODO Refactor nested while loop (program cannot be exited in there)
            brightness = self.color_sensor.reflection()
            if brightness <= DARK:
                start_angle = self.robot.angle()
                found_gap = False
                while self.color_sensor.reflection() <= DARK:
                    self.log("Turning right " + str(start_angle - self.robot.angle())) #TODO remove debug log
                    self.robot.turn(-search_angle)
                    if  start_angle - self.robot.angle() > gap_threshold_angle:
                        found_gap = True
                        break
                if not found_gap:
                    self.log("No gap") #TODO remove debug log
                    continue
                
                self.log("Found gap, crossing...")
                self.log("Turning back") #TODO remove debug log
                self.robot.turn(gap_turnback_angle)
                self.log("Crossing gap") #TODO remove debug log
                self.robot.drive(self.DRIVE_SPEED, 0)
                # Drive forward for 3s
                wait(3000)
                self.log("Following line")

            elif brightness >= LIGHT:
                self.log("Turning left") #TODO remove debug log
                self.robot.turn(search_angle)

            else:
                delta = brightness - mid
                turn_rate = K * delta
                self.log("Driving") #TODO remove debug log
                self.robot.drive(self.DRIVE_SPEED, turn_rate)
                wait(10)

    # Stage 2: Push a block in the corner
    def obstacle(self):
        while not self.course.should_abort():
            if self.check_blueline():
                self.robot.stop()
                return

    # Stage 3: Cross a bridge with perpendicular ramps leading up/down at the ends
    def bridge(self):
        while not self.course.should_abort():
            if self.check_blueline():
                self.robot.stop()
                return

    # Stage 4: Find a red and white patch of color on the floor
    def field(self):
        while not self.course.should_abort():
            if self.check_blueline():
                self.robot.stop()
                course.running = False
                return


course = Course()
robot = Robot(course)
menu = Menu(course)

while True:
    menu.update()
    if course.running:
        robot.start_stage(menu.selected)
        #menu.select_next()
