#!/usr/bin/env pybricks-micropython
from course import Course
from menu import Menu
from robot import Robot

from pybricks.hubs import EV3Brick


ev3 = EV3Brick()
course = Course()
robot = Robot(course)

menu = Menu(course)

while True:
    menu.update()
    if course.running:
        robot.start_stage()
        menu.select_next()
