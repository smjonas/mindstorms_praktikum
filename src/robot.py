from pybricks.ev3devices import (ColorSensor, Motor, TouchSensor,
                                 UltrasonicSensor)
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait

from state import s, t

ev3 = EV3Brick()


class Robot:
    DRIVE_SPEED = 50

    def __init__(self, course):
        self.course = course

        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.D)
        # Since we used tracks, axle_track cannot be measured in a sensible way
        # We set it to the value where it does a 90 degree turn on the parcour floor if we tell it to (other angles do not work therefore).
        self.robot = DriveBase(
            left_motor=self.left_motor, right_motor=self.right_motor, wheel_diameter=56, axle_track=166.5
        )

        self.color_sensor = ColorSensor(Port.S2)
        self.dist_sensor = UltrasonicSensor(Port.S4)

        self.stage_fns = {
            course.Stage.LINE: Robot.lines,
            course.Stage.OBSTACLE: Robot.obstacle,
            course.Stage.BRIDGE: Robot.bridge,
            course.Stage.FIELD: Robot.field,
        }

    def start_stage(self):
        ev3.screen.clear()
        ev3.screen.print("Stage started")
        self.stage_fns[self.course.selected_stage](self)

    def log(self, msg=""):
        ev3.screen.print(msg)

    def check_blueline(self):
        r, g, b = self.color_sensor.rgb()
        # TODO remove debug loop and constant return value
        # while True:
        #    r, g, b = self.color_sensor.rgb()
        #    self.log(str(r) + " " + str(g) + " " + str(b))
        #    wait(100)
        # return b >= 25 and r < 25 and g < 25
        return False

    # Stage 1: Following a white line with gaps and an obstacle
    def lines(self):
        LIGHT, DARK, K = 66, 18, 0.5
        mid = (LIGHT + DARK) / 2
        NEXT_LEVEL, OBSTACLE = 1, 2

        # depends on search_angle
        GAP_THRESHOLD_ANGLE = 84
        GAP_TURNBACK_ANGLE = 120

        obstacle_distance = 100

        def is_dark():
            return self.color_sensor.reflection() <= DARK

        def is_light():
            return self.color_sensor.reflection() >= LIGHT

        def is_gray():
            brightness = self.color_sensor.reflection()
            return brightness > DARK and brightness < LIGHT

        self.start_value = 0

        def store_state():
            self.prev_state = self.robot.state()

        def turn(angle):
            def turn_angle():
                self.robot.drive(0, angle)
                wait(10)
            return turn_angle

        # Lots of currying :D
        def did_turn(degrees):
            def did_turn_degrees():
                delta = self.robot.angle() - self.prev_state.angle
                return delta >= degrees if degrees > 0 else delta <= degrees
            return did_turn_degrees

        def did_drive(distance_in_mm):
            def did_drive_distance():
                delta = self.robot.distance() - self.prev_state.distance
                return delta >= distance_in_mm
            return did_drive_distance

        def check_events(transitions, on_enter=None):
            new_transitions = [
                (self.check_blueline, NEXT_LEVEL),
                (lambda: self.dist_sensor.distance() < obstacle_distance, OBSTACLE),
            ]
            return s(new_transitions + transitions, on_enter)

        def drive_regulated():
            brightness = self.color_sensor.reflection()
            delta = brightness - mid
            turn_rate = K * delta
            self.robot.drive(self.DRIVE_SPEED, turn_rate)
            wait(10)

        def drive_straight():
            self.robot.drive(self.DRIVE_SPEED, 0)

        states = {
            "start": check_events(
                [(is_gray, "drive_regulated"), (is_dark, "store_and_turn_right"), (is_light, "turn_left")],
                lambda: wait(10),
            ),
            "turn_left": s([(is_gray, "drive_regulated")], turn(15)),
            "drive_regulated": check_events(
                [(is_dark, "store_and_drive_right"), (True, "turn_left")], drive_regulated
            ),
            "store_and_turn_right": s([(True, "turn_right")], store_state),
            "turn_right": s(
                [(is_gray, "drive_regulated"), (did_turn(GAP_THRESHOLD_ANGLE), "store_and_turn_back_left")],
                turn(-15),
            ),
            "store_and_turn_back_left": s([(True, "turn_back_left")], store_state),
            "turn_back_left": s([(did_turn(GAP_TURNBACK_ANGLE), "store_and_drive_straight")], turn(-15)),
            "store_and_drive_straight": check_events([(True, "drive_straight")], store_state),
            "drive_straight": check_events([(did_drive(100), "start")], drive_straight),
        }
        cur_state = states["start"]

        while not self.course.should_abort():
            successor = cur_state.check_conditions()
            if not successor:
                cur_state = states[successor]
                if cur_state == NEXT_LEVEL:
                    self.robot.stop()
                    self.course.running = False
                    self.log("Reached end of line")
                    return
                elif cur_state == OBSTACLE:
                    self.robot.stop()
                    self.log("Obstacle found, circumnavigating...")
                    continue

                if cur_state.on_enter:
                    cur_state.on_enter()
                print("set state to", successor)

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
                self.course.running = False
                return
