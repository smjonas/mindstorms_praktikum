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

        obstacle_distance = 100

        def is_dark():
            return self.color_sensor.reflection() <= DARK

        def is_light():
            return self.color_sensor.reflection() >= LIGHT

        def is_gray():
            brightness = self.color_sensor.reflection()
            return brightness > DARK and brightness < LIGHT

        self.start_value = 0
        gap_threshold_angle = 84  # depends on search_angle
        gap_turnback_angle = 120

        def store_state():
            self.prev_state = self.robot.state()

        def drive_right():
            self.robot.drive(0, -15)
            wait(10)

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

        flag_tr = False
        flag_tbl = False
        flag_cg = False
        states = {
            "start": check_events(
                [(is_gray, "drive_regulated"), (is_dark, "store_prev_angle"), (is_light, "turn_left")],
                lambda: wait(10),
            ),
            "turn_left": s([(is_gray, "drive_regulated")]),
            "drive_regulated": check_events(
                [(is_dark, "store_and_drive_right"), (True, "turn_left")], drive_regulated
            ),
            "store_and_turn_right": s([(True, "turn_right")], store_state),
            "turn_right": s([(is_gray, "drive_regulated"), (did_turn(90), "turn_back_left")], drive_right),
            "turn_back_left": s([(did_turn(-90), "drive_straight")], store_state),
            "drive_straight": check_events([(did_drive(100), "start")], store_state),
        }
        cur_state = states["z0"]

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

        self.log("Following line")
        while not self.course.should_abort():
            # self.log(str(self.us_sensor.distance())) #TODO remove debug log
            if self.check_blueline():
                self.robot.stop()
                self.log("Reached end of line")
                self.course.running = False
                return
            if self.dist_sensor.distance() < obstacle_distance:
                self.robot.stop()
                self.log("Obstacle found, circumnavigating...")
                continue

            if flag_tbl:
                if self.robot.angle() - start_value > gap_turnback_angle:
                    flag_tbl = False
                    flag_cg = True
                    start_value = self.robot.distance()
                    self.robot.drive(self.DRIVE_SPEED, 0)
                    self.log("Crossing gap")
                continue

            if flag_cg:
                if self.robot.distance() - start_value > 80:
                    flag_cg = False
                    self.log("Following line")
                continue

            # TODO Refactor nested while loop (program cannot be exited in there)
            brightness = self.color_sensor.reflection()
            if brightness <= DARK:
                if flag_tr:
                    self.log(str(start_value - self.robot.angle()))
                    if start_value - self.robot.angle() > gap_threshold_angle:
                        self.log("Found gap, crossing...")
                        self.log("Turning back")
                        flag_tr = False
                        flag_tbl = True
                        start_value = self.robot.angle()
                        self.robot.drive(0, 30)
                    continue
                flag_tr = True
                start_value = self.robot.angle()
                self.robot.drive(0, -15)
                continue

                # while self.color_sensor.reflection() <= DARK:
                #     self.log("Turning right " + str(start_value - self.robot.angle())) #TODO remove debug log
                #     self.robot.turn(-search_angle)
                #     if  start_value - self.robot.angle() > gap_threshold_angle:
                #         found_gap = True
                #         break
                # if not found_gap:
                #     self.log("No gap") #TODO remove debug log
                #     continue

                # self.log("Found gap, crossing...")
                # self.log("Turning back") #TODO remove debug log
                # self.robot.turn(gap_turnback_angle)
                # self.log("Crossing gap") #TODO remove debug log
                # self.robot.drive(self.DRIVE_SPEED, 0)
                # # Drive forward for 3s
                # wait(3000)
                # self.log("Following line")

            elif brightness >= LIGHT:
                flag_tr = False
                self.log("Turning left")  # TODO remove debug log
                self.robot.drive(
                    0, 15
                )  # multiple drive operations can ovverride each other and are non-blocking -> robot does not jitter

            else:
                flag_tr = False
                delta = brightness - mid
                turn_rate = K * delta
                self.log("Driving")  # TODO remove debug log
                self.robot.drive(self.DRIVE_SPEED, turn_rate)
            wait(5)

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
