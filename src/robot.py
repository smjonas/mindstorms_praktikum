from pybricks.ev3devices import (ColorSensor, Motor, TouchSensor,
                                 UltrasonicSensor)
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait

from state import State

ev3 = EV3Brick()


class Robot:
    def __init__(self, course):
        self.course = course

        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.D)
        # Since we used tracks, axle_track cannot be measured in a sensible way
        # We set it to the value where it does a 90 degree turn on the parcour floor if we tell it to
        # (other angles do not work therefore).
        self.robot = DriveBase(
            left_motor=self.left_motor, right_motor=self.right_motor, wheel_diameter=56, axle_track=166.5
        )
        self.color_sensor_motor = Motor(Port.C)

        self.color_sensor = ColorSensor(Port.S2)
        self.dist_sensor = UltrasonicSensor(Port.S4)
        self.touch_sensor = TouchSensor(Port.S3)

        self.stage_fns = {
            course.Stage.LINE: Robot.lines,
            course.Stage.DELIVERY: Robot.delivery,
            course.Stage.BRIDGE: Robot.bridge,
            course.Stage.FIELD: Robot.field,
        }

    # def reset_color_sensor_motor(self):
    #     self.color_sensor_motor.run_time(-30, 1000)
    #     self.color_sensor_motor.stop()
    #     # self.color_sensor_motor.reset_angle(0)

    def stop(self):
        self.robot.stop()
        self.color_sensor_motor.stop()

    def log(self, msg=""):
        ev3.screen.print(msg)

    def start_stage(self):
        ev3.screen.clear()
        self.log("Stage " + str(self.course.selected_stage + 1) + " started")
        self.stage_fns[self.course.selected_stage](self)

    def check_blueline(self):
        r, g, b = self.color_sensor.rgb()
        return b >= 29 and g < 30 and r < 10

    # Methods for stages of course
    # Stage 1: Following a white line with gaps and an obstacle
    def lines(self):
        LIGHT, DARK, K = 60, 15, 0.01
        GRAY = (LIGHT + DARK) / 2

        ON_LINE_TURN_RATE = 30
        ON_LINE_DRIVE_SPEED = 60

        SET_ANGLE_TURN_RATE = 40
        SET_DISTANCE_DRIVE_SPEED = 100

        GAP_TURNBACK_ANGLE = 105
        OBSTACLE_DISTANCE_THRESHOLD = 70

        WAIT_TIME = 10
        ANGLE_FOR_90_DEGREES = 84

        NEXT_LEVEL_STATE, OBSTACLE_STATE = "next_level", "store_and_obst_1"

        # Check brightness
        def is_dark():
            return self.color_sensor.reflection() <= DARK

        def is_light():
            return self.color_sensor.reflection() >= LIGHT

        def is_gray():
            brightness = self.color_sensor.reflection()
            return brightness > DARK and brightness < LIGHT

        # Save information to check difference in future states
        def store_state():
            self.prev_state = self.robot.state()

        # Check differences in state
        def did_turn(degrees):
            def did_turn_degrees():
                delta = self.robot.angle() - self.prev_state[2]
                return delta >= degrees if degrees > 0 else delta <= degrees
            return did_turn_degrees

        def did_drive(distance_in_mm):
            def did_drive_distance():
                delta = self.robot.distance() - self.prev_state[0]
                return delta >= distance_in_mm if distance_in_mm > 0 else delta <= distance_in_mm
            return did_drive_distance

        def hit_rear():
            return self.touch_sensor.pressed()

        # Nonblocking methods to tell the robot what to do
        def turn(turn_rate):
            def turn_angle():
                self.robot.drive(0, turn_rate)
                wait(WAIT_TIME)
            return turn_angle

        def drive_regulated():
            brightness = self.color_sensor.reflection()
            # if brightness <= 20:
                # turn_rate = 50
            # else:
            delta = brightness - GRAY
            turn_rate = K * delta
            self.robot.drive(ON_LINE_DRIVE_SPEED, turn_rate)
            wait(WAIT_TIME)

        def drive_straight():
            self.robot.drive(SET_DISTANCE_DRIVE_SPEED, 0)
            wait(WAIT_TIME)

        def drive_back():
            self.robot.drive(-SET_DISTANCE_DRIVE_SPEED, 0)
            wait(WAIT_TIME)

        # Defining states
        def check_events(transitions, on_enter=None):
            new_transitions = [
                (self.check_blueline, NEXT_LEVEL_STATE),
                (lambda: self.dist_sensor.distance() < OBSTACLE_DISTANCE_THRESHOLD, OBSTACLE_STATE),
            ]
            return State(new_transitions + transitions, on_enter)

        states = {
            "start": check_events(
                [(is_gray, "drive_regulated"), (is_dark, "store_and_turn_right"), (is_light, "turn_left")],
                lambda: wait(WAIT_TIME),
            ),
            "turn_left": State([(is_gray, "drive_regulated")], turn(ON_LINE_TURN_RATE)),
            "drive_regulated": check_events(
                [(is_dark, "store_and_turn_right"), (is_light, "turn_left")], drive_regulated
            ),
            "store_and_turn_right": State([(True, "turn_right")], store_state),
            "turn_right": check_events(
                [(is_gray, "drive_regulated"), (did_turn(-ANGLE_FOR_90_DEGREES), "store_and_turn_back_left")],
                turn(-ON_LINE_TURN_RATE),
            ),
            "store_and_turn_back_left": State([(True, "turn_back_left")], store_state),
            "turn_back_left": State([(did_turn(GAP_TURNBACK_ANGLE), "store_and_drive_straight")], turn(SET_ANGLE_TURN_RATE)),
            "store_and_drive_straight": check_events([(True, "drive_straight")], store_state),
            "drive_straight": check_events([(is_gray, "start"), (is_light, "start"), (did_drive(110), "start")], drive_straight),
            "store_and_obst_1": State([(True, "obst_1")], store_state),
            "obst_1": State([(did_turn(ANGLE_FOR_90_DEGREES), "obst_2")], turn(SET_ANGLE_TURN_RATE)),
            "obst_2": State([(did_drive(172), "store_and_obst_3")], drive_straight),
            "store_and_obst_3": State([(True, "obst_3")], store_state),
            # Not just angle for 90 degrees to make sure we do not crash into the obstacle
            "obst_3": State([(did_turn(-ANGLE_FOR_90_DEGREES + 4), "obst_4")], turn(-SET_ANGLE_TURN_RATE)),
            "obst_4": State([(did_drive(500), "store_and_obst_5")], drive_straight),
            "store_and_obst_5": State([(True, "obst_5")], store_state),
            "obst_5": State([(did_turn(-ANGLE_FOR_90_DEGREES), "obst_6")], turn(-SET_ANGLE_TURN_RATE)),
            "obst_6": State([(did_drive(150), "store_and_obst_7")], drive_straight),
            "store_and_obst_7": State([(True, "obst_7")], store_state),
            "obst_7": State([(did_turn(ANGLE_FOR_90_DEGREES), "obst_8")], turn(SET_ANGLE_TURN_RATE)),
            # TODO: Decide if we want to drive back, maybe change value in obst_6 to be closer to the line
            "obst_8": State([(hit_rear, "start"), (True, "obst_8")], drive_back)
        }
        cur_state = states["start"]

        # Actual loop for this stage
        while not self.course.should_abort():
            successor = cur_state.check_conditions()
            if successor:
                cur_state = states.get(successor, successor)
                if cur_state == NEXT_LEVEL_STATE:
                    self.stop()
                    self.course.running = False
                    self.log("Reached end of line")
                    return

                if cur_state.on_enter:
                    cur_state.on_enter()
        self.robot.stop()

    # Stage 2: Push a block in the corner
    def delivery(self):
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
        DRIVE_SPEED = 90
        ANGLE_FOR_90_DEGREES = 84
        TURN_RATE = 70
        SHORT_DRIVE_DISTANCE = 140
        OBSTACLE_DISTANCE_THRESHOLD = 100
        WAIT_TIME = 10

        SET_DISTANCE_DRIVE_SPEED = 100
        SWERVE_SPEED = 300
        MOTOR_MIN_ANGLE, MOTOR_MAX_ANGLE = 0, 90
        self.motor_target = MOTOR_MAX_ANGLE
        self.color_sensor_motor.run(SWERVE_SPEED)
        found_red, found_white = False, False

        def swerve_color_sensor():
            angle = self.color_sensor_motor.angle()
            if self.motor_target == MOTOR_MIN_ANGLE and angle <= MOTOR_MIN_ANGLE:
                # self.color_sensor_motor.stop()
                self.motor_target = MOTOR_MAX_ANGLE
                self.color_sensor_motor.run(SWERVE_SPEED)
            elif self.motor_target == MOTOR_MAX_ANGLE and angle >= MOTOR_MAX_ANGLE:
                # self.color_sensor_motor.stop()
                self.motor_target = MOTOR_MIN_ANGLE
                self.color_sensor_motor.run(-SWERVE_SPEED)

            self.log(str(self.color_sensor_motor.angle()))

        def check_colors():
            r, g, b = self.color_sensor.rgb()
            red = r > 20 and g < 15 and b < 10
            white = r > 40 and g > 60 and b > 50
            if red:
                found_red = True
                ev3.speaker.beep(frequency=100, duration=1000)
            elif white:
                found_white = True
                ev3.speaker.beep(frequency=500, duration=1000)

        # Nonblocking methods to tell the robot what to do
        def turn(turn_rate):
            def turn_angle():
                self.robot.drive(0, turn_rate)
                wait(WAIT_TIME)
            return turn_angle

        def drive_straight():
            self.robot.drive(DRIVE_SPEED, 0)
            wait(WAIT_TIME)

        # Save information to check difference in future states
        def store_state():
            self.prev_state = self.robot.state()

        # Check differences in state
        def did_turn(degrees):
            def did_turn_degrees():
                delta = self.robot.angle() - self.prev_state[2]
                return delta >= degrees if degrees > 0 else delta <= degrees
            return did_turn_degrees

        def did_drive(distance_in_mm):
            def did_drive_distance():
                delta = self.robot.distance() - self.prev_state[0]
                return delta >= distance_in_mm if distance_in_mm > 0 else delta <= distance_in_mm
            return did_drive_distance

        def wall_detected():
            return self.dist_sensor.distance() < OBSTACLE_DISTANCE_THRESHOLD

        def hit_rear():
            return self.touch_sensor.pressed()

        def drive_back():
            self.robot.drive(-SET_DISTANCE_DRIVE_SPEED, 0)
            wait(WAIT_TIME)

        states = {
            "start'": State([(True, "start")], store_state),
            "start": State([(did_turn(ANGLE_FOR_90_DEGREES), "adjust_before_drive")], turn(TURN_RATE)),
            "adjust_before_drive": State([(hit_rear, "check_wall_before_right"), (True, "adjust_before_drive")], drive_back),
            "check_wall_before_right": State([(wall_detected, "stop_and_turn_right'")], drive_straight),
            "stop_and_turn_right'": State([(True, "stop_and_turn_right")], store_state),
            "stop_and_turn_right": State([(did_turn(-ANGLE_FOR_90_DEGREES), "turn_right_before_drive'")], turn(-TURN_RATE)),
            "turn_right_before_drive'": State([(True, "turn_right_before_drive")], store_state),
            "turn_right_before_drive": State([(did_turn(-ANGLE_FOR_90_DEGREES), "adjust_before_drive2'")], turn(-TURN_RATE)),
            "adjust_before_drive2": State([(hit_rear, "check_wall_before_left"), (True, "adjust_before_drive2")], drive_back),
            "check_wall_before_left'": State([(True, "check_wall_before_left")], store_state),
            "check_wall_before_left": State([(wall_detected, "stop_and_turn_left'")], drive_straight),
            "stop_and_turn_left'": State([(True, "stop_and_turn_left")], store_state),
            "stop_and_turn_left": State([(did_turn(ANGLE_FOR_90_DEGREES), "drive_before_left'")], turn(TURN_RATE)),
            "drive_before_left'": State([(True, "drive_before_left")], store_state),
            "drive_before_left": State([(did_drive(SHORT_DRIVE_DISTANCE), "start'")], drive_straight)
        }
        cur_state = states["start'"]

        store_state()
        # Actual loop for this stage
        while not self.course.should_abort():
            swerve_color_sensor()
            check_colors()
            if found_red and found_white:
                self.course.running = False
                break
            successor = cur_state.check_conditions()
            if successor:
                print("CUR STATE ", successor)
                cur_state = states[successor]
                if cur_state.on_enter:
                    cur_state.on_enter()
        self.stop()

