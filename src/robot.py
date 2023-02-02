import time

from pybricks.ev3devices import (ColorSensor, Motor, TouchSensor,
                                 UltrasonicSensor)
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

from state import State

ev3 = EV3Brick()


def resolve_stored_states(states, on_enter_cb):
    for key, state in states.copy().items():
        # Turn store_state("start") == "start!": state into the two key-value-pairs...
        if key.startswith("!"):
            # "start": State([(True, "start!")], on_enter_cb)
            states[key[1:]] = State([(True, key)], on_enter_cb)
            # and "start!": state
            states[key] = state
    return states


class Robot:
    WAIT_TIME = 5
    ANGLE_FOR_90_DEGREES = 84
    NEXT_LEVEL_STATE = "next_level"

    def __init__(self, course):
        self.course = course

        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.D)
        # Since we used tracks, axle_track cannot be measured in a sensible way
        # We set it to the value where it does a 90 degree turn on the parcour floor if we tell it to
        # (other angles do not work therefore).
        self.robot = DriveBase(left_motor=self.left_motor, right_motor=self.right_motor, wheel_diameter=56, axle_track=166.5)
        self.color_sensor_motor = Motor(Port.C)

        self.color_sensor = ColorSensor(Port.S2)
        self.dist_sensor = UltrasonicSensor(Port.S4)
        self.touch_sensor = TouchSensor(Port.S1)

        self.stage_fns = {
            course.Stage.LINE: Robot.lines,
            course.Stage.DELIVERY: Robot.delivery,
            course.Stage.BRIDGE: Robot.bridge,
            course.Stage.FIELD: Robot.field,
        }

    def store_state(self):
        self.prev_state = self.robot.state()
        self.dist = self.dist_sensor.distance()
        self.time = int(time.time() * 1000.0)
        self.swerve_angle = self.color_sensor_motor.angle()

    def stop(self):
        self.robot.stop()
        self.color_sensor_motor.stop()

    def log(self, msg=""):
        ev3.screen.print(msg)

    def start_stage(self):
        ev3.screen.clear()
        self.log("Stage " + str(self.course.selected_stage + 1) + " started")
        self.stage_fns[self.course.selected_stage](self)

    def level_loop(self, states, callback=None):
        cur_state = states["start"]
        cur_state.on_enter()

        while not self.course.should_abort():
            if callback is not None and callback() == True:
                return
            successor = cur_state.check_conditions()
            if successor:
                cur_state = states.get(successor, successor)
                self.log(successor)
                if cur_state == Robot.NEXT_LEVEL_STATE:
                    self.stop()
                    return

                if cur_state.on_enter:
                    cur_state.on_enter()

    def check_blueline(self):
        r, g, b = self.color_sensor.rgb()
        return b >= 29 and g < 30 and r < 10

    # BEGIN CONDITION FUNCTIONS
    def did_drive(self, distance_in_mm):
        def did_drive_distance():
            delta = self.robot.distance() - self.prev_state[0]
            return delta >= distance_in_mm if distance_in_mm > 0 else delta <= distance_in_mm
        return did_drive_distance

    def did_drive_time(self, time_in_ms):
        def did_drive_time():
            delta = int(time.time() * 1000.0) - self.time
            return delta >= time_in_ms
        return did_drive_time

    def did_turn(self, degrees):
        def did_turn_degrees():
            delta = self.robot.angle() - self.prev_state[2]
            return delta >= degrees if degrees > 0 else delta <= degrees
        return did_turn_degrees

    def hit_rear(self):
        return self.touch_sensor.pressed()

    def did_swerve_angle(self, angle):
        def did_swerve_angle():
            delta = self.color_sensor_motor.angle() - self.swerve_angle
            if (angle > 0 and delta >= angle) or (angle <= 0 and delta <= angle):
                self.color_sensor_motor.hold()
                return True
            return False
        return did_swerve_angle

    # END CONDITION FUNCTIONS

    # BEGIN ON_ENTER FUNCTIONS
    def drive_straight(self, drive_speed):
        def drive_straight_speed():
            self.robot.drive(drive_speed, 0)
            wait(Robot.WAIT_TIME)
        return drive_straight_speed

    def drive_back(self, drive_speed):
        return self.drive_straight(-drive_speed)

    def turn(self, turn_rate):
        def turn_angle():
            self.robot.drive(0, turn_rate)
            wait(Robot.WAIT_TIME)
        return turn_angle

    def swerve(self, speed):
        def swerve():
            self.color_sensor_motor.run(speed)
        return swerve
    # END ON_ENTER FUNCTIONS

    # Methods for stages of course
    # Stage 1: Following a white line with gaps and an obstacle
    def lines(self):
        LIGHT, DARK, K = 65, 18, 0.375
        GRAY = (LIGHT + DARK) / 2

        ON_LINE_TURN_RATE = 30
        ON_LINE_DRIVE_SPEED = 60

        SET_ANGLE_TURN_RATE = 40
        DRIVE_SPEED = 200

        GAP_TURNBACK_ANGLE = 110
        OBSTACLE_DISTANCE_THRESHOLD = 70

        OBSTACLE_STATE = "obst_1"

        # Check brightness
        def is_dark():
            return self.color_sensor.reflection() <= DARK

        def is_light():
            return self.color_sensor.reflection() >= LIGHT

        def is_gray():
            brightness = self.color_sensor.reflection()
            return brightness > DARK and brightness < LIGHT

        def drive_regulated():
            brightness = self.color_sensor.reflection()
            # if brightness <= 20:
            #     turn_rate = 50
            # else:
            delta = brightness - GRAY
            turn_rate = K * delta
            self.robot.drive(ON_LINE_DRIVE_SPEED, turn_rate)

        # Defining states
        def check_events(transitions, on_enter=None):
            new_transitions = [
                (self.check_blueline, Robot.NEXT_LEVEL_STATE),
                (lambda: self.dist_sensor.distance() < OBSTACLE_DISTANCE_THRESHOLD, OBSTACLE_STATE),
            ]
            return State(new_transitions + transitions, on_enter)

        states = resolve_stored_states(
            {
                "start": check_events(
                    [(is_gray, "drive_regulated"), (is_dark, "turn_right"), (is_light, "turn_left")],
                    lambda: wait(Robot.WAIT_TIME),
                ),
                "turn_left": State([(is_gray, "drive_regulated")], self.turn(ON_LINE_TURN_RATE)),
                "drive_regulated": check_events([(is_dark, "turn_right"), (is_light, "turn_left")], drive_regulated),
                "!turn_right": check_events(
                    [
                        (is_gray, "drive_regulated"),
                        (self.did_turn(-Robot.ANGLE_FOR_90_DEGREES), "turn_back_left"),
                    ],
                    self.turn(-ON_LINE_TURN_RATE),
                ),
                "!turn_back_left": State([(self.did_turn(GAP_TURNBACK_ANGLE), "drive_straight")], self.turn(100)),
                "!drive_straight": check_events(
                    [(is_gray, "start"), (is_light, "start"), (self.did_drive(130), "start")],
                    self.drive_straight(70),
                ),
                # TODO: vor obst1 schon einmal an obstacle ausrichten
                "!obst_1": State([(self.did_turn(Robot.ANGLE_FOR_90_DEGREES), "obst_2")], self.turn(SET_ANGLE_TURN_RATE)),
                "obst_2": State([(self.did_drive(172), "obst_3")], self.drive_straight(DRIVE_SPEED)),
                # Not just angle for 90 degrees to make sure we do not crash into the obstacle
                "!obst_3": State(
                    [(self.did_turn(-82), "obst_4")],
                    self.turn(-SET_ANGLE_TURN_RATE),
                ),
                "obst_4": State([(self.did_drive(420), "obst_5")], self.drive_straight(DRIVE_SPEED)),
                "!obst_5": State([(self.did_turn(-81), "obst_6")], self.turn(-SET_ANGLE_TURN_RATE)),
                "obst_6": State([(self.did_drive(172), "obst_7")], self.drive_straight(DRIVE_SPEED)),
                "!obst_7": State([(self.did_turn(Robot.ANGLE_FOR_90_DEGREES), "obst_8")], self.turn(SET_ANGLE_TURN_RATE)),
                "obst_8": State([(self.hit_rear, "start")], self.drive_back(DRIVE_SPEED)),
            },
            self.store_state,
        )

        self.level_loop(states)
        self.robot.stop()

    # Stage 2: Push a block in the corner
    def delivery(self):
        TURN_RATE = 70
        DRIVE_SPEED = 300

        SWERVE_SPEED = 60
        MAX_BOX_DISTANCE = 340
        BOX_SEARCH_SPEED = 100

        def box_detected():
            dist = self.dist_sensor.distance()
            return dist < MAX_BOX_DISTANCE

        def check_events(transitions, on_enter=None):
            new_transitions = [
                (self.check_blueline, Robot.NEXT_LEVEL_STATE),
            ]
            return State(new_transitions + transitions, on_enter)

        self.stop()
        states = resolve_stored_states(
            {
                "!start": State([(self.did_drive(200), "turn_right")], self.drive_straight(DRIVE_SPEED)),
                "!turn_right": State([(self.did_turn(-82), "drive_back")], self.turn(-TURN_RATE)),
                "!drive_back": State([(self.hit_rear, "continue_driving_back")], self.drive_back(DRIVE_SPEED)),
                "!continue_driving_back": State([(self.did_drive_time(700), "drive_straight")], self.drive_back(DRIVE_SPEED)),
                "!drive_straight": State([(self.did_drive(5), "turn_left")], self.drive_straight(100)),
                "!turn_left": State([(self.did_turn(81), "drive_straight2")], self.turn(TURN_RATE)),
                "!drive_straight2": State([(self.did_drive(320), "turn_us_sensor")], self.drive_straight(DRIVE_SPEED)),
                "!turn_us_sensor": State([(self.did_swerve_angle(110), "turn_right2")], self.swerve(SWERVE_SPEED)),
                # Align at wall before searching for box
                "!turn_right2": State([(self.did_turn(-82), "drive_back2")], self.turn(-TURN_RATE)),
                "!drive_back2": State([(self.hit_rear, "continue_driving_back2")], self.drive_back(DRIVE_SPEED)),
                "!continue_driving_back2": State([(self.did_drive_time(1000), "drive_straight3")], self.drive_back(DRIVE_SPEED)),
                "!drive_straight3": State([(self.did_drive(5), "turn_left2")], self.drive_straight(100)),
                "!turn_left2": State([(self.did_turn(82), "search")], self.turn(TURN_RATE)),
                "!search": State([(box_detected, "search2")], self.drive_straight(BOX_SEARCH_SPEED)),
                "!search2": State([(box_detected, "drive_next_to_box")], self.drive_straight(BOX_SEARCH_SPEED)),
                "!drive_next_to_box": State([(self.did_drive(120), "turn_left3")], self.drive_straight(DRIVE_SPEED)),
                "!turn_left3": State([(self.did_turn(81), "push_box_edge1")], self.turn(TURN_RATE)),
                "!push_box_edge1": State([(True, "turn_sensor_back")], self.drive_back(DRIVE_SPEED)),
                "turn_sensor_back": State([(self.did_swerve_angle(-110), "push_box_edge2")], self.swerve(-SWERVE_SPEED)),
                "push_box_edge2": State([(self.did_drive_time(3000), "drive_straight4")], self.drive_back(DRIVE_SPEED)),
                "!drive_straight4": State([(self.did_drive(15), "turn_right3")], self.drive_straight(DRIVE_SPEED)),
                "!turn_right3": State([(self.did_turn(-77), "drive_back3")], self.turn(-TURN_RATE)),
                "!drive_back3": State([(self.did_drive(-150), "turn_left4")], self.drive_back(DRIVE_SPEED)),
                "!turn_left4": State([(self.did_turn(84), "drive_back4")], self.turn(TURN_RATE)),
                "!drive_back4": State([(self.hit_rear, "continue_driving_back3")], self.drive_back(DRIVE_SPEED)),
                # Align before pushing box
                "!continue_driving_back3": State([(self.did_drive_time(700), "drive_straight5")], self.drive_back(DRIVE_SPEED)),
                "!drive_straight5": State([(self.did_drive(5), "turn_left5")], self.drive_straight(50)),
                "!turn_left5": State([(self.did_turn(80), "push_box_corner")], self.turn(TURN_RATE)),
                "!push_box_corner": State([(self.did_drive_time(3000), "drive_straight6")], self.drive_back(DRIVE_SPEED)),
                "!drive_straight6": State([(self.did_drive(90), "turn_right4")], self.drive_straight(DRIVE_SPEED)),
                "!turn_right4": State([(self.did_turn(-22), "drive_straight7")], self.turn(-TURN_RATE)),
                "!drive_straight7": check_events(
                    [(self.did_drive(500), Robot.NEXT_LEVEL_STATE)], self.drive_straight(DRIVE_SPEED)
                ),
            },
            self.store_state,
        )

        self.level_loop(states)
        self.stop()

    # Stage 3: Cross a bridge with perpendicular ramps leading up/down at the ends
    def bridge(self):
        DRIVE_SPEED = 150
        TURN_RATE = 70

        def see_bright_wood():
            brightness = self.color_sensor.reflection()
            return brightness > 20 and brightness < 40

        def see_void():
            brightness = self.color_sensor.reflection()
            return brightness <= 0

        def check_events(transitions, on_enter=None):
            new_transitions = [
                (self.check_blueline, Robot.NEXT_LEVEL_STATE),
            ]
            return State(new_transitions + transitions, on_enter)

        def drive_left_curve():
            self.robot.drive(DRIVE_SPEED, 3)
            wait(Robot.WAIT_TIME)

        states = resolve_stored_states(
            {
                "!start": State([(self.did_turn(22), "start2")], self.turn(TURN_RATE)),
                "!start2": State([(self.did_drive(170), "start_driving")], self.drive_straight(DRIVE_SPEED)),
                "!start_driving": State([(True, "turn_color_sensor")], drive_left_curve),
                "!turn_color_sensor": State([(self.did_swerve_angle(70), "drive_to_bright_wood")], self.swerve(300)),
                "drive_to_bright_wood": State([(see_bright_wood, "drive_straight"), (see_void, "turn_right1")], drive_left_curve),

                "!turn_right1": State([(self.did_turn(-13), "drive_to_bright_wood")], self.turn(-TURN_RATE)),

                "!drive_straight": State([(self.did_drive(200), "drive_to_void")], self.drive_straight(DRIVE_SPEED)),
                "!drive_to_void": State([(see_void, "drive_back")], self.drive_straight(DRIVE_SPEED)),
                "!drive_back": State([(self.did_drive(-30), "turn_left")], self.drive_back(DRIVE_SPEED)),
                "!turn_left": State([(self.did_turn(80), "cross_bridge")], self.turn(TURN_RATE)),
                "!cross_bridge": State([(see_void, "turn_right2")], drive_left_curve),

                "!turn_right2": State([(self.did_turn(-10), "check_void")], self.turn(-TURN_RATE)),
                "check_void": State([(see_void, "drive_back2"), (True, "cross_bridge")], None),

                "!drive_back2": State([(self.did_drive(-40), "turn_left2")], self.drive_back(DRIVE_SPEED)),
                "!turn_left2": State([(self.did_turn(110), "drive_straight2")], self.turn(TURN_RATE)),

                "!drive_straight2": State([(self.did_drive(250), "drive_straight3")], self.drive_straight(DRIVE_SPEED)),
                "!drive_straight3": State([(see_void, "turn_right3"), (self.did_drive(300), "start_driving2")], drive_left_curve),
                "!turn_right3": State([(self.did_turn(-13), "start_driving2")], self.turn(-TURN_RATE)),
                "!start_driving2": State([(True, "turn_back_color_sensor")], drive_left_curve),
                "!turn_back_color_sensor": State([(self.did_swerve_angle(-70), "drive_straight4")], self.swerve(-300)),
                "!drive_straight4": check_events([(self.did_drive(600), Robot.NEXT_LEVEL_STATE)], self.drive_straight(DRIVE_SPEED)),
            },
            self.store_state,
        )

        self.level_loop(states)
        self.stop()

    # Stage 4: Find a red and white patch of color on the floor
    def field(self):
        DRIVE_SPEED = 90
        TURN_RATE = 70
        SHORT_DRIVE_DISTANCE = 140
        OBSTACLE_DISTANCE_THRESHOLD = 100

        DRIVE_SPEED = 100
        SWERVE_SPEED = 300
        MOTOR_MIN_ANGLE, MOTOR_MAX_ANGLE = 0, 90
        self.motor_target = MOTOR_MAX_ANGLE
        self.color_sensor_motor.run(SWERVE_SPEED)
        self.found_red, self.found_white = False, False

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

        def check_colors():
            r, g, b = self.color_sensor.rgb()
            red = r > 20 and g < 15 and b < 10
            white = r > 40 and g > 60 and b > 50
            if red and not self.found_red:
                self.found_red = True
                ev3.speaker.beep(frequency=100, duration=10)
            elif white and not self.found_white:
                self.found_white = True
                ev3.speaker.beep(frequency=500, duration=10)

        # Save information to check difference in future states
        def store_state():
            self.prev_state = self.robot.state()
            self.time = int(time.time() * 1000.0)

        def wall_detected():
            angle = self.color_sensor_motor.angle()
            return self.dist_sensor.distance() < OBSTACLE_DISTANCE_THRESHOLD and angle < 45

        states = resolve_stored_states(
            {
                "!start": State([(self.did_drive(-30), "turn_right")], self.drive_back(DRIVE_SPEED)),
                "!turn_right": State(
                    [(self.did_turn(-Robot.ANGLE_FOR_90_DEGREES), "adjust_before_drive")],
                    self.turn(-TURN_RATE),
                ),
                "adjust_before_drive": State([(self.hit_rear, "continue_driving_back")], self.drive_back(DRIVE_SPEED)),
                "!continue_driving_back": State(
                    [(self.did_drive_time(700), "check_wall_before_right")], self.drive_back(DRIVE_SPEED)
                ),
                "check_wall_before_right": State([(wall_detected, "stop_and_turn_right")], self.drive_straight(DRIVE_SPEED)),
                "!stop_and_turn_right": State(
                    [(self.did_turn(-Robot.ANGLE_FOR_90_DEGREES), "turn_right_before_drive")],
                    self.turn(-TURN_RATE),
                ),
                "!turn_right_before_drive": State(
                    [(self.did_turn(-Robot.ANGLE_FOR_90_DEGREES), "adjust_before_drive2")],
                    self.turn(-TURN_RATE),
                ),
                "adjust_before_drive2": State([(self.hit_rear, "continue_driving_back2")], self.drive_back(DRIVE_SPEED)),
                "!continue_driving_back2": State(
                    [(self.did_drive_time(700), "check_wall_before_left")], self.drive_back(DRIVE_SPEED)
                ),
                "check_wall_before_left": State([(wall_detected, "stop_and_turn_left")], self.drive_straight(DRIVE_SPEED)),
                "!stop_and_turn_left": State(
                    [(self.did_turn(Robot.ANGLE_FOR_90_DEGREES), "drive_before_left")], self.turn(TURN_RATE)
                ),
                "!drive_before_left": State([(self.did_drive(SHORT_DRIVE_DISTANCE), "start")], self.drive_straight(DRIVE_SPEED)),
                "!turn_left": State([(self.did_turn(Robot.ANGLE_FOR_90_DEGREES), "adjust_before_drive")], self.turn(TURN_RATE)),
            },
            self.store_state,
        )

        cur_state = states["start"]
        cur_state.on_enter()

        def callback():
            swerve_color_sensor()
            check_colors()
            if self.found_red and self.found_white:
                self.course.running = False
                return True

        self.level_loop(states, callback)
        self.stop()
