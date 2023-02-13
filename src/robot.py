import time

from pybricks.ev3devices import (ColorSensor, Motor, TouchSensor,
                                 UltrasonicSensor)
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

from state import State

ev3 = EV3Brick()


def resolve_states_requiring_store_state(states, on_enter_cb):
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

    # general functions used in every stage
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
            if callback is not None and callback() is True:
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

    # BEGIN CONDITION FUNCTIONS

    def check_blueline(self):
        r, g, b = self.color_sensor.rgb()
        return b >= 29 and g < 30 and r < 10

    def drove_distance(self, distance_in_mm):
        def drove_distance():
            delta = self.robot.distance() - self.prev_state[0]
            return delta >= distance_in_mm if distance_in_mm > 0 else delta <= distance_in_mm
        return drove_distance

    def drove_time(self, time_in_ms):
        def drove_time():
            delta = int(time.time() * 1000.0) - self.time
            return delta >= time_in_ms
        return drove_time

    def turned_degrees(self, degrees):
        def turned_degrees():
            delta = self.robot.angle() - self.prev_state[2]
            return delta >= degrees if degrees > 0 else delta <= degrees
        return turned_degrees

    def hit_rear(self):
        return self.touch_sensor.pressed()

    def swerved_angle(self, angle):
        def swerved_angle():
            delta = self.color_sensor_motor.angle() - self.swerve_angle
            if (angle > 0 and delta >= angle) or (angle <= 0 and delta <= angle):
                self.color_sensor_motor.hold()
                return True
            return False
        return swerved_angle

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
        LIGHT, DARK, K = 59, 16, 0.375
        GRAY = (LIGHT + DARK) / 2

        ON_LINE_TURN_RATE = 30
        ON_LINE_DRIVE_SPEED = 60

        SET_ANGLE_TURN_RATE = 40
        DRIVE_SPEED = 200

        GAP_TURNBACK_ANGLE = 102
        OBSTACLE_DISTANCE_THRESHOLD = 70

        OBSTACLE_STATE = "turn_left_before_obst"

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

        states = resolve_states_requiring_store_state(
            {
                "start": check_events([(is_gray, "drive_regulated"), (is_dark, "turn_right"), (is_light, "turn_left")], lambda: wait(Robot.WAIT_TIME)),

                # The different states of driving on the line
                "turn_left": State([(is_gray, "drive_regulated")], self.turn(ON_LINE_TURN_RATE)),
                "drive_regulated": check_events([(is_dark, "turn_right"), (is_light, "turn_left")], drive_regulated),
                "!turn_right": check_events([(is_gray, "drive_regulated"), (self.turned_degrees(-84), "turn_back_left")], self.turn(-ON_LINE_TURN_RATE)),

                # Cross a gap
                "!turn_back_left": State([(self.turned_degrees(GAP_TURNBACK_ANGLE), "cross_gap")], self.turn(100)),
                "!cross_gap": check_events([(is_gray, "start"), (is_light, "start"), (self.drove_distance(130), "start")], self.drive_straight(70)),

                # Drive around obstacle
                "!turn_left_before_obst": State([(self.turned_degrees(84), "drive_parallel_to_obst")], self.turn(SET_ANGLE_TURN_RATE)),
                "drive_parallel_to_obst": State([(self.drove_distance(172), "turn_parallel_to_line")], self.drive_straight(DRIVE_SPEED)),
                "!turn_parallel_to_line": State([(self.turned_degrees(-82), "drive_past_obst")], self.turn(-SET_ANGLE_TURN_RATE)),
                "drive_past_obst": State([(self.drove_distance(420), "turn_back_to_line")], self.drive_straight(DRIVE_SPEED)),
                "!turn_back_to_line": State([(self.turned_degrees(-81), "drive_back_to_line")], self.turn(-SET_ANGLE_TURN_RATE)),
                "drive_back_to_line": State([(self.drove_distance(172), "turn_back_to_original_orientation")], self.drive_straight(DRIVE_SPEED)),
                "!turn_back_to_original_orientation": State([(self.turned_degrees(84), "align_on_obst")], self.turn(SET_ANGLE_TURN_RATE)),
                "align_on_obst": State([(self.hit_rear, "start")], self.drive_back(DRIVE_SPEED)),
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
        states = resolve_states_requiring_store_state(
            {
                "!start": State([(self.drove_distance(200), "turn_right")], self.drive_straight(DRIVE_SPEED)),

                # Align on wall in the beginning
                "!turn_right": State([(self.turned_degrees(-82), "drive_back")], self.turn(-TURN_RATE)),
                "!drive_back": State([(self.hit_rear, "continue_driving_back")], self.drive_back(DRIVE_SPEED)),
                "!continue_driving_back": State([(self.drove_time(700), "drive_straight")], self.drive_back(DRIVE_SPEED)),
                "!drive_straight": State([(self.drove_distance(5), "turn_left")], self.drive_straight(100)),
                "!turn_left": State([(self.turned_degrees(81), "drive_straight2")], self.turn(TURN_RATE)),

                # Drive to the beginning of the area containing the box
                "!drive_straight2": State([(self.drove_distance(320), "turn_us_sensor")], self.drive_straight(DRIVE_SPEED)),
                "!turn_us_sensor": State([(self.swerved_angle(110), "turn_right2")], self.swerve(SWERVE_SPEED)),
                # Align on wall before searching for box
                "!turn_right2": State([(self.turned_degrees(-82), "drive_back2")], self.turn(-TURN_RATE)),
                "!drive_back2": State([(self.hit_rear, "continue_driving_back2")], self.drive_back(DRIVE_SPEED)),
                "!continue_driving_back2": State([(self.drove_time(1000), "drive_straight3")], self.drive_back(DRIVE_SPEED)),
                "!drive_straight3": State([(self.drove_distance(5), "turn_left2")], self.drive_straight(100)),
                "!turn_left2": State([(self.turned_degrees(82), "search")], self.turn(TURN_RATE)),

                # Drive next to the box, search 2 times to deal with wrong us sensor values
                "!search": State([(box_detected, "search2")], self.drive_straight(BOX_SEARCH_SPEED)),
                "!search2": State([(box_detected, "drive_next_to_box")], self.drive_straight(BOX_SEARCH_SPEED)),
                "!drive_next_to_box": State([(self.drove_distance(120), "turn_left3")], self.drive_straight(DRIVE_SPEED)),

                # Box is to the right of robot -> Robot pushed it to the wall with its back
                "!turn_left3": State([(self.turned_degrees(81), "push_box_edge1")], self.turn(TURN_RATE)),
                "!push_box_edge1": State([(True, "turn_sensor_back")], self.drive_back(DRIVE_SPEED)),
                "turn_sensor_back": State([(self.swerved_angle(-110), "push_box_edge2")], self.swerve(-SWERVE_SPEED)),
                "push_box_edge2": State([(self.drove_time(3000), "drive_straight4")], self.drive_back(DRIVE_SPEED)),

                # Robot rotates around the box anticlockwise and pushes it in the corner
                "!drive_straight4": State([(self.drove_distance(15), "turn_right3")], self.drive_straight(DRIVE_SPEED)),
                "!turn_right3": State([(self.turned_degrees(-77), "drive_back3")], self.turn(-TURN_RATE)),
                "!drive_back3": State([(self.drove_distance(-150), "turn_left4")], self.drive_back(DRIVE_SPEED)),
                "!turn_left4": State([(self.turned_degrees(84), "drive_back4")], self.turn(TURN_RATE)),
                "!drive_back4": State([(self.hit_rear, "continue_driving_back3")], self.drive_back(DRIVE_SPEED)),
                # Align before pushing box
                "!continue_driving_back3": State([(self.drove_time(700), "drive_straight5")], self.drive_back(DRIVE_SPEED)),
                "!drive_straight5": State([(self.drove_distance(1), "turn_left5")], self.drive_straight(50)),
                "!turn_left5": State([(self.turned_degrees(80), "push_box_corner")], self.turn(TURN_RATE)),
                "!push_box_corner": State([(self.drove_time(3000), "drive_straight6")], self.drive_back(DRIVE_SPEED)),

                # Drive to bridge
                "!drive_straight6": State([(self.drove_distance(90), "turn_right4")], self.drive_straight(DRIVE_SPEED)),
                "!turn_right4": State([(self.turned_degrees(-22), "drive_straight7")], self.turn(-TURN_RATE)),
                "!drive_straight7": check_events(
                    [(self.drove_distance(500), Robot.NEXT_LEVEL_STATE)], self.drive_straight(DRIVE_SPEED)
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

        states = resolve_states_requiring_store_state(
            {
                # Bring robot in a state where it can drive up the ramp
                "!start": State([(self.turned_degrees(26), "start2")], self.turn(TURN_RATE)),
                "!start2": State([(self.drove_distance(170), "start_driving")], self.drive_straight(DRIVE_SPEED)),
                "!start_driving": State([(True, "turn_color_sensor")], drive_left_curve),
                "!turn_color_sensor": State([(self.swerved_angle(70), "drive_to_bright_wood")], self.swerve(300)),

                # Drive to upper edge of ramp (there is a line of bright wood)
                "drive_to_bright_wood": State([(see_bright_wood, "drive_straight"), (see_void, "turn_right1")], drive_left_curve),
                "!turn_right1": State([(self.turned_degrees(-13), "drive_to_bright_wood")], self.turn(-TURN_RATE)),

                # Get ready to cross main part of bridge
                "!drive_straight": State([(self.drove_distance(200), "drive_to_void")], self.drive_straight(DRIVE_SPEED)),
                "!drive_to_void": State([(see_void, "drive_back")], self.drive_straight(DRIVE_SPEED)),
                "!drive_back": State([(self.drove_distance(-30), "turn_left")], self.drive_back(DRIVE_SPEED)),
                "!turn_left": State([(self.turned_degrees(80), "cross_bridge")], self.turn(TURN_RATE)),

                # Cross bridge
                "!cross_bridge": State([(see_void, "turn_right2")], drive_left_curve),
                "!turn_right2": State([(self.turned_degrees(-10), "check_void")], self.turn(-TURN_RATE)),
                "check_void": State([(see_void, "drive_back2"), (True, "cross_bridge")], None),

                # Get ready to drive down ramp
                "!drive_back2": State([(self.drove_distance(-40), "turn_left2")], self.drive_back(DRIVE_SPEED)),
                "!turn_left2": State([(self.turned_degrees(110), "drive_straight2")], self.turn(TURN_RATE)),

                # Drive down the ramp
                "!drive_straight2": State([(self.drove_distance(250), "drive_straight3")], self.drive_straight(DRIVE_SPEED)),
                "!drive_straight3": State([(see_void, "turn_right3"), (self.drove_distance(300), "start_driving2")], drive_left_curve),
                "!turn_right3": State([(self.turned_degrees(-13), "start_driving2")], self.turn(-TURN_RATE)),
                "!start_driving2": State([(True, "turn_back_color_sensor")], drive_left_curve),
                "!turn_back_color_sensor": State([(self.swerved_angle(-70), "drive_straight4")], self.swerve(-300)),
                "!drive_straight4": check_events([(self.drove_distance(600), Robot.NEXT_LEVEL_STATE)], self.drive_straight(DRIVE_SPEED)),
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
        self.found_red, self.found_white = False, False

        self.should_swerve = False
        self.beeped = False

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
            elif white and not self.found_white:
                self.found_white = True

            # Only beep if just first color is found
            if self.found_red != self.found_white and not self.beeped:
                ev3.speaker.beep(frequency=500, duration=10)
                self.beeped = True

        # Save information to check difference in future states
        def store_state():
            self.prev_state = self.robot.state()
            self.time = int(time.time() * 1000.0)

        def wall_detected():
            angle = self.color_sensor_motor.angle()
            return self.dist_sensor.distance() < OBSTACLE_DISTANCE_THRESHOLD and angle < 45

        def enable_swerving():
            self.should_swerve = True
            self.color_sensor_motor.run(SWERVE_SPEED)

        states = resolve_states_requiring_store_state(
            {
                # Adjust on walls to deal with random orientation after crossing bridge; last drive back to wall omitted and the one from the loop itself is used
                "!start": State([(self.drove_distance(300), "turn_parallel_to_left_wall")], self.drive_straight(3 * DRIVE_SPEED)),
                "!turn_parallel_to_left_wall": State([(self.turned_degrees(84), "drive_to_left_wall")], self.turn(TURN_RATE)),
                "!drive_to_left_wall": State([(self.drove_distance(200), "turn_back_to_left_wall")], self.drive_straight(3 * DRIVE_SPEED)),
                "!turn_back_to_left_wall": State([(self.turned_degrees(-84), "adjust_left_wall_touch")], self.turn(-TURN_RATE)),
                "adjust_left_wall_touch": State([(self.hit_rear, "adjust_left_wall_align")], self.drive_back(3 * DRIVE_SPEED)),
                "!adjust_left_wall_align": State([(self.drove_time(300), "detach_left_wall")], None),
                "!detach_left_wall": State([(self.drove_distance(5), "turn_back_to_right_wall")], self.drive_straight(DRIVE_SPEED)),
                "!turn_back_to_right_wall": State([(self.turned_degrees(84), "start_swerving")], self.turn(TURN_RATE)),

                # Color sensor starts swerving
                "start_swerving": State([(True, "adjust_before_drive")], enable_swerving),

                # Drive across field then turn right (just turning in place because color sensor is on the left side of the robot)
                "drive_until_wall1": State([(wall_detected, "stop_and_turn_right")], self.drive_straight(DRIVE_SPEED)),
                "!stop_and_turn_right": State([(self.turned_degrees(-84), "turn_right_before_drive")], self.turn(-TURN_RATE)),
                "!turn_right_before_drive": State([(self.turned_degrees(-84), "adjust_before_drive2")], self.turn(-TURN_RATE)),
                "adjust_before_drive2": State([(self.hit_rear, "continue_driving_back2")], self.drive_back(DRIVE_SPEED)),
                "!continue_driving_back2": State([(self.drove_time(700), "drive_until_wall2")], self.drive_back(DRIVE_SPEED)),

                # Drive across field then turn left
                "drive_until_wall2": State([(wall_detected, "stop_and_turn_left")], self.drive_straight(DRIVE_SPEED)),
                "!stop_and_turn_left": State([(self.turned_degrees(84), "drive_before_left")], self.turn(TURN_RATE)),
                "!drive_before_left": State([(self.drove_distance(SHORT_DRIVE_DISTANCE), "turn_left_before_drive")], self.drive_straight(DRIVE_SPEED)),
                "!turn_left_before_drive": State([(self.turned_degrees(84), "adjust_before_drive")], self.turn(TURN_RATE)),
                "adjust_before_drive": State([(self.hit_rear, "continue_driving_back")], self.drive_back(DRIVE_SPEED)),
                "!continue_driving_back": State([(self.drove_time(700), "drive_until_wall1")], None),
            },
            self.store_state,
        )

        cur_state = states["start"]
        cur_state.on_enter()

        def callback():
            if self.should_swerve:
                swerve_color_sensor()
            check_colors()
            if self.found_red and self.found_white:
                self.stop()
                ev3.speaker.beep(frequency=300, duration=200)
                self.course.running = False
                return True

        self.level_loop(states, callback)
        self.stop()
