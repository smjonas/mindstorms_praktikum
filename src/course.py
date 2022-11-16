from enum import Enum

from pybricks.hubs import EV3Brick
from pybricks.parameters import Button
ev3 = EV3Brick()


class Course:
    def __init__(self):
        self.running = False
        self.selected_stage = self.Stage.LINE

    def should_abort(self):
        abort = Button.LEFT in ev3.buttons.pressed()
        if abort:
            self.running = False
        return abort
    
    class Stage(Enum):
        LINE = 0
        OBSTACLE = 1
        BRIDGE = 2
        FIELD = 3

        def to_string(Stage):
            return {
                Stage.LINE: "Follow lines",
                Stage.OBSTACLE: "Move obstacle",
                Stage.BRIDGE: "Cross bridge",
                Stage.FIELD: "Color field",
            }[Stage]
