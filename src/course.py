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
    
    class Stage:
        COUNT = 4
        LINE, OBSTACLE, BRIDGE, FIELD = range(COUNT)

        def to_string(stage):
            return {
                Course.Stage.LINE: "Follow lines",
                Course.Stage.OBSTACLE: "Move obstacle",
                Course.Stage.BRIDGE: "Cross bridge",
                Course.Stage.FIELD: "Color field",
            }[stage]
