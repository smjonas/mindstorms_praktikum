from course import Course

from pybricks.hubs import EV3Brick
from pybricks.parameters import Button


ev3 = EV3Brick()


class Menu:
    SCREEN_WIDTH = ev3.screen.width
    SCREEN_HEIGHT = ev3.screen.height

    def __init__(self, course):
        self.course = course
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

    def redraw(self):
        ev3.screen.clear()
        # A few magic numbers here to make the menu look good
        for i in range(Course.Stage.COUNT):
            ev3.screen.draw_text(3, i * (Menu.SCREEN_HEIGHT / Course.Stage.COUNT), self.course.Stage.to_string(i))
        ev3.screen.draw_box(
            1,
            self.course.selected_stage * (Menu.SCREEN_HEIGHT / Course.Stage.COUNT),
            Menu.SCREEN_WIDTH - 1,
            (self.course.selected_stage + 1) * (Menu.SCREEN_HEIGHT / Course.Stage.COUNT) - 8,
        )

    def select_next(self):
        self.course.selected_stage = (self.course.selected_stage + 1) % Course.Stage.COUNT
        self.redraw()

    def select_prev(self):
        self.course.selected_stage = (self.course.selected_stage - 1) % Course.Stage.COUNT
        self.redraw()
