class State:
    def __init__(self, transitions, on_enter=None):
        self.transitions = transitions
        self.on_enter = on_enter

    def check_conditions(self):
        for t in self.transitions:
            condition, successor = t
            if condition is True or condition():
                return successor
        return None
