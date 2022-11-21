class Transition:
    def __init__(self, condition, successor):
        self.condition = condition
        self.successor = successor


class State:
    def __init__(self, transitions, on_enter=None):
        self.transitions = transitions
        self.on_enter = on_enter

    def check_conditions(self):
        for transition in self.transitions:
            if transition.condition():
                return transition.successor
        return None


def t(*args):
    return Transition(args)


def s(*args):
    return State(args)
