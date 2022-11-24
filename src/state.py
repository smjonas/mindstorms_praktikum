class State:
    def __init__(self, transitions, on_enter=None):
        self.transitions = transitions
        self.on_enter = on_enter

    def check_conditions(self):
        for t in self.transitions:
            transition = {"condition": t[0], "successor": t[1]}
            if transition.condition():
                return transition.successor
        return None


def s(*args):
    return State(args)
