from transitions import Machine
import enum
import time
import uglyConst


class UglyFSM(object):

    def is_entering_ige(self):
        if (self.height < uglyConst.DIST_IGE - uglyConst.DIST_IGE_HYST):
            return True
        else:
            return False

    def is_exiting_ige(self):
        if (self.height > uglyConst.DIST_IGE + uglyConst.DIST_IGE_HYST):
            return True
        else:
            return False

    def on_enter_SEEKING(self):
        while self.is_SEEKING():
            print('Hej')
            time.sleep(1)

    class States(enum.Enum):
        INITIALIZING = 0
        TAKINGOFF = 1
        SEEKING = 2
        OGECTL = 3
        IGECTL = 4
        LANDING = 5
        LANDED = 6

    # transition: trigger, source, destination
    transitions = [
        {'trigger': 'nextState', 'source': States.INITIALIZING, 'dest': States.TAKINGOFF},
        {'trigger': 'nextState', 'source': States.TAKINGOFF, 'dest': States.SEEKING},
        {'trigger': 'nextState', 'source': States.SEEKING, 'dest': States.OGECTL, 'conditions':'is_exiting_ige'},
        {'trigger': 'nextState', 'source': States.OGECTL, 'dest': States.IGECTL, 'conditions':'is_entering_ige'}
    ]

    def __init__(self):
        self.height = 0.1
        self.machine = Machine(model=self, states=self.States, transitions=self.transitions, initial=self.States.INITIALIZING)


if __name__ == '__main__':
    
    fsm = UglyFSM()
    print(fsm.state)
    fsm.nextState()
    print(fsm.state)
    fsm.nextState()
    print(fsm.state)
    fsm.nextState()
    print(fsm.state)
    fsm.nextState()
    fsm.nextState()

