'''a very simple idiom for a state machine'''

from random import random
from time import sleep


# Each of the state functions below performs some action and then implements
# logic to choose next state.  Each state function returns the next state.

class Machine():
    def state0(self):
        print ("state0")
        # delay and decision path to simulate some application logic
        sleep(.5)
        if random()>.5:
            return self.state1
        else:
            return self.state2

    def state1(self):
        print ("state1")
        # delay and decision path to simulate some application logic
        sleep(.5)
        if random()>.5:
            return self.state0
        else:
            return self.state2

    def state2(self):
        print ("state2")
        # delay and decision path to simulate some application logic
        sleep(.5)
        if random()>.5:
            return self.state0
        else:
            return None

    def __init__(self): 
        state=self.state0()    # initial state
        while state: state=state()  # launch state machine
        print ("Done with states")

fsm = Machine()