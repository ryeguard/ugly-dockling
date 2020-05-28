from random import random
from time import sleep
from time import time

class FiniteStateMachine():

    def state0(self):
        print ("state0")
        sleep(random()) # some calculations...
        if random()>.5:
            return self.state1
        else:
            return self.state2

    def state1(self):
        print ("state1")
        sleep(random()) # some calculations...
        if random()>.5:
            return self.state0
        else:
            return self.state2

    def state2(self):
        print ("state2")
        sleep(random()) # some calculations...
        if random()>.5:
            return self.state0
        else:
            return None

    def __init__(self): 
        T = 1.0
        state=self.state0()     # initial state
        while state:            # state machine loop
            t0 = time()
            state = state() 
            duration = time() - t0
            if (duration < T):
                sleep(T - duration)

        print ("Done with states")

fsm = FiniteStateMachine()