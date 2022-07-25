#from asyncio.tasks import _T1, _T2, _T3
from hmac import trans_36
from pickle import TRUE
import random
import math

from matplotlib.style import reload_library
import numpy
from constants import *
from roomba import Roomba


class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardState(State):
    def __init__(self):
        super().__init__("MoveForward")
        self.time = 0

    def check_transition(self, agent, state_machine):
        if  self.time > MOVE_FORWARD_TIME:
            state_machine.change_state(MoveInSpiralState())
        elif agent.get_bumper_state():
            state_machine.change_state(GoBackState())
        pass

    def execute(self, agent):
        self.time += SAMPLE_TIME
        agent.set_velocity(FORWARD_SPEED,0)
        pass


class MoveInSpiralState(State):
    def __init__(self):
        super().__init__("MoveInSpiral")
        self.time = 0
    
    def check_transition(self, agent, state_machine):
        if  self.time > MOVE_IN_SPIRAL_TIME:
            state_machine.change_state(MoveForwardState())
        elif agent.get_bumper_state():
            state_machine.change_state(GoBackState())
        pass

    def execute(self, agent):
        self.time += SAMPLE_TIME
        agent.set_velocity(FORWARD_SPEED,FORWARD_SPEED/(INITIAL_RADIUS_SPIRAL+ self.time*SPIRAL_FACTOR))
        pass


class GoBackState(State):
    def __init__(self):
        super().__init__("GoBack")
        self.time = 0

    def check_transition(self, agent, state_machine):
        if self.time > GO_BACK_TIME:
            state_machine.change_state(RotateState())
        pass

    def execute(self, agent):
        self.time += SAMPLE_TIME
        agent.set_velocity(BACKWARD_SPEED,0)
        pass


class RotateState(State):
    def __init__(self):
        super().__init__("Rotate")
        self.time = 0
        self.angle = numpy.random.random()
        self.T = (self.angle*math.pi)/ANGULAR_SPEED

    def check_transition(self, agent, state_machine):
        if self.time > self.T:
            state_machine.change_state(MoveForwardState())
        pass
    
    def execute(self, agent):
        self.time += SAMPLE_TIME
        agent.set_velocity(0,ANGULAR_SPEED)
        pass
