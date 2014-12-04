"""
.. module:: state
   :platform: Unix, Windows
   :synopsis: A module to represent a state in the statemachine

.. moduleauthor:: Sebastian Brunner


"""

from support import log
logger = log.get_logger(__name__)

import threading
from gtkmvc import Observable

state_id_counter = 0


def generate_id():
    """
        TODO: replace this by a more sophisticated id generation routine
    """
    global state_id_counter
    state_id_counter = state_id_counter + 1
    return state_id_counter


class State(threading.Thread, Observable):

    """A class for representing a state in the statemachine

    It inherits from Observable to make a change of its attributes observable (for example for the MVC architecture).

    :ivar _id: the id of the state
    :ivar _name: the name of the state
    :ivar _input_keys: holds the input data keys of the state
    :ivar _output_keys: holds the output data keys of the state
    :ivar _outcomes: holds the state outcomes, which are the connection points for transitions
    :ivar _is_start: indicates if this state is a start state of a hierarchy
    :ivar _is_final: indicates if this state is a end state of a hierarchy
    :ivar _sm_status: reference to the status of the statemachine
    :ivar _state_status: holds the status of the state

    """

    def __init__(self):
        Observable.__init__(self)
        threading.Thread.__init__(self)

        self.id = "Test"

        self._id = generate_id()
        self._name = "Untitled"
        self._input_keys = None
        self._output_keys = None
        self._outcomes = None
        self._is_start = None
        self._is_final = None
        self._sm_status = None
        self._state_status = None

        logger.debug("State with id %s initialized" % self._id)

    def run(self):
        logger.debug("Starting state with id %s" % self._id)


    @property
    def id(self):
        return self._id

    @id.setter
    @Observable.observed
    def id(self, id):
        if not isinstance(id, basestring):
            raise TypeError("ID must be of type (base)string")
        if len(id) < 1:
            raise ValueError("ID must have at least one character")

        self._id = id

    @property
    def name(self):
        return self._name

    @name.setter
    @Observable.observed
    def name(self, name):
        if not isinstance(name, basestring):
            raise TypeError("Name must be of type (base)string")
        if len(name) < 1:
            raise ValueError("Name must have at least one character")

        self._name = name

    pass