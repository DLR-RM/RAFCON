"""
.. module:: state
   :platform: Unix, Windows
   :synopsis: A module to represent a state in the statemachine

.. moduleauthor:: Sebastian Brunner


"""


import threading


state_id_counter = 0


def generate_id():
    """
        TODO: replace this by a more sophisticated id generation routine
    """
    global state_id_counter
    state_id_counter = state_id_counter + 1
    return state_id_counter


class State(threading.Thread):

    """
        A class for representing a state in the statemachine

        :ivar _id: the id of the state:
        :ivar _name: the name of the state:
        :ivar _input_keys: holds the input data keys of the state:
        :ivar _output_keys: holds the output data keys of the state:
        :ivar _outcomes: holds the state outcomes, which are the connection points for transitions:
        :ivar _is_start: indicates if this state is a start state of a hierarchy:
        :ivar _is_final: indicates if this state is a end state of a hierarchy:
        :ivar _sm_status: reference to the status of the statemachine:
        :ivar _state_status: holds the status of the state:

    """

    def __init__(self):

        threading.Thread.__init__(self)

        self.id = "Test"

        self._id = generate_id()
        self._name = None
        self._input_keys = None
        self._output_keys = None
        self._outcomes = None
        self._is_start = None
        self._is_final = None
        self._sm_status = None
        self._state_status = None

        print "State with id %s initialized" % self._id

    def run(self):
        print "Starting state with id %s" % self._id