"""
.. module:: state
   :platform: Unix, Windows
   :synopsis: A module to represent a state in the statemachine

.. moduleauthor:: Sebastian Brunner


"""

import threading
import sys
from gtkmvc import Observable
import Queue
from enum import Enum
import yaml

from utils import log
logger = log.get_logger(__name__)
from statemachine.outcome import Outcome
from statemachine.script import Script
from statemachine.execution.statemachine_status import StateMachineStatus
from statemachine.id_generator import *


class DataPort(Observable, yaml.YAMLObject):

    #TODO: should the value of the data port be stored here as well?
    """A class for representing a data ports in a state

    :ivar _name: the name of the data port
    :ivar _data_type: the value type of the port

    """
    def __init__(self, name=None, data_type=None):

        Observable.__init__(self)

        self._name = None
        self.name = name
        self._data_type = None
        self.data_type = data_type

    def __str__(self):
        return "DataPort: \n name: %s \n data_type: %s " % (self.name, self.data_type)

    yaml_tag = u'!DataPort'

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'name': data.name,
            'data_type': data.data_type
        }
        print dict_representation
        node = dumper.represent_mapping(u'!DataPort', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node)
        name = dict_representation['name']
        data_type = dict_representation['data_type']
        return DataPort(name, data_type)


#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @property
    def name(self):
        """Property for the _name field

        """
        return self._name

    @name.setter
    @Observable.observed
    def name(self, name):
        if not isinstance(name, str):
            raise TypeError("ID must be of type str")
        self._name = name

    @property
    def data_type(self):
        """Property for the _data_type field

        """
        return self._data_type

    @data_type.setter
    @Observable.observed
    def data_type(self, data_type):
        if not isinstance(data_type, str):
            raise TypeError("data_type must be of type str")
        if not data_type in ("int", "float", "bool", "str", "dict", "tuple", "list"):
            if not getattr(sys.modules[__name__], data_type):
                raise TypeError("" + data_type + " is not a valid python data type")
        self._data_type = data_type


StateType = Enum('STATE_TYPE', 'EXECUTION HIERARCHY BARRIER_CONCURRENCY PREEMPTION_CONCURRENCY')


class State(threading.Thread, Observable, yaml.YAMLObject):

    """A class for representing a state in the state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar _state_id: the id of the state
    :ivar _name: the name of the state
    :ivar _input_keys: holds the input data keys of the state
    :ivar _output_keys: holds the output data keys of the state
    :ivar _outcomes: holds the state outcomes, which are the connection points for transitions
    :ivar _is_start: indicates if this state is a start state of a hierarchy
    :ivar _is_final: indicates if this state is a end state of a hierarchy
    :ivar _sm_status: reference to the status of the state machine
    :ivar _state_status: holds the status of the state
    :ivar _script: a script file that holds the definitions of the custom state functions (entry, execute, exit)
    :ivar _input_data: the input data of the state
    :ivar _output_data: the output data of the state
    :ivar _preempted: a flag to show if the state was preempted from outside
    :ivar _concurrency_queue: a queue to signal a preemptive concurrency state, that the execution of the state
                                finished
    :ivar _final_outcome: the final outcome of a state, when it finished execution
    :ivar _state_type: the type of the container state (i.e. hierarchy, concurrency etc.)

    """

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 sm_status=None, path=None, filename=None):

        Observable.__init__(self)
        threading.Thread.__init__(self)

        self._name = name
        if state_id is None:
            self._state_id = state_id_generator()
        else:
            self._state_id = state_id

        self._input_data_ports = None
        self.input_data_ports = input_data_ports

        self._output_data_ports = None
        self.output_data_ports = output_data_ports

        self._outcomes = None
        self.outcomes = outcomes

        self._is_start = None
        self._is_final = None

        if sm_status is None:
            self._sm_status = StateMachineStatus()
        else:
            self._sm_status = sm_status

        self._state_status = None
        self.script = Script(path, filename)

        self._input_data = {}
        self._output_data = {}
        self._preempted = False
        self._concurrency_queue = None
        self._final_outcome = None
        self._state_type = None

        logger.debug("State with id %s initialized" % self._state_id)

    def add_input_key(self, name, data_type):
        """Add a new input data port to the state

        :param name: the name of the new input data port
        :param data_type: the type of the new input data port

        """
        self._input_data_ports[name] = DataPort(name, data_type)

    def remove_input_key(self, name):
        """Remove an input data port from the state

        :param name: the name or the output data port to remove

        """
        self._input_data_ports.pop(name, None)

    def add_output_key(self, name, data_type):
        """Add a new output data port to the state

        :param name: the name of the new output data port
        :param data_type: the type of the new output data port

        """
        self._output_data_ports[name] = DataPort(name, data_type)

    def remove_output_key(self, name):
        """Remove an output data port from the state

        :param name: the name of the output data port to remove

        """
        self._output_data_ports.pop(name, None)

    def add_outcome(self, name, outcome_id=None):
        """Add a new outcome to the state

        :param name: the name of the outcome to add
        :param outcome_id: the optional outcome_id of the new outcome

        """
        if outcome_id is None:
            outcome_id = generate_outcome_id()
        outcome = Outcome(outcome_id, name)
        self._outcomes[outcome_id] = outcome
        return outcome_id

    def remove_outcome(self, outcome_id):
        """Remove an outcome from the state

        :param outcome_id: the id of the outcome to remove

        """
        self._outcomes.pop(outcome_id, None)

    def run(self, *args, **kwargs):
        """Implementation of the abstract run() method of the :class:`threading.Thread`

        TODO: Should be filled with code, that should be executed for each state derivative
        """
        raise NotImplementedError("The State.run() function has to be implemented!")

    def check_input_data_type(self, input_data):
        """Check the input data types of the state

        """
        for key, value in self.input_data_ports.iteritems():
            #check for primitive data types
            if not str(type(input_data[key]).__name__) == value.data_type:
                #check for classes
                if not isinstance(input_data[key], getattr(sys.modules[__name__], value.data_type)):
                    raise TypeError("Input of execute function must be of type %s" % str(value.data_type))

    def check_output_data_type(self, output_data):
        """Check the output data types of the state

        """
        for key, value in self.output_data_ports.iteritems():
            #check for primitive data types
            if not str(type(output_data[key]).__name__) == value.data_type:
                #check for classes
                if not isinstance(output_data[key], getattr(sys.modules[__name__], value.data_type)):
                    raise TypeError("Input of execute function must be of type %s" % str(value.data_type))

    def __str__(self):
        return "Common state values:\nstate_id: %s\nname: %s" % (self.state_id, self.name)

#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @property
    def state_id(self):
        """Property for the _state_id field

        """
        return self._state_id

    @state_id.setter
    @Observable.observed
    def state_id(self, state_id):
        """Setter function for the _id property

        :param state_id: The new id of the state. The type of the id has to be a string
        :type state_id: str
        """
        if not isinstance(state_id, str):
            raise TypeError("ID must be of type str")
        if len(state_id) < 1:
            raise ValueError("ID must have at least one character")

        self._state_id = state_id

    @property
    def name(self):
        """Property for the _name field

        """
        return self._name

    @name.setter
    @Observable.observed
    def name(self, name):
        if not isinstance(name, str):
            raise TypeError("Name must be of type str")
        if len(name) < 1:
            raise ValueError("Name must have at least one character")

        self._name = name

    @property
    def input_data_ports(self):
        """Property for the _input_data_ports field

        """
        return self._input_data_ports

    @input_data_ports.setter
    @Observable.observed
    def input_data_ports(self, input_data_ports):
        if input_data_ports is None:
            self._input_data_ports = {}
        else:
            if not isinstance(input_data_ports, dict):
                raise TypeError("input_data_ports must be of type dict")
            for key, value in input_data_ports.iteritems():
                if not isinstance(value, DataPort):
                    raise TypeError("element of input_data_ports must be of type DataPort")
            self._input_data_ports = input_data_ports

    @property
    def output_data_ports(self):
        """Property for the _output_data_ports field

        """
        return self._output_data_ports

    @output_data_ports.setter
    @Observable.observed
    def output_data_ports(self, output_data_ports):
        if output_data_ports is None:
            self._output_data_ports = {}
        else:
            if not isinstance(output_data_ports, dict):
                raise TypeError("output_data_ports must be of type dict")
            for key, value in output_data_ports.iteritems():
                if not isinstance(value, DataPort):
                    raise TypeError("element of output_data_ports must be of type DataPort")
            self._output_data_ports = output_data_ports

    @property
    def outcomes(self):
        """Property for the _outcomes field

        """
        return self._outcomes

    @outcomes.setter
    @Observable.observed
    def outcomes(self, outcomes):
        if outcomes is None:
            self._outcomes = {}
            self.add_outcome("success", 0)
            self.add_outcome("aborted", 1)
            self.add_outcome("preempted", 2)
        else:
            if not isinstance(outcomes, dict):
                raise TypeError("outcomes must be of type dict")
            for key, value in outcomes.iteritems():
                if not isinstance(value, Outcome):
                    raise TypeError("element of outcomes must be of type Outcome")
            self._outcomes = outcomes

    @property
    def is_start(self):
        """Property for the _is_start field

        """
        return self._is_start

    @is_start.setter
    @Observable.observed
    def is_start(self, is_start):
        if not isinstance(is_start, bool):
            raise TypeError("is_start must be of type bool")
        self._is_start = is_start

    @property
    def is_final(self):
        """Property for the _is_final field

        """
        return self._is_final

    @is_final.setter
    @Observable.observed
    def is_final(self, is_final):
        if not isinstance(is_final, bool):
            raise TypeError("is_final must be of type bool")
        self._is_final = is_final

    @property
    def sm_status(self):
        """Property for the _sm_status field

        """
        return self._sm_status

    @sm_status.setter
    @Observable.observed
    def sm_status(self, sm_status):
        if not isinstance(sm_status, int):
            raise TypeError("sm_status must be of type int")
        self._sm_status = sm_status

    @property
    def state_status(self):
        """Property for the _state_status field

        """
        return self._state_status

    @state_status.setter
    @Observable.observed
    def state_status(self, state_status):
        if not isinstance(state_status, StateMachineStatus):
            raise TypeError("state_status must be of type StatemachineStatus")
        self._state_status = state_status

    @property
    def script(self):
        """Property for the _script field

        """
        return self._script

    @script.setter
    @Observable.observed
    def script(self, script):
        if not isinstance(script, Script):
            raise TypeError("script must be of type Script")
        self._script = script

    @property
    def input_data(self):
        """Property for the _input_data field

        """
        return self._input_data

    @input_data.setter
    @Observable.observed
    def input_data(self, input_data):
        if not isinstance(input_data, dict):
            raise TypeError("input_data must be of type dict")
        self._input_data = input_data

    @property
    def output_data(self):
        """Property for the _output_data field

        """
        return self._output_data

    @output_data.setter
    @Observable.observed
    def output_data(self, output_data):
        if not isinstance(output_data, dict):
            raise TypeError("output_data must be of type script")
        self._output_data = output_data

    @property
    def preempted(self):
        """Property for the _preempted field

        """
        return self._preempted

    @preempted.setter
    @Observable.observed
    def preempted(self, preempted):
        if not isinstance(preempted, bool):
            raise TypeError("preempted must be of type bool")
        self._preempted = preempted

    @property
    def concurrency_queue(self):
        """Property for the _concurrency_queue field

        """
        return self._concurrency_queue

    @concurrency_queue.setter
    @Observable.observed
    def concurrency_queue(self, concurrency_queue):
        if not isinstance(concurrency_queue, Queue.Queue):
            if not concurrency_queue is None:
                raise TypeError("concurrency_queue must be of type Queue or None")
            else:
                #concurrency_queue is None
                pass
        self._concurrency_queue = concurrency_queue

    @property
    def final_outcome(self):
        """Property for the _final_outcome field

        """
        return self._final_outcome

    @final_outcome .setter
    @Observable.observed
    def final_outcome(self, final_outcome):
        if not isinstance(final_outcome, Outcome):
            raise TypeError("final_outcome must be of type Outcome")
        self._final_outcome = final_outcome

    @property
    def state_type(self):
        """Property for the _state_type field

        """
        return self._state_type

    @state_type.setter
    @Observable.observed
    def state_type(self, state_type):
        if not state_type is None:
            if not isinstance(state_type, StateType):
                raise TypeError("state_type must be of type StateType")
        self._state_type = state_type