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
from statemachine.script import Script, ScriptType
from statemachine.execution.statemachine_status import StateMachineStatus
from statemachine.id_generator import *


class DataPort(Observable, yaml.YAMLObject):

    #TODO: should the value of the data port be stored here as well?
    """A class for representing a data ports in a state

    :ivar _name: the name of the data port
    :ivar _data_type: the value type of the port

    """
    def __init__(self, name=None, data_type=None, default_value=None, data_port_id=None):

        Observable.__init__(self)

        if data_port_id is None:
            self._data_port_id = generate_data_port_id()
        else:
            self._data_port_id = data_port_id

        self._name = None
        self.name = name
        self._data_type = None
        self.data_type = data_type
        self._default_value = None
        self.default_value = default_value

        logger.debug("DataPort with name %s initialized" % self.name)

    def __str__(self):
        return "DataPort: \n name: %s \n data_type: %s \n default_value: %s " % (self.name, self.data_type,
                                                                                 self.default_value)

    yaml_tag = u'!DataPort'

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'data_port_id' : data.data_port_id,
            'name': data.name,
            'data_type': data.data_type,
            'default_value': data.default_value
        }
        print dict_representation
        node = dumper.represent_mapping(u'!DataPort', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node)
        data_port_id = dict_representation['data_port_id']
        name = dict_representation['name']
        data_type = dict_representation['data_type']
        default_value = dict_representation['default_value']
        return DataPort(name, data_type, default_value, data_port_id)

#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @property
    def data_port_id(self):
        """Property for the _data_port_id field

        """
        return self._data_port_id

    @property
    def name(self):
        """Property for the _name field

        """
        return self._name

    @name.setter
    @Observable.observed
    def name(self, name):
        if not isinstance(name, str):
            raise TypeError("name must be of type str")
        self._name = name

    @property
    def data_type(self):
        """Property for the _data_type field

        """
        return self._data_type

    @data_type.setter
    @Observable.observed
    def data_type(self, data_type):
        if not data_type is None:
            if not isinstance(data_type, str):
                raise TypeError("data_type must be of type str")
            if not data_type in ("int", "float", "bool", "str", "dict", "tuple", "list"):
                if not getattr(sys.modules[__name__], data_type):
                    raise TypeError("" + data_type + " is not a valid python data type")
        self._data_type = data_type

    @property
    def default_value(self):
        """Property for the _default_value field

        """
        return self._default_value

    @default_value.setter
    @Observable.observed
    def default_value(self, default_value):
        if not default_value is None:
            #check for primitive data types
            if not str(type(default_value).__name__) == self.data_type:
                #check for classes
                print type(default_value).__name__
                if not isinstance(default_value, getattr(sys.modules[__name__], self.data_type)):
                    raise TypeError("Input of execute function must be of type %s" % str(self.data_type))
        self._default_value = default_value


StateType = Enum('STATE_TYPE', 'EXECUTION HIERARCHY BARRIER_CONCURRENCY PREEMPTION_CONCURRENCY LIBRARY')
DataPortType = Enum('DATA_PORT_TYPE', 'INPUT OUTPUT SCOPED')


class State(threading.Thread, Observable, yaml.YAMLObject, object):

    """A class for representing a state in the state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar _state_id: the id of the state
    :ivar _name: the name of the state
    :ivar _input_data_ports: holds the input data ports of the state
    :ivar _output_data_ports: holds the output data ports of the state
    :ivar _outcomes: holds the state outcomes, which are the connection points for transitions
    :ivar _is_start: indicates if this state is a start state of a hierarchy
    :ivar _is_final: indicates if this state is a end state of a hierarchy
    :ivar _sm_status: reference to the status of the state machine
    :ivar _state_status: holds the status of the state during runtime
    :ivar _script: a script file that holds the definitions of the custom state functions (entry, execute, exit)
    :ivar _input_data: the input data of the state during execution
    :ivar _output_data: the output data of the state during execution
    :ivar _preempted: a flag to show if the state was preempted from outside
    :ivar _concurrency_queue: a queue to signal a preemptive concurrency state, that the execution of the state
                                finished
    :ivar _final_outcome: the final outcome of a state, when it finished execution
    :ivar _state_type: the type of the container state (i.e. hierarchy, concurrency etc.)

    """

    #input_data_ports = []
    #__observables__ = ("input_data_ports", )

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 sm_status=None, path=None, filename=None, state_type=None, parent=None):

        Observable.__init__(self)
        threading.Thread.__init__(self)

        self._name = None
        self.name = name
        if state_id is None:
            self._state_id = state_id_generator()
        else:
            self._state_id = state_id

        self._parent = None
        self.parent = parent

        self._state_type = None
        self.state_type = state_type

        self._input_data_ports = None
        self.input_data_ports = input_data_ports

        self._output_data_ports = None
        self.output_data_ports = output_data_ports

        self._used_outcome_ids = []
        self._outcomes = None
        self.outcomes = outcomes

        self._is_start = None
        self._is_final = None

        if sm_status is None:
            self._sm_status = StateMachineStatus()
        else:
            self._sm_status = sm_status

        self._state_status = None
        if state_type is StateType.EXECUTION:
            self.script = Script(path, filename, script_type=ScriptType.EXECUTION)
        else:
            self.script = Script(path, filename, script_type=ScriptType.CONTAINER)

        self._input_data = {}
        self._output_data = {}
        self._preempted = False
        self._concurrency_queue = None
        self._final_outcome = None
        self._description = None

        self._active = None

        logger.debug("State with id %s initialized" % self._state_id)

    @Observable.observed
    def add_input_data_port(self, name, data_type=None, default_value=None, data_port_id=None):
        """Add a new input data port to the state

        :param name: the name of the new input data port
        :param data_type: the type of the new input data port
        :param default_value: the default value of the data port

        """
        if data_port_id is None:
            data_port_id = generate_data_flow_id()
        self._input_data_ports[data_port_id] = DataPort(name, data_type, default_value, data_port_id)
        return data_port_id

    @Observable.observed
    def remove_input_data_port(self, data_port_id):
        """Remove an input data port from the state

        :param data_port_id: the id or the output data port to remove

        """
        if data_port_id in self._input_data_ports:
            self.remove_data_flows_with_data_port_id(data_port_id)
            del self._input_data_ports[data_port_id]
        else:
            raise AttributeError("input data port with name %s does not exit", data_port_id)

    def remove_data_flows_with_data_port_id(self, data_port_id):
        """Remove an data ports whose from_key or to_key equals the passed data_port_id

        :param data_port_id: the id of a data_port of which all data_flows should be removed, the id can be a input or
                            output data port id

        """
        if not self.parent is None:
            # delete all data flows in parent related to data_port_id and self.state_id
            data_flow_ids_to_remove = []
            for data_flow_id, data_flow in self.parent.data_flows.iteritems():
                if data_flow.from_state == self.state_id and data_flow.from_key == data_port_id or \
                        data_flow.to_state == self.state_id and data_flow.to_key == data_port_id:
                    data_flow_ids_to_remove.append(data_flow_id)

            for data_flow_id in data_flow_ids_to_remove:
                self.parent.remove_data_flow(data_flow_id)
                # del self.parent.data_flows[data_flow_id]

    @Observable.observed
    def add_output_data_port(self, name, data_type, default_value=None, data_port_id=None):
        """Add a new output data port to the state

        :param name: the name of the new output data port
        :param data_type: the type of the new output data port
        :param default_value: the default value of the data port

        """
        if data_port_id is None:
            data_port_id = generate_data_flow_id()
        self._output_data_ports[data_port_id] = DataPort(name, data_type, default_value, data_port_id)
        return data_port_id

    @Observable.observed
    def remove_output_data_port(self, data_port_id):
        """Remove an output data port from the state

        :param data_port_id: the id of the output data port to remove

        """
        if data_port_id in self._output_data_ports:
            self.remove_data_flows_with_data_port_id(data_port_id)
            del self._output_data_ports[data_port_id]
        else:
            raise AttributeError("output data port with name %s does not exit", data_port_id)

    def get_io_data_port_id_from_name_and_type(self, name, data_port_type):
        """Returns the data_port_id of a data_port with a certain name and data port type

        :param name: the name of the target data_port
        :param data_port_type: the data port type of the target data port

        """
        if data_port_type is DataPortType.INPUT:
            for ip_id, output_port in self.input_data_ports.iteritems():
                if output_port.name == name:
                    #print ip_id
                    return ip_id
            raise AttributeError("Name %s is not in input_data_ports", name)
        elif data_port_type is DataPortType.OUTPUT:
            for op_id, output_port in self.output_data_ports.iteritems():
                if output_port.name == name:
                    return op_id
            raise AttributeError("Name %s is not in output_data_ports", name)

    def get_data_port_by_id(self, id):
        """ Returns the io-data_port or scoped_variable with a certain id
        :param port_id:
        :return:
        """
        if id in self.input_data_ports:
            return self.input_data_ports[id]
        elif id in self.output_data_ports:
            return self.output_data_ports[id]
        else:
            raise AttributeError("Data_Port_id %s is not in input_data_ports or output_data_ports", id)

    @Observable.observed
    def add_outcome(self, name, outcome_id=None):
        """Add a new outcome to the state

        :param name: the name of the outcome to add
        :param outcome_id: the optional outcome_id of the new outcome

        """
        if outcome_id is None:
            outcome_id = generate_outcome_id()
        if name in self._outcomes:
            logger.error("Two outcomes cannot have the same names")
            return
        if outcome_id in self._used_outcome_ids:
            logger.error("Two outcomes cannot have the same outcome_ids")
            return
        outcome = Outcome(outcome_id, name, self.modify_outcome_name)
        self._outcomes[outcome_id] = outcome
        self._used_outcome_ids.append(outcome_id)
        return outcome_id

    @Observable.observed
    def remove_outcome(self, outcome_id):
        """Remove an outcome from the state

        :param outcome_id: the id of the outcome to remove

        """
        if not outcome_id in self._used_outcome_ids:
            raise AttributeError("There is no outcome_id %s" % str(outcome_id))

        if outcome_id == -1 or outcome_id == -2:
            raise AttributeError("You cannot remove the outcomes with id -1 or -2 as a state must always be able to"
                                 "return aborted or preempted")

        # delete all transitions connected to this outcome
        if not self.parent is None:
            transition_ids_to_remove = []
            for transition_id, transition in self.parent.transitions.iteritems():
                if transition.from_outcome == outcome_id:
                    transition_ids_to_remove.append(transition_id)

            for transition_id in transition_ids_to_remove:
                self.parent.remove_transition(transition_id)
                # del self.parent.transitions[transition_id]

        # delete outcome it self
        self._used_outcome_ids.remove(outcome_id)
        self._outcomes.pop(outcome_id, None)

    @Observable.observed
    def modify_outcome_name(self, name, outcome_id):

        def define_unique_name(name, dict_of_names, count=0):
            count += 1
            if name + str(count) in dict_of_names.values():
                count = define_unique_name(name, dict_of_names, count)
            return count

        dict_of_names = {}
        for o_id, outcome in self._outcomes.items():
            dict_of_names[o_id] = outcome.name
        #print dict_of_names

        if outcome_id in self._outcomes.keys() and self._outcomes[outcome_id].name == name:
            name = self._outcomes[outcome_id].name
        elif name in dict_of_names.values():
            name += str(define_unique_name(name, dict_of_names))
        return name

    def run(self, *args, **kwargs):
        """Implementation of the abstract run() method of the :class:`threading.Thread`

        TODO: Should be filled with code, that should be executed for each state derivative
        """
        raise NotImplementedError("The State.run() function has to be implemented!")

    def check_input_data_type(self, input_data):
        """Check the input data types of the state

        """
        # print input_data
        # for key, idp in self.input_data_ports.iteritems():
        #     print key
        #     print idp

        for input_data_port_key, data_port in self.input_data_ports.iteritems():
            if not input_data[data_port.name] is None:
                #check for primitive data types
                if not str(type(input_data[data_port.name]).__name__) == data_port.data_type:
                    #check for classes
                    if not isinstance(input_data[data_port.name], getattr(sys.modules[__name__], data_port.data_type)):
                        raise TypeError("Input of execute function must be of type %s" % str(data_port.data_type))

    def check_output_data_type(self, output_data):
        """Check the output data types of the state

        """
        for output_port_id, output_port in self.output_data_ports.iteritems():
            if not output_data[output_port.name] is None:
                #check for primitive data types
                if not str(type(output_data[output_port.name]).__name__) == output_port.data_type:
                    #check for classes
                    if not isinstance(output_data[output_port.name], getattr(sys.modules[__name__], output_port.data_type)):
                        raise TypeError("Input of execute function must be of type %s" % str(output_port.data_type))

    def __str__(self):
        return "State properties of state: %s \nstate_id: %s" % (self.name, self.state_id)

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
        if not name is None:
            if not isinstance(name, str):
                raise TypeError("Name must be of type str")
            if len(name) < 1:
                raise ValueError("Name must have at least one character")
        self._name = name

    @property
    def parent(self):
        """Property for the _parent field

        """
        return self._parent

    @parent.setter
    @Observable.observed
    def parent(self, parent):
        if not parent is None:
            if not isinstance(parent, State):
                raise TypeError("parent must be of type State")

        self._parent = parent

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
                if not key == value.data_port_id:
                    raise AttributeError("the key of the input dictionary and the name of the data port do not match")
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
            self.add_outcome("aborted", -1)
            self.add_outcome("preempted", -2)
            if self.state_type is StateType.BARRIER_CONCURRENCY:
                #for a barrier concurrency case, there is only one successfull outcome
                self.add_outcome("success", 0)

        else:
            if not isinstance(outcomes, dict):
                raise TypeError("outcomes must be of type dict")
            for key, value in outcomes.iteritems():
                if not isinstance(value, Outcome):
                    raise TypeError("element of outcomes must be of type Outcome")
            self._outcomes = outcomes
            if self.state_type is StateType.BARRIER_CONCURRENCY:
                if not "success" in outcomes:
                    self.add_outcome("success", 0)
            #aborted and preempted must always exist
            if not -1 in outcomes:
                self.add_outcome("aborted", -1)
            if not -2 in outcomes:
                self.add_outcome("preempted", -2)


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
        #if hasattr(self, '_script'):
        #    print "change script of state.name: %s from script: \n %s" % (self.name, self._script.script)
        self._script = script
        #print "change script of state: %s state.name: %s to script: \n %s" % (self.state_id, self.name, self._script.script)

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

    @property
    def description(self):
        """Property for the _description field

        """
        return self._description

    @description.setter
    @Observable.observed
    def description(self, description):
        if not isinstance(description, str):
            if not isinstance(description, unicode):
                raise TypeError("ID must be of type str or unicode")
        if len(description) < 1:
            raise ValueError("ID must have at least one character")

        self._description = description

    @property
    def active(self):
        """Property for the _active field

        """
        return self._active

    @active.setter
    @Observable.observed
    def active(self, active):
        if not isinstance(active, bool):
            raise TypeError("active must be of type bool")

        self._active = active
