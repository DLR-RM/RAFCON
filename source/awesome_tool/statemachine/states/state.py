"""
.. module:: state
   :platform: Unix, Windows
   :synopsis: A module to represent a state in the statemachine

.. moduleauthor:: Sebastian Brunner


"""

import threading
import sys
import Queue
import copy

from gtkmvc import Observable
import yaml

from awesome_tool.utils import log
logger = log.get_logger(__name__)

from awesome_tool.statemachine.data_port import DataPort
from awesome_tool.statemachine.enums import DataPortType, StateExecutionState
from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.script import Script, ScriptType
from awesome_tool.statemachine.id_generator import *


PATH_SEPARATOR = '/'


class State(Observable, yaml.YAMLObject, object):

    """A class for representing a state in the state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar state_id: the id of the state
    :ivar name: the name of the state
    :ivar parent: the parent of the state
    :ivar input_data_ports: holds the input data ports of the state
    :ivar output_data_ports: holds the output data ports of the state
    :ivar outcomes: holds the state outcomes, which are the connection points for transitions
    :ivar parent: a reference to the parent state

    """

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 parent=None):

        Observable.__init__(self)
        self.thread = None

        if name is None:
            name = "Untitled"
        self._name = None
        self.name = name
        if state_id is None:
            self._state_id = state_id_generator()
        else:
            self._state_id = state_id

        self._parent = None
        self.parent = parent

        self._used_data_port_ids = set([])
        self._input_data_ports = None
        self.input_data_ports = input_data_ports

        self._output_data_ports = None
        self.output_data_ports = output_data_ports

        self._used_outcome_ids = []
        self._outcomes = None
        self.outcomes = outcomes

        self._script = None

        # the input data of the state during execution
        self._input_data = {}
        # the output data of the state during execution
        self._output_data = {}
        # a flag to show if the state was preempted from outside
        self._preempted = threading.Event()
        # a queue to signal a preemptive concurrency state, that the execution of the state finished
        self._concurrency_queue = None
        # the final outcome of a state, when it finished execution
        self._final_outcome = None
        self._description = None
        # detailed execution status of the state
        self._state_execution_status = None
        self.state_execution_status = StateExecutionState.INACTIVE

        self.edited_since_last_execution = False
        self.execution_history = None
        self.backward_execution = False

        logger.debug("State with id %s and name %s initialized" % (self._state_id, self.name))

    # ---------------------------------------------------------------------------------------------
    # ----------------------------------- execution functions -------------------------------------
    # ---------------------------------------------------------------------------------------------

    # give the state the appearance of a thread that can be started several times
    def start(self, execution_history, backward_execution=False):
        """ Starts the execution of the state in a new thread.

        :return:
        """
        self.execution_history = execution_history
        self.backward_execution = copy.copy(backward_execution)
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def join(self):
        """ Waits until the state finished execution.

        """
        if self.thread:
            self.thread.join()
        else:
            logger.debug("State %s was not started yet, cannot join" % self.name)

    def setup_run(self):
        """ Executes a generic set of actions that has to be called in the run methods of each derived state class.

        :return:
        """
        self.state_execution_status = StateExecutionState.ACTIVE
        self.preempted = False
        if not isinstance(self.input_data, dict):
            raise TypeError("states must be of type dict")
        if not isinstance(self.output_data, dict):
            raise TypeError("states must be of type dict")
        self.check_input_data_type(self.input_data)

    def setup_backward_run(self):
        self.state_execution_status = StateExecutionState.ACTIVE
        self.preempted = False

    def run(self, *args, **kwargs):
        """Implementation of the abstract run() method of the :class:`threading.Thread`

        TODO: Should be filled with code, that should be executed for each state derivative
        """
        raise NotImplementedError("The State.run() function has to be implemented!")

    def recursively_preempt_states(self):
        """ Preempt the state
        """
        self.preempted = True

    def get_previously_executed_state(self):
        """
        Calculates the state that was executed before this state
        :return: The last state in the execution history
        """
        return self.execution_history.get_last_history_item().prev.state_reference

    # ---------------------------------------------------------------------------------------------
    # ------------------------------- input/output data handling ----------------------------------
    # ---------------------------------------------------------------------------------------------

    def get_default_input_values_for_state(self, state):
        """ Computes the default input values for a state

        :param state: the state to get the default input values for

        """
        result_dict = {}
        for input_port_key, value in state.input_data_ports.iteritems():
            default = value.default_value
            # if the user sets the default value to a string starting with $, try to retrieve the value
            # from the global variable manager
            if isinstance(default, str) and default[0] == '$':
                from awesome_tool.statemachine.singleton import global_variable_manager as gvm
                var_name = default[1:]
                if not gvm.variable_exist(var_name):
                    logger.error("The global variable '{0}' does not exist".format(var_name))
                    global_value = None
                else:
                    global_value = gvm.get_variable(var_name)
                result_dict[value.name] = global_value
            # set input to its default value
            result_dict[value.name] = copy.copy(default)
        return result_dict

    def create_output_dictionary_for_state(self, state):
        """Return empty output dictionary for a state

        :param state: the state of which the output data is determined
        :return: the output data of the target state
        """
        result_dict = {}
        for key, data_port in state.output_data_ports.iteritems():
            result_dict[data_port.name] = None
        return result_dict
    # ---------------------------------------------------------------------------------------------
    # ----------------------------------- data port functions -------------------------------------
    # ---------------------------------------------------------------------------------------------

    @Observable.observed
    def add_input_data_port(self, name, data_type=None, default_value=None, data_port_id=None):
        """Add a new input data port to the state

        :param name: the name of the new input data port
        :param data_type: the type of the new input data port
        :param default_value: the default value of the data port

        """
        if data_port_id is None or data_port_id in self._used_data_port_ids:
            if data_port_id in self._used_data_port_ids:
                logger.warning("handed data_port_id is already in list of _used_data_port_ids id: %s list: %s" %
                               (data_port_id, self._used_data_port_ids))
            data_port_id = generate_data_flow_id()
            while data_port_id in self._used_data_port_ids:
                data_port_id = generate_data_flow_id()
        self._used_data_port_ids.add(data_port_id)
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
            self._used_data_port_ids.remove(data_port_id)
        else:
            raise AttributeError("input data port with name %s does not exit", data_port_id)

    def remove_data_flows_with_data_port_id(self, data_port_id):
        """Remove all data flows whose from_key or to_key equals the passed data_port_id

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

        if data_port_id is None or data_port_id in self._used_data_port_ids:
            if data_port_id in self._used_data_port_ids:
                logger.warning("handed data_port_id is already in list of _used_data_port_ids id: %s list: %s" %
                               (data_port_id, self._used_data_port_ids))
                logger.warning("%s %s %s" % (self._input_data_ports.keys(),
                                             self._output_data_ports.keys(),
                                             self._scoped_variables.keys()))
            data_port_id = generate_data_flow_id()
            while data_port_id in self._used_data_port_ids:
                data_port_id = generate_data_flow_id()
        self._used_data_port_ids.add(data_port_id)
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
            self._used_data_port_ids.remove(data_port_id)
        else:
            raise AttributeError("output data port with name %s does not exit", data_port_id)

    def get_io_data_port_id_from_name_and_type(self, name, data_port_type):
        """Returns the data_port_id of a data_port with a certain name and data port type

        :param name: the name of the target data_port
        :param data_port_type: the data port type of the target data port
        :return: the data port specified by the name and the type
        """
        if data_port_type is DataPortType.INPUT:
            for ip_id, output_port in self.input_data_ports.iteritems():
                if output_port.name == name:
                    return ip_id
            raise AttributeError("Name %s is not in input_data_ports", name)
        elif data_port_type is DataPortType.OUTPUT:
            for op_id, output_port in self.output_data_ports.iteritems():
                if output_port.name == name:
                    return op_id
            raise AttributeError("Name %s is not in output_data_ports", name)

    def get_data_port_by_id(self, id):
        """ Returns the io-data_port or scoped_variable with a certain id
        :param id: the id of the target data port
        :return: the data port specified by the id
        """
        if id in self.input_data_ports:
            return self.input_data_ports[id]
        elif id in self.output_data_ports:
            return self.output_data_ports[id]
        else:
            raise AttributeError("Data_Port_id %s is not in input_data_ports or output_data_ports", id)

    # ---------------------------------------------------------------------------------------------
    # ------------------------------------ outcome functions --------------------------------------
    # ---------------------------------------------------------------------------------------------

    def get_path(self, appendix=None):
        """ Recursively create the path of the state. In bottom up method i.e. from the nested child states to the root
        state.
        :param appendix: the part of the path that was already calculated by previous function calls
        :return: the full path to the root state
        """
        if self.parent:
            if appendix is None:
                return self.parent.get_path(self.state_id)
            else:
                return self.parent.get_path(self.state_id + PATH_SEPARATOR + appendix)
        else:
            if appendix is None:
                return self.state_id
            else:
                return self.state_id + PATH_SEPARATOR + appendix

    @Observable.observed
    def add_outcome(self, name, outcome_id=None):
        """Add a new outcome to the state

        :param name: the name of the outcome to add
        :param outcome_id: the optional outcome_id of the new outcome

        :return: outcome_id: the outcome if of the generated state

        """
        if outcome_id is None:
            outcome_id = generate_outcome_id(self._used_outcome_ids)
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
        if outcome_id not in self._used_outcome_ids:
            raise AttributeError("There is no outcome_id %s" % str(outcome_id))

        if outcome_id == -1 or outcome_id == -2:
            raise AttributeError("You cannot remove the outcomes with id -1 or -2 as a state must always be able to"
                                 "return aborted or preempted")

        self.remove_outcome_hook(outcome_id)

        # delete possible transition connected to this outcome
        if self.parent is not None:
            for transition_id, transition in self.parent.transitions.iteritems():
                if transition.from_outcome == outcome_id and transition.from_state == self.state_id:
                    self.parent.remove_transition(transition_id)
                    break  # found the one outgoing transition

        # delete outcome it self
        self._used_outcome_ids.remove(outcome_id)
        self._outcomes.pop(outcome_id, None)

    def remove_outcome_hook(self, outcome_id):
        """Hook for adding more logic when removing an outcome

        This hook is intended for the use of inherited classed, which can add more functionality if needed. A
        container state would remove its transitions going the removed outcome here.

        :param outcome_id: The id of the outcome that is removed
        """
        pass

    def is_valid_outcome_id(self, outcome_id):
        """Checks if outcome_id valid type and points to element of state.

        :param int outcome_id:
        :return:
        """
        #check if types are valid
        if not isinstance(outcome_id, int):
            raise TypeError("outcome_id must be of type int")
        # consistency check
        if outcome_id not in self.outcomes:
            raise AttributeError("outcome_id %s has to be in container_state %s outcomes-list" %
                                 (outcome_id, self.state_id))

    def modify_outcome_name(self, name, outcome):
        """Checks if the outcome name already exists. If this is the case a unique number is appended to the name

        :param name: the desired name of a possibly new outcome
        :return: name: a unique outcome name for the state
        """
        def define_unique_name(name, dict_of_names, count=0):
            count += 1
            if name + str(count) in dict_of_names.values():
                count = define_unique_name(name, dict_of_names, count)
            return count

        dict_of_names = {}
        for o_id, o in self._outcomes.items():
            dict_of_names[o_id] = o.name

        if name in dict_of_names.values() and not outcome.name == name:
            name += str(define_unique_name(name, dict_of_names))
        return name

    def connect_all_outcome_function_handles(self):
        """In case of the outcomes were created by loading from a yaml file, the function handlers are not set.
            This method allows to set the handlers for all outcomes.
        """
        for outcome_id, outcome in self.outcomes.iteritems():
            outcome.check_name = self.modify_outcome_name

    # ---------------------------------------------------------------------------------------------
    # -------------------------------------- misc functions ---------------------------------------
    # ---------------------------------------------------------------------------------------------

    def check_input_data_type(self, input_data):
        """Check the input data types of the state

        :param input_data: the input_data dictionary to check
        """
        for input_data_port_key, data_port in self.input_data_ports.iteritems():
            if input_data_port_key in input_data:
                if not input_data[data_port.name] is None:
                    #check for primitive data types
                    if not str(type(input_data[data_port.name]).__name__) == data_port.data_type:
                        #check for classes
                        if not isinstance(input_data[data_port.name], getattr(sys.modules[__name__], data_port.data_type)):
                            raise TypeError("Input of execute function must be of type %s" % str(data_port.data_type))

    def check_output_data_type(self):
        """Check the output data types of the state

        """
        for output_port_id, output_port in self.output_data_ports.iteritems():
            if hasattr(self.output_data, output_port.name) and self.output_data[output_port.name] is not None:
                #check for primitive data types
                if not str(type(self.output_data[output_port.name]).__name__) == output_port.data_type:
                    #check for classes
                    if not isinstance(self.output_data[output_port.name], getattr(sys.modules[__name__], output_port.data_type)):
                        raise TypeError("Input of execute function must be of type %s" % str(output_port.data_type))

    def set_script_text(self, new_text):
        """
        Sets the text of the script. This function can be overridden to prevent setting the script under certain
        circumstances.
        :param new_text: The new text to replace to old text with.
        :return: Returns True if the script was successfully set.
        """
        self.script.script = new_text
        return True

    def change_state_id(self, state_id=None):
        """
        Changes the id of the state to a new id. If now state_id is passed as parameter, a new state id is generated.
        :param state_id: The new state if of the state
        :return:
        """
        new_state_id = None
        if state_id is None:
            new_state_id = state_id_generator()
        else:
            new_state_id = state_id
        if self.parent is not None:
            while self.parent.state_id_exists(new_state_id):
                new_state_id = state_id_generator()

        self._state_id = new_state_id

    def __str__(self):
        return "State '{0}' with ID '{1}' and and type {2}".format(self.name, self.state_id, type(self))

#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @property
    def state_id(self):
        """Property for the _state_id field

        """
        return self._state_id

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
            if self._input_data_ports is not None:
                for input_port_id in self._input_data_ports.keys():
                    self._used_data_port_ids.remove(input_port_id)
            self._input_data_ports = {}
        else:
            if not isinstance(input_data_ports, dict):
                raise TypeError("input_data_ports must be of type dict")
            for key, value in input_data_ports.iteritems():
                if not isinstance(value, DataPort):
                    raise TypeError("element of input_data_ports must be of type DataPort")
                if not key == value.data_port_id:
                    raise AttributeError("the key of the input dictionary and the name of the data port do not match")
                if value.data_port_id in self._used_data_port_ids:
                    raise AttributeError("data_port_id %s already exists" % (str(value.data_port_id)))
                self._used_data_port_ids.add(value.data_port_id)
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
            if self._output_data_ports is not None:
                for output_port_id in self._output_data_ports.keys():
                    self._used_data_port_ids.remove(output_port_id)
            self._output_data_ports = {}
        else:
            if not isinstance(output_data_ports, dict):
                raise TypeError("output_data_ports must be of type dict")
            for key, value in output_data_ports.iteritems():
                if not isinstance(value, DataPort):
                    raise TypeError("element of output_data_ports must be of type DataPort")
                if not key == value.data_port_id:
                    raise AttributeError("the key of the output dictionary and the name of the data port do not match")
                if value.data_port_id in self._used_data_port_ids:
                    raise AttributeError("data_port_id %s already exists" % (str(value.data_port_id)))
                self._used_data_port_ids.add(value.data_port_id)
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
            if self.outcomes is not None:
                for id in self.outcomes.keys():
                    self._used_outcome_ids.remove(id)
            self._outcomes = {}
            self.add_outcome("success", 0)
            self.add_outcome("aborted", -1)
            self.add_outcome("preempted", -2)

        else:
            if not isinstance(outcomes, dict):
                raise TypeError("outcomes must be of type dict")
            for key, value in outcomes.iteritems():
                if not isinstance(value, Outcome):
                    raise TypeError("element of outcomes must be of type Outcome")
            self._outcomes = outcomes
            # aborted and preempted must always exist
            if -1 not in outcomes:
                self.add_outcome("aborted", -1)
            if -2 not in outcomes:
                self.add_outcome("preempted", -2)
            for id, o in outcomes.iteritems():
                self._used_outcome_ids.append(id)

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
    #@Observable.observed
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
    #@Observable.observed
    def output_data(self, output_data):
        if not isinstance(output_data, dict):
            raise TypeError("output_data must be of type dict")
        self._output_data = output_data

    @property
    def preempted(self):
        """Property for the _preempted field

        """
        return self._preempted.is_set()

    @preempted.setter
    #@Observable.observed
    def preempted(self, preempted):
        if not isinstance(preempted, bool):
            raise TypeError("preempted must be of type bool")
        if preempted:
            self._preempted.set()
        else:
            self._preempted.clear()

    @property
    def concurrency_queue(self):
        """Property for the _concurrency_queue field

        """
        return self._concurrency_queue

    @concurrency_queue.setter
    #@Observable.observed
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
    #@Observable.observed
    def final_outcome(self, final_outcome):
        if not isinstance(final_outcome, Outcome):
            raise TypeError("final_outcome must be of type Outcome")
        self._final_outcome = final_outcome

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
                raise TypeError("Description must be of type str or unicode")
        if len(description) < 1:
            raise ValueError("Description must have at least one character")

        self._description = description

    @property
    def active(self):
        """Property for the _active field

        """
        if self.state_execution_status is StateExecutionState.INACTIVE:
            return False
        else:
            return True

    # @active.setter
    # @Observable.observed
    # def active(self, active):
    #     if not isinstance(active, bool):
    #         raise TypeError("active must be of type bool")
    #
    #     self._active = active

    @property
    def state_execution_status(self):
        """Property for the _state_execution_status field

        """
        return self._state_execution_status

    @state_execution_status.setter
    @Observable.observed
    def state_execution_status(self, state_execution_status):
        if not isinstance(state_execution_status, StateExecutionState):
            raise TypeError("state_execution_status must be of type StateExecutionState")

        self._state_execution_status = state_execution_status

    def preemptive_wait(self, time=None):
        """Waiting method which can be preempted

        Use this method if you want a state to pause. In contrast to time.sleep(), the pause can be preempted. This
        method can also be used if you want to have a daemon thread within a preemptive concurrency state. In this
        case, time has to be set to None and the method waits indefinitely or until it is preempted from outside.
        :param time: The time in seconds to wait or None (default) for infinity
        :return: True, if the wait was preempted, False else
        """
        return self._preempted.wait(time)