"""
.. module:: library_state
   :platform: Unix, Windows
   :synopsis: A module to represent a library state in the state machine

.. moduleauthor:: Sebastian Brunner


"""
from gtkmvc import Observable

from rafcon.statemachine.enums import StateExecutionState
from rafcon.statemachine.states.state import State
from rafcon.statemachine.singleton import library_manager
from rafcon.utils import log
from rafcon.utils.type_helpers import convert_string_value_to_type_value

logger = log.get_logger(__name__)


class LibraryState(State):
    """A class to represent a library state for the state machine

    Only the variables are listed that are not already contained in the state base class

    :ivar library_path: the path of the library relative to a certain library path (e.g. lwr/gripper/)
    :ivar library_name: the name of the library between all child states: (e.g. open, or close)
    :ivar version: the version of the needed library
    """

    yaml_tag = u'!LibraryState'

    _library_path = None
    _library_name = None
    _version = None
    _state_copy = None

    _input_data_port_runtime_values = {}
    _use_runtime_value_input_data_ports = {}
    _output_data_port_runtime_values = {}
    _use_runtime_value_output_data_ports = {}

    def __init__(self, library_path=None, library_name=None, version=None,  # library state specific attributes
                 # the following are the container state specific attributes
                 name=None, state_id=None, outcomes=None,
                 input_data_port_runtime_values=None, use_runtime_value_input_data_ports=None,
                 output_data_port_runtime_values=None, use_runtime_value_output_data_ports=None):

        # this variable is set to true if the state initialization is finished! after initialization no change to the
        # library state is allowed any more
        self.initialized = False
        State.__init__(self, name, state_id, None, None, outcomes)

        self.library_path = library_path
        self.library_name = library_name
        self.version = version

        lib_os_path, new_library_path, new_library_name = \
            library_manager.get_os_path_to_library(library_path, library_name)

        if library_path != new_library_path or library_name != new_library_name:
            self.library_name = new_library_name
            self.library_path = new_library_path
            logger.info("Changing information about location of library")
            logger.info("Old library name '{0}' was located at {1}".format(library_name, library_path))
            logger.info("New library name '{0}' is located at {1}".format(new_library_name, new_library_path))

        state_machine, lib_version, creation_time = library_manager.storage.load_statemachine_from_path(lib_os_path)
        self.state_copy = state_machine.root_state
        self.state_copy.parent = self
        if not str(lib_version) == version and not str(lib_version) == "None":
            raise AttributeError("Library does not have the correct version!")

        # copy all ports and outcomes of self.state_copy to let the library state appear like the container state
        # this will also set the parent of all outcomes and data ports to self
        self.outcomes = self.state_copy.outcomes
        self.input_data_ports = self.state_copy.input_data_ports
        self.output_data_ports = self.state_copy.output_data_ports

        # handle runtime values
        # input runtime values
        self.input_data_port_runtime_values = input_data_port_runtime_values
        self.use_runtime_value_input_data_ports = use_runtime_value_input_data_ports
        for key, idp in self.input_data_ports.iteritems():  # check if all input data ports have a runtime value
            if key not in self.input_data_port_runtime_values.iterkeys():
                self.input_data_port_runtime_values[key] = idp.default_value
                self.use_runtime_value_input_data_ports[key] = True

        # output runtime values
        self.output_data_port_runtime_values = output_data_port_runtime_values
        self.use_runtime_value_output_data_ports = use_runtime_value_output_data_ports
        for key, idp in self.output_data_ports.iteritems():  # check if all output data ports have a runtime value
            if key not in self.output_data_port_runtime_values.iterkeys():
                self.output_data_port_runtime_values[key] = idp.default_value
                self.use_runtime_value_output_data_ports[key] = True

        self.initialized = True

    def run(self):
        """ This defines the sequence of actions that are taken when the library state is executed

        It basically just calls the run method of the container state
        :return:
        """
        self.state_execution_status = StateExecutionState.ACTIVE
        logger.debug("Entering library state '{0}' with name '{1}'".format(self.library_name, self.name))
        # self.state_copy.parent = self.parent
        self.state_copy.input_data = self.input_data
        self.state_copy.output_data = self.output_data
        self.state_copy.execution_history = self.execution_history
        self.state_copy.backward_execution = self.backward_execution
        self.state_copy.run()
        logger.debug("Exiting library state '{0}' with name '{1}'".format(self.library_name, self.name))
        self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
        self.finalize(self.state_copy.final_outcome)

    def recursively_preempt_states(self):
        """ Preempt the state and all of it child states.
        """
        self.preempted = True
        self.state_copy.recursively_preempt_states()

    def add_outcome(self, name, outcome_id=None):
        """Overwrites the add_outcome method of the State class. Prevents user from adding a
        outcome to the library state.

        For further documentation, look at the State class.

        """
        if self.initialized:
            logger.error("It is not allowed to add a outcome to a library state")
        else:
            return State.add_outcome(self, name, outcome_id)

    def add_output_data_port(self, name, data_type, default_value=None, data_port_id=None):
        """Overwrites the add_output_data_port method of the State class. Prevents user from adding a
        output data port to the library state.

        For further documentation, look at the State class.

        """
        if self.initialized:
            logger.error("It is not allowed to add a output data port to a library state")
        else:
            return State.add_output_data_port(self, name, data_type, default_value, data_port_id)

    def add_input_data_port(self, name, data_type=None, default_value=None, data_port_id=None):
        """Overwrites the add_input_data_port method of the State class. Prevents user from adding a
        output data port to the library state.

        For further documentation, look at the State class.

        """
        if self.initialized:
            logger.error("It is not allowed to add a input data port to a library state")
        else:
            return State.add_input_data_port(self, name, data_type, default_value, data_port_id)

    def add_state(self, state):
        """Overwrites the add_state method of the ContainerState class. Prevents user from adding a state to the
        library state.

        For further documentation, look at the ContainerState class.

        """
        if self.initialized:
            logger.error("It is not allowed to add a state to a library state")
        else:
            return State.add_state(self, state)

    def add_scoped_variable(self, name, data_type=None, default_value=None, scoped_variable_id=None):
        """Overwrites the add_scoped_variable method of the ContainerState class. Prevents user from adding a
        scoped_variable to the library state.

        For further documentation, look at the ContainerState class.

        """
        if self.initialized:
            logger.error("It is not allowed to add a scoped_variable to a library state")
        else:
            return State.add_scoped_variable(self, name, data_type, default_value, scoped_variable_id)

    @Observable.observed
    def set_input_runtime_value(self, input_data_port_id, value):
        checked_value = self.state_copy.input_data_ports[input_data_port_id].check_default_value(value)
        self._input_data_port_runtime_values[input_data_port_id] = checked_value

    @Observable.observed
    def set_use_input_runtime_value(self, input_data_port_id, use_value):
        self._use_runtime_value_input_data_ports[input_data_port_id] = use_value

    @Observable.observed
    def set_output_runtime_value(self, output_data_port_id, value):
        checked_value = self.state_copy.output_data_ports[output_data_port_id].check_default_value(value)
        self._output_data_port_runtime_values[output_data_port_id] = checked_value

    @Observable.observed
    def set_use_output_runtime_value(self, output_data_port_id, use_value):
        self._use_runtime_value_output_data_ports[output_data_port_id] = use_value

    # overwrite data port function of State class, actually not necessary as the data ports cannot be changed
    # from the library state, but have to be changed in the library-statemachine itself

    @Observable.observed
    def add_input_data_port(self, name, data_type=None, default_value=None, data_port_id=None):
        self._use_runtime_value_input_data_ports[data_port_id] = False
        self._input_data_port_runtime_values[data_port_id] = default_value
        return State.add_input_data_port(name, data_type, default_value, data_port_id)

    @Observable.observed
    def remove_input_data_port(self, data_port_id):
        del self._use_runtime_value_input_data_ports[data_port_id]
        del self._input_data_port_runtime_values[data_port_id]
        State.remove_input_data_port(data_port_id)

    @Observable.observed
    def add_output_data_port(self, name, data_type, default_value=None, data_port_id=None):
        self._use_runtime_value_output_data_ports[data_port_id] = False
        self._output_data_port_runtime_values[data_port_id] = default_value
        State.add_output_data_port(name, data_type, default_value, data_port_id)

    @Observable.observed
    def remove_output_data_port(self, data_port_id):
        del self._use_runtime_value_output_data_ports[data_port_id]
        del self._output_data_port_runtime_values[data_port_id]
        State.remove_output_data_port(data_port_id)

    @classmethod
    def from_dict(cls, dictionary):
        library_path = dictionary['library_path']
        library_name = dictionary['library_name']
        version = dictionary['version']
        name = dictionary['name']
        state_id = dictionary['state_id']
        outcomes = dictionary['outcomes']

        input_data_port_runtime_values = {}
        use_runtime_value_input_data_ports = {}
        output_data_port_runtime_values = {}
        use_runtime_value_output_data_ports = {}

        if 'input_data_ports' in dictionary:  # this case is for backward compatibility
            for idp_id, input_data_port in dictionary['input_data_ports'].iteritems():
                input_data_port_runtime_values[idp_id] = input_data_port.default_value
                use_runtime_value_input_data_ports[idp_id] = True

            for odp_id, output_data_port in dictionary['output_data_ports'].iteritems():
                output_data_port_runtime_values[odp_id] = output_data_port.default_value
                use_runtime_value_output_data_ports[odp_id] = True
        else:  # this is the default case
            input_data_port_runtime_values = dictionary['input_data_port_runtime_values']
            use_runtime_value_input_data_ports = dictionary['use_runtime_value_input_data_ports']
            output_data_port_runtime_values = dictionary['output_data_port_runtime_values']
            use_runtime_value_output_data_ports = dictionary['use_runtime_value_output_data_ports']

        return cls(library_path, library_name, version, name, state_id, outcomes,
                   input_data_port_runtime_values, use_runtime_value_input_data_ports,
                   output_data_port_runtime_values, use_runtime_value_output_data_ports)

    @staticmethod
    def state_to_dict(state):
        dict_representation = {
            'library_path': state.library_path,
            'library_name': state.library_name,
            'version': state.version,
            'name': state.name,
            'state_id': state.state_id,
            'outcomes': state.outcomes,
            'input_data_port_runtime_values': state.input_data_port_runtime_values,
            'use_runtime_value_input_data_ports': state.use_runtime_value_input_data_ports,
            'output_data_port_runtime_values': state.output_data_port_runtime_values,
            'use_runtime_value_output_data_ports': state.use_runtime_value_output_data_ports
        }
        return dict_representation

    #########################################################################
    # Properties for all class fields that must be observed by gtkmvc
    #########################################################################

    @property
    def library_path(self):
        """Property for the _library_path field

        """
        return self._library_path

    @library_path.setter
    @Observable.observed
    def library_path(self, library_path):
        if not isinstance(library_path, basestring):
            raise TypeError("library_path must be of type str")

        self._library_path = library_path

    @property
    def library_name(self):
        """Property for the _library_name field

        """
        return self._library_name

    @library_name.setter
    @Observable.observed
    def library_name(self, library_name):
        if not isinstance(library_name, basestring):
            raise TypeError("library_name must be of type str")

        self._library_name = library_name

    @property
    def version(self):
        """Property for the _version field

        """
        return self._version

    @version.setter
    @Observable.observed
    def version(self, version):
        if not isinstance(version, basestring):
            raise TypeError("version must be of type str")

        self._version = version

    @property
    def state_copy(self):
        """Property for the _state_copy field

        """
        return self._state_copy

    @state_copy.setter
    @Observable.observed
    def state_copy(self, state_copy):
        if not isinstance(state_copy, State):
            raise TypeError("state_copy must be of type State")

        self._state_copy = state_copy

    @property
    def input_data_port_runtime_values(self):
        """Property for the _input_data_port_runtime_values field

        """
        return self._input_data_port_runtime_values

    @input_data_port_runtime_values.setter
    @Observable.observed
    def input_data_port_runtime_values(self, input_data_port_runtime_values):
        if not input_data_port_runtime_values:
            self._input_data_port_runtime_values = {}
        else:
            if not isinstance(input_data_port_runtime_values, dict):
                raise TypeError("input_data_port_runtime_values must be of type dict")
            self._input_data_port_runtime_values = input_data_port_runtime_values

    @property
    def use_runtime_value_input_data_ports(self):
        """Property for the _use_runtime_value_input_data_ports field

        """
        return self._use_runtime_value_input_data_ports

    @use_runtime_value_input_data_ports.setter
    @Observable.observed
    def use_runtime_value_input_data_ports(self, use_runtime_value_input_data_ports):
        if not use_runtime_value_input_data_ports:
            self._use_runtime_value_input_data_ports = {}
        else:
            if not isinstance(use_runtime_value_input_data_ports, dict):
                raise TypeError("use_runtime_value_input_data_ports must be of type dict")
            self._use_runtime_value_input_data_ports = use_runtime_value_input_data_ports

    @property
    def output_data_port_runtime_values(self):
        """Property for the _output_data_port_runtime_values field

        """
        return self._output_data_port_runtime_values

    @output_data_port_runtime_values.setter
    @Observable.observed
    def output_data_port_runtime_values(self, output_data_port_runtime_values):
        if not output_data_port_runtime_values:
            self._output_data_port_runtime_values = {}
        else:
            if not isinstance(output_data_port_runtime_values, dict):
                raise TypeError("output_data_port_runtime_values must be of type dict")
            self._output_data_port_runtime_values = output_data_port_runtime_values

    @property
    def use_runtime_value_output_data_ports(self):
        """Property for the _use_runtime_value_output_data_ports field

        """
        return self._use_runtime_value_output_data_ports

    @use_runtime_value_output_data_ports.setter
    @Observable.observed
    def use_runtime_value_output_data_ports(self, use_runtime_value_output_data_ports):
        if not use_runtime_value_output_data_ports:
            self._use_runtime_value_output_data_ports = {}
        else:
            if not isinstance(use_runtime_value_output_data_ports, dict):
                raise TypeError("use_runtime_value_output_data_ports must be of type dict")
            self._use_runtime_value_output_data_ports = use_runtime_value_output_data_ports

    @staticmethod
    def copy_state(source_state):
        """
        Copies a state.
        :param source_state: the state to copy
        :return: the copy of the source state
        """
        state_copy = LibraryState()
        # TODO: copy fields from source_state into the state_copy
        return state_copy

    @property
    def child_execution(self):
        """Property for the _child_execution field
        """
        if self.state_execution_status is StateExecutionState.EXECUTE_CHILDREN:
            return True
        else:
            return False
