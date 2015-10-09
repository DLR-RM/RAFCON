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

logger = log.get_logger(__name__)


class LibraryState(State):

    """A class to represent a hierarchy state for the state machine

    Only the variables are listed that are not already contained in the container state base class

    :ivar library_path: the path of the library relative to a certain library path (e.g. lwr/gripper/)
    :ivar library_name: the name of the library between all child states: (e.g. open, or close)
    :ivar version: the version of the needed library

    """

    yaml_tag = u'!LibraryState'

    def __init__(self, library_path=None, library_name=None, version=None,  # library state specific attributes
                 # the following are the container state specific attributes
                 name=None, state_id=None, outcomes=None,
                 input_data_port_runtime_values={}, use_runtime_value_input_data_ports={},
                 output_data_port_runtime_values={}, use_runtime_value_output_data_ports={}):

        # this variable is set to true if the state initialization is finished! after initialization no change to the
        # library state is allowed any more
        self.initialized = False
        State.__init__(self, name, state_id, None, None, outcomes)

        self._library_path = None
        self.library_path = library_path
        self._library_name = None
        self.library_name = library_name
        self._version = None
        self.version = version

        self._state_copy = None
        lib_os_path, new_library_path, new_library_name = \
            library_manager.get_os_path_to_library(library_path, library_name)

        if library_path != new_library_path or library_name != new_library_name:
            self.library_name = new_library_name
            self.library_path = new_library_path
            logger.info("Changing information about location of library")
            logger.info("Old library name '{0}' was located at {1}".format(library_name, library_path))
            logger.info("New library name '{0}' is located at {1}".format(new_library_name, new_library_path))

        state_machine, lib_version, creation_time = library_manager.storage.load_statemachine_from_yaml(lib_os_path)
        self.state_copy = state_machine.root_state
        self.state_copy.parent = self
        if not str(lib_version) == version and not str(lib_version) == "None":
            raise AttributeError("Library does not have the correct version!")

        # copy all ports and outcomes of self.state_copy to let the library state appear like the container state
        self.outcomes = self.state_copy.outcomes
        self.input_data_ports = self.state_copy.input_data_ports
        self.output_data_ports = self.state_copy.output_data_ports

        # handle runtime values
        # input runtime values
        self._input_data_port_runtime_values = {}
        self._use_runtime_value_input_data_ports = {}
        if len(input_data_port_runtime_values) == 0:  # no input data port runtime values specified
            for key, idp in self.input_data_ports.iteritems():
                self.input_data_port_runtime_values[key] = idp.default_value
                self.use_runtime_value_input_data_ports[key] = True  # TODO: True or False better here?
        else:
            self.input_data_port_runtime_values = input_data_port_runtime_values
            self.use_runtime_value_input_data_ports = use_runtime_value_input_data_ports

        # output runtime values
        self._output_data_port_runtime_values = {}
        self._use_runtime_value_output_data_ports = {}
        if len(output_data_port_runtime_values) == 0:  # no output data port runtime values specified
            for key, idp in self.output_data_ports.iteritems():
                self.output_data_port_runtime_values[key] = idp.default_value
                self.use_runtime_value_output_data_ports[key] = True
        else:
            self.output_data_port_runtime_values = output_data_port_runtime_values
            self.use_runtime_value_output_data_ports = use_runtime_value_output_data_ports

        logger.debug("Initialized library state with name %s" % name)
        self.initialized = True

    def __str__(self):
        return str(self.state_copy) + "library_path: %s, library_name: %s, version: %s, state_id: %s" % \
                                      (self.library_path, self.library_name, self.version, self.state_id)

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

    def add_input_data_port(self, name, data_type, default_value=None, data_port_id=None):
        """Overwrites the add_input_data_port method of the State class. Prevents user from adding a
        output data port to the library state.

        For further documentation, look at the State class.

        """
        if self.initialized:
            logger.error("It is not allowed to add a input data port to a library state")
        else:
            return State.add_input_data_port(self, name, data_type, default_value, data_port_id)

    def add_state(self, state):
        """Overwrites the add_state method of the ContainerState class. Prevents user from adding a state to the library state.

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

    def add_input_runtime_value(self, input_data_port_id, value):
        self.input_data_port_runtime_values[input_data_port_id] = value

    def set_use_input_runtime_value(self, input_data_port_id, use_value):
        self.use_runtime_value_input_data_ports[input_data_port_id] = use_value
        
    def add_output_runtime_value(self, output_data_port_id, value):
        self.output_data_port_runtime_values[output_data_port_id] = value

    def set_use_output_runtime_value(self, output_data_port_id, use_value):
        self.use_runtime_value_output_data_ports[output_data_port_id] = use_value

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'library_path': data.library_path,
            'library_name': data.library_name,
            'version': data.version,
            'name': data.name,
            'state_id': data.state_id,
            'input_data_port_runtime_values': data.input_data_port_runtime_values,
            'use_runtime_value_input_data_ports': data.use_runtime_value_input_data_ports,
            'output_data_port_runtime_values': data.output_data_port_runtime_values,
            'use_runtime_value_output_data_ports': data.use_runtime_value_output_data_ports,
            'outcomes': data.outcomes,
        }
        node = dumper.represent_mapping(cls.yaml_tag, dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        library_path = dict_representation['library_path']
        library_name = dict_representation['library_name']
        version = dict_representation['version']
        name = dict_representation['name']
        state_id = dict_representation['state_id']
        outcomes = dict_representation['outcomes']

        input_data_port_runtime_values = {}
        use_runtime_value_input_data_ports = {}
        output_data_port_runtime_values = {}
        use_runtime_value_output_data_ports = {}

        if 'input_data_ports' in dict_representation:  # this case is for backward compatibility
            for idp_id, input_data_port in dict_representation['input_data_ports'].iteritems():
                input_data_port_runtime_values[idp_id] = input_data_port.default_value
                use_runtime_value_input_data_ports[idp_id] = True
            
            for odp_id, output_data_port in dict_representation['output_data_ports'].iteritems():
                output_data_port_runtime_values[odp_id] = output_data_port.default_value
                use_runtime_value_output_data_ports[odp_id] = True
        else:  # this is the default case
            input_data_port_runtime_values = dict_representation['input_data_port_runtime_values']
            use_runtime_value_input_data_ports = dict_representation['use_runtime_value_input_data_ports']
            output_data_port_runtime_values = dict_representation['output_data_port_runtime_values']
            use_runtime_value_output_data_ports = dict_representation['use_runtime_value_output_data_ports']

        return LibraryState(library_path, library_name, version, name, state_id, outcomes,
                            input_data_port_runtime_values, use_runtime_value_input_data_ports,
                            output_data_port_runtime_values, use_runtime_value_output_data_ports)

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
        if not isinstance(library_path, str):
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
        if not isinstance(library_name, str):
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
        if not isinstance(version, str):
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