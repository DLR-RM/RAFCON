"""
.. module:: library_state
   :platform: Unix, Windows
   :synopsis: A module to represent a library state in the state machine

.. moduleauthor:: Sebastian Brunner


"""
from gtkmvc import Observable
import yaml

from statemachine.enums import StateType
from statemachine.states.container_state import ContainerState
from statemachine.states.state import State
import statemachine.singleton
from utils import log

logger = log.get_logger(__name__)


class LibraryState(ContainerState, yaml.YAMLObject):

    """A class to represent a hierarchy state for the state machine

    Only the variables are listed that are not already contained in the container state base class

    :ivar library_path: the path of the library relative to a certain library path (e.g. lwr/gripper/)
    :ivar library_name: the name of the library between all child states: (e.g. open, or close)
    :ivar version: the version of the needed library

    """

    yaml_tag = u'!LibraryState'

    def __init__(self, library_path=None, library_name=None, version=None,  # library state specific attributes
                 # the following are the container state specific attributes
                 name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None, check_path=True):

        # this variable is set to true if the state initialization is finished! after initialization no change to the
        # library state is allowed any more
        self.initialized = False
        ContainerState.__init__(self, name=name, state_id=state_id, input_data_ports=input_data_ports,
                                output_data_ports=output_data_ports, outcomes=outcomes, states=states,
                                transitions=transitions, data_flows=data_flows, start_state=start_state,
                                scoped_variables=scoped_variables, v_checker=v_checker, path=path, filename=filename,
                                state_type=StateType.LIBRARY, check_path=check_path)

        self._library_path = None
        self.library_path = library_path
        self._library_name = None
        self.library_name = library_name
        self._version = None
        self.version = version

        self._state_copy = None

        path_list = library_path.split("/")
        target_lib_dict = statemachine.singleton.library_manager.libraries
        # go down the path to the correct library
        # TODO: make this more robust
        for element in path_list:
            target_lib_dict = target_lib_dict[element]
        # get a fresh copy of the library state from disk
        logger.debug("Load state to which this library state links")
        state_machine, lib_version, creationtime = statemachine.singleton.library_manager.storage.\
            load_statemachine_from_yaml(target_lib_dict[library_name])
        self.state_copy = state_machine.root_state
        if not str(lib_version) == version:
            raise AttributeError("Library does not have the correct version!")

        #copy all ports and outcomes of self.state_copy to let the library state appear like the container state
        self.input_data_ports = self.state_copy.input_data_ports
        self.output_data_ports = self.state_copy.output_data_ports
        self.outcomes = self.state_copy.outcomes

        logger.debug("Initialized library state with name %s" % name)
        self.script.script = self.state_copy.script.script
        self.initialized = True

    def __str__(self):
        return str(self.state_copy) + "library_path: %s, library_name: %s, version: %s, state_id: %s" % \
                                      (self.library_path, self.library_name, self.version, self.state_id)

    def run(self):
        """ This defines the sequence of actions that are taken when the library state is executed

        It basically just calls the run method of the container state
        :return:
        """
        self.active = True
        logger.debug("Entering library state %s" % self.library_name)
        self.state_copy.input_data = self.input_data
        self.state_copy.output_data = self.output_data
        self.state_copy.run()
        self.final_outcome = self.state_copy.final_outcome
        logger.debug("Exiting library state %s" % self.library_name)
        self.active = False

    def add_outcome(self, name, outcome_id=None):
        """Overwrites the add_outcome method of the State class. Prevents user from adding a
        outcome to the library state.

        For further documentation, look at the State class.

        """
        if self.initialized:
            logger.error("It is not allowed to add a outcome to a library state")
        else:
            return ContainerState.add_outcome(self, name, outcome_id)

    def add_output_data_port(self, name, data_type, default_value=None, data_port_id=None):
        """Overwrites the add_output_data_port method of the State class. Prevents user from adding a
        output data port to the library state.

        For further documentation, look at the State class.

        """
        if self.initialized:
            logger.error("It is not allowed to add a output data port to a library state")
        else:
            return ContainerState.add_output_data_port(self, name, data_type, default_value, data_port_id)

    def add_input_data_port(self, name, data_type, default_value=None, data_port_id=None):
        """Overwrites the add_input_data_port method of the State class. Prevents user from adding a
        output data port to the library state.

        For further documentation, look at the State class.

        """
        if self.initialized:
            logger.error("It is not allowed to add a input data port to a library state")
        else:
            return ContainerState.add_input_data_port(self, name, data_type, default_value, data_port_id)

    def add_state(self, state):
        """Overwrites the add_state method of the ContainerState class. Prevents user from adding a state to the library state.

        For further documentation, look at the ContainerState class.

        """
        if self.initialized:
            logger.error("It is not allowed to add a state to a library state")
        else:
            return ContainerState.add_state(self, state)

    def add_transition(self, from_state_id, from_outcome, to_state_id=None, to_outcome=None, transition_id=None):
        """Overwrites the add_transition method of the ContainerState class. Prevents user from adding a
        transition to the library state.

        For further documentation, look at the ContainerState class.

        """
        if self.initialized:
            logger.error("It is not allowed to add a transition to a library state")
        else:
            return ContainerState.add_transition(self, from_state_id, from_outcome, to_state_id, to_outcome, transition_id)

    def add_data_flow(self, from_state_id, from_data_port_id, to_state_id, to_data_port_id, data_flow_id=None):
        """Overwrites the add_data_flow method of the ContainerState class. Prevents user from adding a
        data_flow to the library state.

        For further documentation, look at the ContainerState class.

        """
        if self.initialized:
            logger.error("It is not allowed to add a data flow to a library state")
        else:
            return ContainerState.add_data_flow(self, from_state_id, from_data_port_id, to_state_id, to_data_port_id,
                                                data_flow_id)

    def add_scoped_variable(self, name, data_type=None, default_value=None, scoped_variable_id=None):
        """Overwrites the add_scoped_variable method of the ContainerState class. Prevents user from adding a
        scoped_variable to the library state.

        For further documentation, look at the ContainerState class.

        """
        if self.initialized:
            logger.error("It is not allowed to add a scoped_variable to a library state")
        else:
            return ContainerState.add_scoped_variable(self, name, data_type, default_value, scoped_variable_id)

    def set_script_text(self, new_text):
        """
        Overwrites the set_script_text method of the
        circumstances.
        :param new_text: the new text
        :return:
        """
        if self.initialized:
            logger.error("It is not allowed to set the script text of a library state")
            return False
        else:
            return ContainerState.set_script_text(self, new_text)

    @classmethod
    def to_yaml(cls, dumper, data):
        container_dict_representation = ContainerState.get_container_state_yaml_dict(data)
        dict_representation = {
            'library_path': data.library_path,
            'library_name': data.library_name,
            'version': data.version,
        }
        dict_representation.update(container_dict_representation)
        node = dumper.represent_mapping(u'!LibraryState', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node)
        return LibraryState(library_path=dict_representation['library_path'],
                            library_name=dict_representation['library_name'],
                            version=dict_representation['version'],
                            name=dict_representation['name'],
                            state_id=dict_representation['state_id'],
                            input_data_ports=dict_representation['input_data_ports'],
                            output_data_ports=dict_representation['output_data_ports'],
                            outcomes=dict_representation['outcomes'],
                            states=None,
                            transitions=dict_representation['transitions'],
                            data_flows=dict_representation['data_flows'],
                            start_state=dict_representation['start_state'],
                            scoped_variables=dict_representation['scoped_variables'],
                            v_checker=None,
                            path=dict_representation['path'],
                            filename=dict_representation['filename'],
                            check_path=False)

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