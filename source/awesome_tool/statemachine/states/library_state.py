"""
.. module:: library_state
   :platform: Unix, Windows
   :synopsis: A module to represent a library state in the state machine

.. moduleauthor:: Sebastian Brunner


"""
from gtkmvc import Observable
import yaml

from statemachine.states.state import State, StateType
from statemachine.states.container_state import ContainerState
import statemachine.singleton
from utils import log
logger = log.get_logger(__name__)


class LibraryState(ContainerState, Observable, yaml.YAMLObject):

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
                 sm_status=None, states=None, transitions=None, data_flows=None, start_state=None,
                 scoped_variables=None, v_checker=None, path=None, filename=None, state_type=None):
        #logger.debug("------------------------------------------")
        logger.debug("Initialized library state %s" % library_name)
        #logger.debug("------------------------------------------")
        ContainerState.__init__(self, name=library_name, state_id=state_id, input_data_ports=input_data_ports,
                                output_data_ports=output_data_ports, outcomes=outcomes, sm_status=sm_status,
                                states=states, transitions=transitions, data_flows=data_flows, start_state=start_state,
                                scoped_variables=scoped_variables, v_checker=v_checker, path=path, filename=filename,
                                state_type=StateType.LIBRARY)
        Observable.__init__(self)

        self._library_path = None
        self.library_path = library_path
        self._library_name = None
        self.library_name = library_name
        self._version = None
        self.version = version

        self._state_copy = None

        # for element in library_path.split("/"):
        #     print element
        path_list = library_path.split("/")
        target_lib_dict = statemachine.singleton.library_manager.libraries
        for element in path_list:
            target_lib_dict = target_lib_dict[element]
        # get a fresh copy of the library state from disk
        [self.state_copy, lib_version, creationtime] = statemachine.singleton.library_manager.storage.\
            load_statemachine_from_yaml(target_lib_dict[library_name])
        if not str(lib_version) == version:
            raise AttributeError("Library does not have the correct version!")

        #copy all ports and outcomes of self.state_copy to let the library state appear like the container state
        self.input_data_ports = self.state_copy.input_data_ports
        self.output_data_ports = self.state_copy.output_data_ports
        self.outcomes = self.state_copy.outcomes

    def __str__(self):
        return str(self.state_copy) + "library_path: %s, library_name: %s, version: %s, state_id: %s" % \
               (self.library_path, self.library_name, self.version, self.state_id)

    #just call the run method of the container state
    def run(self):
        self.active = True
        logger.debug("Entering library state %s" % self.library_name)
        self.state_copy.input_data = self.input_data
        self.state_copy.output_data = self.output_data
        self.state_copy.run()
        self.final_outcome = self.state_copy.final_outcome
        logger.debug("Exiting library state %s" % self.library_name)
        self.active = False


    @classmethod
    def to_yaml(cls, dumper, data):
        container_dict_representation = ContainerState.get_container_state_yaml_dict(data)
        dict_representation = {
            'library_path': data.library_path,
            'library_name': data.library_name,
            'version': data.version,
        }
        print "Before: ", dict_representation
        dict_representation.update(container_dict_representation)
        print "After: ", dict_representation
        node = dumper.represent_mapping(u'!LibraryState', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node)
        return LibraryState(library_path=dict_representation['library_path'],
                            library_name=dict_representation['library_name'],
                            version=dict_representation['version'],
                            state_id=dict_representation['state_id'],
                            input_data_ports=dict_representation['input_data_ports'],
                            output_data_ports=dict_representation['output_data_ports'],
                            outcomes=dict_representation['outcomes'],
                            sm_status=None,
                            states=None,
                            transitions=dict_representation['transitions'],
                            data_flows=dict_representation['data_flows'],
                            start_state=dict_representation['start_state'],
                            scoped_variables=dict_representation['scoped_variables'],
                            v_checker=None,
                            path=dict_representation['path'],
                            filename=dict_representation['filename'])

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
        if not isinstance(state_copy, ContainerState):
            raise TypeError("state_copy must be of type ContainerState")

        self._state_copy = state_copy