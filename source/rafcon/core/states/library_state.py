# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Michael Vilzmann <michael.vilzmann@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: library_state
   :synopsis: A module to represent a library state in the state machine

"""
from future.utils import string_types
from builtins import str
from weakref import ref
from copy import copy, deepcopy

from gtkmvc3.observable import Observable
from rafcon.core.states.state import StateExecutionStatus
from rafcon.core.singleton import library_manager
from rafcon.core.states.state import State, PATH_SEPARATOR
from rafcon.core.decorators import lock_state_machine
from rafcon.core.config import global_config
from rafcon.utils import log
from rafcon.utils import type_helpers
from rafcon.utils.hashable import Hashable

logger = log.get_logger(__name__)


class LibraryState(State):
    """A class to represent a library state for the state machine

    Only the variables are listed that are not already contained in the state base class.

    The constructor uses an exceptions.AttributeError if the passed version of the library and the version found in
    the library paths do not match.

    :ivar str library_path: the path of the library relative to a certain library path (e.g. lwr/gripper/)
    :ivar str library_name: the name of the library between all child states: (e.g. open, or close)
    :ivar str State.name: the name of the library state
    :ivar str State.state_id: the id of the library state
    :ivar dict input_data_port_runtime_values: a dict to store all the runtime values for the input data ports
    :ivar dict use_runtime_value_input_data_ports: flags to indicate if the runtime or the default value should be used
                                                    for a specific input data port
    :ivar dict output_data_port_runtime_values: a dict to store all the runtime values for the output data ports
    :ivar dict use_runtime_value_output_data_ports: flags to indicate if the runtime or the default value should be used
                                                    for a specific output data port
    :ivar dict allow_user_interaction: flag to indicate if the user can support in localizing moved libraries
    :ivar skip_runtime_data_initialization: flag to indicate if the runtime-data data structures have to be initialized,
                                            this is not needed e.g. in the case of a copy
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
                 name=None, state_id=None,
                 income=None, outcomes=None,
                 input_data_port_runtime_values=None, use_runtime_value_input_data_ports=None,
                 output_data_port_runtime_values=None, use_runtime_value_output_data_ports=None,
                 allow_user_interaction=True, safe_init=True, skip_runtime_data_initialization=False):

        # this variable is set to true if the state initialization is finished! after initialization no change to the
        # library state is allowed any more
        self.initialized = False
        State.__init__(self, name, state_id, None, None, income, outcomes, safe_init=safe_init)

        self.library_path = library_path
        self.library_name = library_name
        self.version = version

        lib_os_path, new_library_path, new_library_name = \
            library_manager.get_os_path_to_library(library_path, library_name, allow_user_interaction)
        self.lib_os_path = lib_os_path

        if library_path != new_library_path or library_name != new_library_name:
            self.library_name = new_library_name
            self.library_path = new_library_path
            # TODO this should trigger the marked_dirty of the state machine to become true
            logger.info("Changing information about location of library")
            logger.info("Old library name '{0}' was located at {1}".format(library_name, library_path))
            logger.info("New library name '{0}' is located at {1}".format(new_library_name, new_library_path))

        # key = load_library_root_state_timer.start()
        lib_version, state_copy = library_manager.get_library_state_copy_instance(self.lib_os_path)
        if not str(lib_version) == version and not str(lib_version) == "None":
            raise AttributeError("Library does not have the correct version!")
        self.state_copy = state_copy

        if safe_init:
            LibraryState._safe_init(self, name)
        else:
            LibraryState._unsafe_init(self, name)

        if not skip_runtime_data_initialization:
            # load_library_root_state_timer.stop(key)
            self._handle_runtime_values(input_data_port_runtime_values, use_runtime_value_input_data_ports,
                                        output_data_port_runtime_values, use_runtime_value_output_data_ports)
        else:
            self._input_data_port_runtime_values = input_data_port_runtime_values
            self._use_runtime_value_input_data_ports = use_runtime_value_input_data_ports
            self._output_data_port_runtime_values = output_data_port_runtime_values
            self._use_runtime_value_output_data_ports = use_runtime_value_output_data_ports

        self.initialized = True

    def _safe_init(self, name):
        self.state_copy.parent = self
        if name is None:
            self.name = self.state_copy.name
        # copy all ports and outcomes of self.state_copy to let the library state appear like the container state
        # this will also set the parent of all outcomes and data ports to self
        self.outcomes = self.state_copy.outcomes
        self.input_data_ports = self.state_copy.input_data_ports
        self.output_data_ports = self.state_copy.output_data_ports

    def _unsafe_init(self, name):
        self.state_copy._parent = ref(self)
        if name is None:
            self._name = self.state_copy.name
        self._outcomes = self.state_copy.outcomes
        # add parents manually
        for outcome_id, outcome in self._outcomes.items():
            outcome._parent = ref(self)
        self._input_data_ports = self.state_copy.input_data_ports
        for port_id, port in self._input_data_ports.items():
            port._parent = ref(self)
        self._output_data_ports = self.state_copy.output_data_ports
        for port_id, port in self._output_data_ports.items():
            port._parent = ref(self)

    def _handle_runtime_values(self, input_data_port_runtime_values, use_runtime_value_input_data_ports,
                               output_data_port_runtime_values, use_runtime_value_output_data_ports):
        # handle input runtime values
        self.input_data_port_runtime_values = input_data_port_runtime_values
        self.use_runtime_value_input_data_ports = use_runtime_value_input_data_ports
        for data_port_id, data_port in self.input_data_ports.items():
            # Ensure that all input data ports have a runtime value
            if data_port_id not in self.input_data_port_runtime_values.keys():
                self.input_data_port_runtime_values[data_port_id] = data_port.default_value
                self.use_runtime_value_input_data_ports[data_port_id] = True
            # Ensure that str and unicode is correctly differentiated
            elif isinstance(self.input_data_port_runtime_values[data_port_id], string_types):
                try:
                    self.input_data_port_runtime_values[data_port_id] = type_helpers.convert_string_value_to_type_value(
                        self.input_data_port_runtime_values[data_port_id], data_port.data_type)
                except AttributeError:
                    # the parameter cannot be converted
                    # this can be the case when the data type of port of the library state changed
                    self.input_data_port_runtime_values[data_port_id] = data_port.default_value
                    self.use_runtime_value_input_data_ports[data_port_id] = True
                    self.marked_dirty = True

        # if there is a key existing in the runtime values but not in the input_data_ports we delete it
        for key in list(self.use_runtime_value_input_data_ports.keys()):
            if key not in self.input_data_ports:
                del self.use_runtime_value_input_data_ports[key]
                del self.input_data_port_runtime_values[key]
                # state machine cannot be marked dirty directly, as it does not exist yet
                self.marked_dirty = True

        # handle output runtime values
        self.output_data_port_runtime_values = output_data_port_runtime_values
        self.use_runtime_value_output_data_ports = use_runtime_value_output_data_ports
        for data_port_id, data_port in self.output_data_ports.items():
            # Ensure that all output data ports have a runtime value
            if data_port_id not in self.output_data_port_runtime_values.keys():
                self.output_data_port_runtime_values[data_port_id] = data_port.default_value
                self.use_runtime_value_output_data_ports[data_port_id] = True
            # Ensure that str and unicode is correctly differentiated
            elif isinstance(self.output_data_port_runtime_values[data_port_id], string_types):
                try:
                    self.output_data_port_runtime_values[data_port_id] = \
                        type_helpers.convert_string_value_to_type_value(
                            self.output_data_port_runtime_values[data_port_id], data_port.data_type)
                except AttributeError:
                    # the parameter cannot be converted
                    # this can be the case when the data type of port of the library state changed
                    self.output_data_port_runtime_values[data_port_id] = data_port.default_value
                    self.use_runtime_value_output_data_ports[data_port_id] = True
                    self.marked_dirty = True

        # if there is a key existing in the runtime values but not in the output_data_ports we delete it
        for key in list(self.use_runtime_value_output_data_ports.keys()):
            if key not in self.output_data_ports:
                del self.use_runtime_value_output_data_ports[key]
                del self.output_data_port_runtime_values[key]
                # state machine cannot be marked dirty directly, as it does not exist yet
                self.marked_dirty = True

    def __hash__(self):
        return id(self)

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        return str(self) == str(other) and self._state_copy == other.state_copy

    def __copy__(self):
        income = self._income
        outcomes = {key: copy(self.outcomes[key]) for key in self.outcomes.keys()}
        state = self.__class__(self._library_path, self._library_name, self._version,  # library specific attributes
                               # the following are the container state specific attributes
                               self._name, self._state_id, income, outcomes,
                               copy(self.input_data_port_runtime_values), copy(self.use_runtime_value_input_data_ports),
                               copy(self.output_data_port_runtime_values), copy(self.use_runtime_value_output_data_ports),
                               False, safe_init=False, skip_runtime_data_initialization=True)

        state._semantic_data = deepcopy(self.semantic_data)
        state._file_system_path = self.file_system_path
        return state

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    def destroy(self, recursive=True):
        super(LibraryState, self).destroy(recursive)
        if recursive:
            if self.state_copy:
                self.state_copy.destroy(recursive)
            else:
                logger.verbose("Multiple calls of destroy {0}".format(self))
            self._state_copy = None

    def run(self):
        """ This defines the sequence of actions that are taken when the library state is executed

        It basically just calls the run method of the container state
        :return:
        """
        self.state_execution_status = StateExecutionStatus.ACTIVE
        logger.debug("Entering library state '{0}' with name '{1}'".format(self.library_name, self.name))
        # self.state_copy.parent = self.parent
        self.state_copy._run_id = self._run_id
        self.state_copy.input_data = self.input_data
        self.state_copy.output_data = self.output_data
        self.state_copy.execution_history = self.execution_history
        self.state_copy.backward_execution = self.backward_execution
        self.state_copy.run()
        logger.debug("Exiting library state '{0}' with name '{1}'".format(self.library_name, self.name))
        self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE
        self.finalize(self.state_copy.final_outcome)

    def recursively_preempt_states(self):
        """Preempt the state and all of it child states.
        """
        super(LibraryState, self).recursively_preempt_states()
        self.state_copy.recursively_preempt_states()

    def recursively_pause_states(self):
        """Pause the state and all of it child states.
        """
        super(LibraryState, self).recursively_pause_states()
        self.state_copy.recursively_pause_states()

    def recursively_resume_states(self):
        """Resume the state and all of it child states.
        """
        super(LibraryState, self).recursively_resume_states()
        self.state_copy.recursively_resume_states()

    @lock_state_machine
    def add_outcome(self, name, outcome_id=None):
        """Overwrites the add_outcome method of the State class. Prevents user from adding a
        outcome to the library state.

        For further documentation, look at the State class.

        :raises exceptions.NotImplementedError: in any case
        """
        raise NotImplementedError("Add outcome is not implemented for library state {}".format(self))

    @lock_state_machine
    def remove_outcome(self, outcome_id, force=False, destroy=True):
        """Overwrites the remove_outcome method of the State class. Prevents user from removing a
        outcome from the library state.

        For further documentation, look at the State class.

        :raises exceptions.NotImplementedError: in any case
        """
        if force:
            return State.remove_outcome(self, outcome_id, force, destroy)
        else:
            raise NotImplementedError("Remove outcome is not implemented for library state {}".format(self))

    @lock_state_machine
    def add_input_data_port(self, name, data_type=None, default_value=None, data_port_id=None):
        """Overwrites the add_input_data_port method of the State class. Prevents user from adding a
        output data port to the library state.

        For further documentation, look at the State class.
        :raises exceptions.NotImplementedError: in any case
        """
        raise NotImplementedError("Add input data port is not implemented for library state {}".format(self))

    @lock_state_machine
    def remove_input_data_port(self, data_port_id, force=False, destroy=True):
        """
        Overwrites the remove_input_data_port method of the State class. Prevents user from removing a
        input data port from the library state.

        For further documentation, look at the State class.

        :param bool force: True if the removal should be forced
        :raises exceptions.NotImplementedError: in the removal is not forced
        """
        if force:
            return State.remove_input_data_port(self, data_port_id, force, destroy)
        else:
            raise NotImplementedError("Remove input data port is not implemented for library state {}".format(self))

    @lock_state_machine
    def add_output_data_port(self, name, data_type, default_value=None, data_port_id=None):
        """Overwrites the add_output_data_port method of the State class. Prevents user from adding a
        output data port to the library state.

        For further documentation, look at the State class.
        :raises exceptions.NotImplementedError: in any case
        """
        raise NotImplementedError("Add a output data port is not implemented for library state {}".format(self))

    @lock_state_machine
    def remove_output_data_port(self, data_port_id, force=False, destroy=True):
        """Overwrites the remove_output_data_port method of the State class. Prevents user from removing a
        output data port from the library state.

        For further documentation, look at the State class.
        :param bool force: True if the removal should be forced
        :raises exceptions.NotImplementedError: in the removal is not forced
        """
        if force:
            return State.remove_output_data_port(self, data_port_id, force, destroy)
        else:
            raise NotImplementedError("Remove output data port is not implemented for library state {}".format(self))

    @lock_state_machine
    @Observable.observed
    def set_input_runtime_value(self, input_data_port_id, value):
        checked_value = self.state_copy.input_data_ports[input_data_port_id].check_default_value(value)
        self._input_data_port_runtime_values[input_data_port_id] = checked_value

    @lock_state_machine
    @Observable.observed
    def set_use_input_runtime_value(self, input_data_port_id, use_value):
        self._use_runtime_value_input_data_ports[input_data_port_id] = use_value

    @lock_state_machine
    @Observable.observed
    def set_output_runtime_value(self, output_data_port_id, value):
        checked_value = self.state_copy.output_data_ports[output_data_port_id].check_default_value(value)
        self._output_data_port_runtime_values[output_data_port_id] = checked_value

    @lock_state_machine
    @Observable.observed
    def set_use_output_runtime_value(self, output_data_port_id, use_value):
        self._use_runtime_value_output_data_ports[output_data_port_id] = use_value

    @classmethod
    def from_dict(cls, dictionary):
        library_path = dictionary['library_path']
        library_name = dictionary['library_name']
        version = dictionary['version']
        name = dictionary['name']
        state_id = dictionary['state_id']
        income = dictionary.get('income', None)  # older state machine versions don't have this set
        outcomes = dictionary['outcomes']

        input_data_port_runtime_values = {}
        use_runtime_value_input_data_ports = {}
        output_data_port_runtime_values = {}
        use_runtime_value_output_data_ports = {}

        if 'input_data_ports' in dictionary:  # this case is for backward compatibility
            for idp_id, input_data_port in dictionary['input_data_ports'].items():
                input_data_port_runtime_values[idp_id] = input_data_port.default_value
                use_runtime_value_input_data_ports[idp_id] = True

            for odp_id, output_data_port in dictionary['output_data_ports'].items():
                output_data_port_runtime_values[odp_id] = output_data_port.default_value
                use_runtime_value_output_data_ports[odp_id] = True
        else:  # this is the default case
            input_data_port_runtime_values = dictionary['input_data_port_runtime_values']
            use_runtime_value_input_data_ports = dictionary['use_runtime_value_input_data_ports']
            output_data_port_runtime_values = dictionary['output_data_port_runtime_values']
            use_runtime_value_output_data_ports = dictionary['use_runtime_value_output_data_ports']

        return cls(library_path, library_name, version, name, state_id, income, outcomes,
                   input_data_port_runtime_values, use_runtime_value_input_data_ports,
                   output_data_port_runtime_values, use_runtime_value_output_data_ports, safe_init=False)

    def update_hash(self, obj_hash):
        super(LibraryState, self).update_hash(obj_hash)
        self.state_copy.update_hash(obj_hash)

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

    def get_states_statistics(self, hierarchy_level):
        """
        Returns the numer of child states. As per default states do not have child states return 1.
        :return:
        """
        return self.state_copy.get_states_statistics(hierarchy_level)

    def get_number_of_transitions(self):
        """
        Return the number of transitions for a state. Per default states do not have transitions.
        :return:
        """
        return self.state_copy.get_number_of_transitions()

    def get_number_of_data_flows(self):
        """
        Return the number of data flows for a state. Per default states do not have data flows.
        :return:
        """
        return self.state_copy.get_number_of_data_flows()

    #########################################################################
    # Properties for all class fields that must be observed by gtkmvc3
    #########################################################################

    @property
    def library_path(self):
        """Property for the _library_path field

        """
        return self._library_path

    @library_path.setter
    @lock_state_machine
    @Observable.observed
    def library_path(self, library_path):
        if not isinstance(library_path, string_types):
            raise TypeError("library_path must be a string")

        self._library_path = library_path

    @property
    def library_name(self):
        """Property for the _library_name field

        """
        return self._library_name

    @library_name.setter
    @lock_state_machine
    @Observable.observed
    def library_name(self, library_name):
        if not isinstance(library_name, string_types):
            raise TypeError("library_name must be a string")

        self._library_name = library_name

    @property
    def version(self):
        """Property for the _version field

        """
        return self._version

    @version.setter
    @lock_state_machine
    @Observable.observed
    def version(self, version):
        if version is not None and not isinstance(version, (string_types, int, float)):
            raise TypeError("version must be a string, got: {}, {}".format(type(version), version))

        self._version = str(version)

    @property
    def state_copy(self):
        """Property for the _state_copy field

        """
        return self._state_copy

    @state_copy.setter
    @lock_state_machine
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
    @lock_state_machine
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
    @lock_state_machine
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
    @lock_state_machine
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
    @lock_state_machine
    @Observable.observed
    def use_runtime_value_output_data_ports(self, use_runtime_value_output_data_ports):
        if not use_runtime_value_output_data_ports:
            self._use_runtime_value_output_data_ports = {}
        else:
            if not isinstance(use_runtime_value_output_data_ports, dict):
                raise TypeError("use_runtime_value_output_data_ports must be of type dict")
            self._use_runtime_value_output_data_ports = use_runtime_value_output_data_ports

    @property
    def child_execution(self):
        """Property for the _child_execution field
        """
        if self.state_execution_status is StateExecutionStatus.EXECUTE_CHILDREN:
            return True
        else:
            return False

    def get_storage_path(self, appendix=None):
        if appendix is None:
            return super(LibraryState, self).get_storage_path(appendix)
        else:
            return self.lib_os_path + PATH_SEPARATOR + appendix

    @property
    def library_hierarchy_depth(self):
        """ Calculates the library hierarchy depth

        Counting starts at the current library state. So if the there is no upper library state the depth is one.
        
        :return: library hierarchy depth
        :rtype: int
        """
        current_library_hierarchy_depth = 1
        library_root_state = self.get_next_upper_library_root_state()
        while library_root_state is not None:
            current_library_hierarchy_depth += 1
            library_root_state = library_root_state.parent.get_next_upper_library_root_state()
        return current_library_hierarchy_depth
