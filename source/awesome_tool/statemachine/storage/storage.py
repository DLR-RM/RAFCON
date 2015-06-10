"""
.. module:: storage
   :platform: Unix, Windows
   :synopsis: A module to store a statemachine in the local file system

.. moduleauthor:: Sebastian Brunner


"""

import os
import shutil
from time import gmtime, strftime

import yaml
from gtkmvc import Observable

from awesome_tool.statemachine.state_machine import StateMachine
from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.utils.storage_utils import StorageUtils
from awesome_tool.statemachine.enums import UNIQUE_DECIDER_STATE_ID


class StateMachineStorage(Observable):

    """This class implements the Storage interface by using a file system on the disk.

    The data storage format is a directory system.

    :ivar base_path: the base path to resolve all relative paths
    :ivar _paths_to_remove_before_sm_save: each state machine holds a list of paths that are going to be removed while
                                            saving the state machine
    :ivar ids_of_modified_state_machines: each state machine has a flag if it was modified since the last saving
    """

    GRAPHICS_FILE = 'gui_gtk.yaml'
    META_FILE = 'meta.yaml'
    SCRIPT_FILE = 'script.py'
    STATEMACHINE_FILE = 'statemachine.yaml'
    LIBRARY_FILE = 'library.yaml'

    def __init__(self, base_path='/tmp'):
        Observable.__init__(self)

        self.storage_utils = StorageUtils(base_path)
        self._base_path = None
        self.base_path = os.path.abspath(base_path)
        logger.debug("Storage class initialized!")
        self._paths_to_remove_before_sm_save = {}

    def mark_path_for_removal_for_sm_id(self, state_machine_id, path):
        """
        Marks a path for removal. The path is removed if the state machine of the specified state machine id is saved.
        :param state_machine_id: the state machine id the path belongs to
        :param path: the path to a state that was deleted
        :return:
        """
        if state_machine_id not in self._paths_to_remove_before_sm_save.iterkeys():
            self._paths_to_remove_before_sm_save[state_machine_id] = []
            logger.debug("path %s for state machine id %s is going to be deleted during saving of the state machine" %
                         (str(path), str(state_machine_id)))
        self._paths_to_remove_before_sm_save[state_machine_id].append(path)

    def unmark_path_for_removal_for_sm_id(self, state_machine_id, path):
        """
        Unmarks a path for removal.
        :param state_machine_id: the state machine id the path belongs to
        :param path: the path to a state that should not be removed
        :return:
        """
        if state_machine_id in self._paths_to_remove_before_sm_save.iterkeys():
            if path in self._paths_to_remove_before_sm_save[state_machine_id]:
                self._paths_to_remove_before_sm_save[state_machine_id].remove(path)

    def save_statemachine_as_yaml(self, statemachine, base_path, version=None, delete_old_state_machine=False):
        """
        Saves a root state to a yaml file.
        :param statemachine: the statemachine to be saved
        :param base_path: base_path to which all further relative paths refers to
        :param version: the version of the statemachine to save
        :return:
        """
        if base_path is not None:
            self.base_path = base_path

        # remove all paths that were marked to be removed
        if statemachine.state_machine_id in self._paths_to_remove_before_sm_save.iterkeys():
            for path in self._paths_to_remove_before_sm_save[statemachine.state_machine_id]:
                if self.storage_utils.exists_path(path):
                    self.storage_utils.remove_path(path)

        root_state = statemachine.root_state
        # clean old path first
        if self.storage_utils.exists_path(self.base_path):
            if delete_old_state_machine:
                self.storage_utils.remove_path(self.base_path)
        if not self.storage_utils.exists_path(self.base_path):
            self.storage_utils.create_path(self.base_path)
        f = open(os.path.join(self.base_path, self.STATEMACHINE_FILE), 'w')
        last_update = strftime("%Y-%m-%d %H:%M:%S", gmtime())
        creation_time = last_update
        if hasattr(statemachine, 'creation_time'):
            creation_time = statemachine.creation_time
        statemachine_content = "root_state: %s\nversion: %s\ncreation_time: %s\nlast_update: %s"\
                               % (root_state.state_id, version, creation_time, last_update)
        yaml.dump(yaml.load(statemachine_content), f, indent=4)
        f.close()
        # add root state recursively
        self.save_state_recursively(root_state, "")
        if statemachine.marked_dirty:
            statemachine.marked_dirty = False
        statemachine.base_path = self.base_path
        logger.debug("Successfully saved statemachine!")

    def save_script_file_for_state_and_source_path(self, state, state_path):
        """
        Saves the script file for a state to the directory of the state. The script name will be set to the SCRIPT_FILE
        constant.
        :param state: The state of which the script file should be saved
        :param state_path: The path of the state meta file
        :return:
        """
        state_path_full = os.path.join(self.base_path, state_path)
        source_script_file = os.path.join(state.script.path, state.script.filename)
        destination_script_file = os.path.join(state_path_full, self.SCRIPT_FILE)
        if not source_script_file == destination_script_file:
            shutil.copyfile(source_script_file,
                            destination_script_file)
            state.script.path = state_path
            state.script.filename = self.SCRIPT_FILE
        else:  # load text into script file
            script_file = open(source_script_file, 'w')
            script_file.write(state.script.script)
            script_file.close()

    @staticmethod
    def save_script_file(state):
        script_file_path = os.path.join(state.script.path, state.script.filename)
        script_file = open(script_file_path, 'w')
        script_file.write(state.script.script)
        script_file.close()

    def save_state_recursively(self, state, parent_path):
        """
        Recursively saves a state to a yaml file. It calls this method on all its substates.
        :param state:
        :param parent_path:
        :return:
        """
        state_path = os.path.join(parent_path, str(state.state_id))
        state_path_full = os.path.join(self.base_path, state_path)
        self.storage_utils.create_path(state_path_full)
        self.save_script_file_for_state_and_source_path(state, state_path)
        self.storage_utils.save_object_to_yaml_abs(state, os.path.join(state_path_full, self.META_FILE))
        state.script.path = state_path_full
        state.script.filename = self.SCRIPT_FILE

        #create yaml files for all children
        if hasattr(state, 'states'):
            for key, state in state.states.iteritems():
                self.save_state_recursively(state, state_path)

    def clean_transitions_of_sm(self, root_state):
        affected_sm = False
        if hasattr(root_state, "states"):
            for t_id, transition in root_state.transitions.iteritems():
                if transition.to_state is None:
                    affected_sm = True
                    transition.to_state = root_state.state_id
            for s_id, state in root_state.states.iteritems():
                if hasattr(state, "states"):
                    affected_sm |= self.clean_transitions_of_sm(state)
        return affected_sm

    def load_statemachine_from_yaml(self, base_path=None):
        """
        Loads a state machine from a given path. If no path is specified the state machine is tried to be loaded
        from the base path.
        :param base_path: An optional base path for the state machine.
        :return: a tuple of the loaded container state, the version of the state and the creation time
        """
        if base_path is not None:
            self.base_path = base_path
        logger.debug("Load state machine from path %s" % str(base_path))
        try:
            stream = file(os.path.join(self.base_path, self.STATEMACHINE_FILE), 'r')
        except IOError:
            import sys
            exc = AttributeError("Provided path doesn't contain a valid state-machine: {0}".format(base_path))
            raise AttributeError, exc, sys.exc_info()[2]
        tmp_dict = yaml.load(stream)
        root_state_id = tmp_dict['root_state']
        version = tmp_dict['version']
        creation_time = tmp_dict['creation_time']
        if 'last_update' not in tmp_dict:
            last_update = creation_time
        else:
            last_update = tmp_dict['last_update']
        tmp_base_path = os.path.join(self.base_path, root_state_id)
        logger.debug("Loading root state from path %s" % tmp_base_path)
        sm = StateMachine()
        sm.version = version
        sm.creation_time = creation_time
        sm.last_update = last_update
        sm.base_path = base_path
        sm.root_state = self.storage_utils.load_object_from_yaml_abs(os.path.join(tmp_base_path, self.META_FILE))
        # set path after loading the state, as the yaml parser did not know the path during state creation
        sm.root_state.script.path = tmp_base_path
        self.load_script_file(sm.root_state)
        for p in os.listdir(tmp_base_path):
            if os.path.isdir(os.path.join(tmp_base_path, p)):
                elem = os.path.join(tmp_base_path, p)
                logger.debug("Going down the statemachine hierarchy recursively to state %s" % str(elem))
                self.load_state_recursively(sm.root_state, elem)
                logger.debug("Going up back to state %s" % str(tmp_base_path))

        sm.base_path = self.base_path
        sm.marked_dirty = False

        # this is a backward compatibility function to ensure that old libraries are still working
        if self.clean_transitions_of_sm(sm.root_state):
            self.save_statemachine_as_yaml(sm, base_path)

        return [sm, version, creation_time]

    def load_state_from_yaml(self, state_path):
        """
        Loads a state from a given path
        from the base path.
        :param state_path: The path of the state on the file system.
        :return: the loaded state
        """
        root_state = self.storage_utils.load_object_from_yaml_abs(os.path.join(state_path, self.META_FILE))
        # set path after loading the state, as the yaml parser did not know the path during state creation
        root_state.script.path = state_path
        # load_and_build the module to load the correct content into root_state.script.script
        self.load_script_file(root_state)
        for p in os.listdir(state_path):
            if os.path.isdir(os.path.join(state_path, p)):
                elem = os.path.join(state_path, p)
                logger.debug("Going down the statemachine hierarchy recursively to state %s" % str(elem))
                self.load_state_recursively(root_state, elem)
                logger.debug("Going up back to state %s" % str(state_path))
        return root_state

    def load_state_recursively(self, parent_state, state_path=None):
        """
        Recursively loads the state. It calls this method on each sub-state of a container state.
        :param parent_state:  the root state of the last load call to which the loaded state will be added
        :param state_path: the path on the filesystem where to find eht meta file for the state
        :return:
        """
        state = self.storage_utils.load_object_from_yaml_abs(os.path.join(state_path, self.META_FILE))
        state.script.path = state_path
        # connect the missing function_handlers for setting the outcome names
        state.connect_all_outcome_function_handles()
        parent_state.add_state(state, storage_load=True)
        # the library state sets his script file to the script file of the root state of its library, thus it should
        # not be overwritten in this case
        from awesome_tool.statemachine.states.library_state import LibraryState
        if not isinstance(state, LibraryState):
            self.load_script_file(state)
        for p in os.listdir(state_path):
            if os.path.isdir(os.path.join(state_path, p)):
                elem = os.path.join(state_path, p)
                self.load_state_recursively(state, elem)

    def load_script_file(self, state):
        script_file = open(os.path.join(state.script.path, self.SCRIPT_FILE), 'r')
        text = script_file.read()
        script_file.close()
        state.script.script = text

    #########################################################################
    # Properties for all class fields that must be observed by gtkmvc
    #########################################################################

    @property
    def base_path(self):
        """Property for the _base_path field

        """
        return self._base_path

    @base_path.setter
    @Observable.observed
    def base_path(self, base_path):
        if not isinstance(base_path, str):
            raise TypeError("base_path must be of type str")

        self._base_path = base_path
        self.storage_utils.base_path = base_path