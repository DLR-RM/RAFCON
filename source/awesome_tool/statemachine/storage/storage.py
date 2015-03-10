"""
.. module:: storage
   :platform: Unix, Windows
   :synopsis: A module to store a statemachine on disk

.. moduleauthor:: Sebastian Brunner


"""

import os
import shutil
import json
from time import gmtime, strftime

import yaml
from gtkmvc import Observable

import awesome_tool.statemachine.states.state
from awesome_tool.statemachine.state_machine import StateMachine
from awesome_tool.statemachine.enums import StateType
from awesome_tool.utils import log
logger = log.get_logger(__name__)


class Storage(Observable):

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

    def __init__(self, base_path):
        Observable.__init__(self)
        self._base_path = None
        self.base_path = os.path.abspath(base_path)
        logger.debug("Storage class initialized!")
        self._paths_to_remove_before_sm_save = {}
        self.ids_of_modified_state_machines = []

    def reset_dirty_flags(self):
        """
        Resets the dirty flags of all state machines.
        :return:
        """
        self.ids_of_modified_state_machines = []

    def mark_dirty(self, sm_id):
        """
        Marks the sate machine as dirty i.e. the state machine is changed since the last saving
        :param sm_id: the state machine id of the state machine to mark as dirty
        :return:
        """
        if sm_id not in self.ids_of_modified_state_machines:
            logger.debug("State machine with state machine id %s was marked as dirty!" % str(sm_id))
            self.ids_of_modified_state_machines.append(sm_id)

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

    def save_object_to_yaml_rel(self, object, rel_path):
        """
        Saves an object, which inherits from yaml.YAMLObject, to yaml file
        :param object: the object to be saved
        :param rel_path: the relative path of the target yaml file
        :return:
        """
        f = open(os.path.join(self.base_path, rel_path), 'w')
        yaml.dump(object, f, indent=4)
        f.close()

    def save_object_to_yaml_abs(self, object, abs_path):
        """
        Saves an object, which inherits from yaml.YAMLObject, to yaml file
        :param object: the object to be saved
        :param abs_path: the absolute path of the target yaml file
        :return:
        """
        f = open(abs_path, 'w')
        yaml.dump(object, f, indent=4)
        f.close()

    def load_object_from_yaml_rel(self, rel_path):
        """
        Loads an object, which inherits from yaml.YAMLObject, from a yaml file
        :param object: the object to be saved
        :param abs_path: the relative path of the target yaml file
        :return:
        """
        stream = file(os.path.join(self.base_path, rel_path), 'r')
        state = yaml.load(stream)
        return state

    def load_object_from_yaml_abs(self, abs_path):
        """
        Loads an object, which inherits from yaml.YAMLObject, from a yaml file
        :param object: the object to be saved
        :param abs_path: the absolute path of the target yaml file
        :return:
        """
        stream = file(abs_path, 'r')
        state = yaml.load(stream)
        return state

    def write_dict_to_yaml(self, dict_to_write, path):
        """
        Writes a dictionary to a yaml file
        :param dict_to_write:  the dictionary to be writte
        :param path: the absolute path of the target yaml file
        :return:
        """
        f = open(path, 'w')
        yaml.dump(dict_to_write, f, indent=4)
        f.close()

    def load_dict_from_yaml(self, path):
        """
        Loads a dictionary from a yaml file
        :param path: the absolute path of the target yaml file
        :return:
        """
        stream = file(path, 'r')
        yaml_object = yaml.load(stream)
        return yaml_object

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
                self.remove_path(path)

        root_state = statemachine.root_state
        # clean old path first
        if self._exists_path(self.base_path):
            if delete_old_state_machine:
                self.remove_path(self.base_path)
        if not self._exists_path(self.base_path):
            self._create_path(self.base_path)
        f = open(os.path.join(self.base_path, self.STATEMACHINE_FILE), 'w')
        statemachine_content = "root_state: %s\nversion: %s\ncreation_time: %s"\
                               % (root_state.state_id, version, strftime("%Y-%m-%d %H:%M:%S", gmtime()))
        yaml.dump(yaml.load(statemachine_content), f, indent=4)
        f.close()
        # add root state recursively
        self.save_state_recursively(root_state, "")
        if statemachine.state_machine_id in self.ids_of_modified_state_machines:
            self.ids_of_modified_state_machines.remove(statemachine.state_machine_id)
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

    def save_script_file(self, state):
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
        self._create_path(state_path_full)
        self.save_script_file_for_state_and_source_path(state, state_path)
        self.save_object_to_yaml_abs(state, os.path.join(state_path_full, self.META_FILE))
        state.script.path = state_path_full
        state.script.filename = self.SCRIPT_FILE

        #create yaml files for all children
        if not state.state_type is awesome_tool.statemachine.states.state.StateType.EXECUTION:
            for key, state in state.states.iteritems():
                self.save_state_recursively(state, state_path)

    def load_statemachine_from_yaml(self, base_path=None):
        """
        Loads a state machine from a given path. If no path is specified the state machine is tried to be loaded
        from the base path.
        :param base_path: An optional base path for the state machine.
        :return: a tuple of: the loaded container state, the version of the state and the creation time of when the
                state was saved
        """
        if base_path is not None:
            self.base_path = base_path
        logger.debug("Load state machine from path %s" % str(base_path))
        stream = file(os.path.join(self.base_path, self.STATEMACHINE_FILE), 'r')
        tmp_dict = yaml.load(stream)
        root_state_id = tmp_dict['root_state']
        version = tmp_dict['version']
        creation_time = tmp_dict['creation_time']
        tmp_base_path = os.path.join(self.base_path, root_state_id)
        logger.debug("Loading root state from path %s" % tmp_base_path)
        sm = StateMachine()
        sm.base_path = base_path
        sm.root_state = self.load_object_from_yaml_abs(os.path.join(tmp_base_path, self.META_FILE))
        # set path after loading the state, as the yaml parser did not know the path during state creation
        sm.root_state.script.path = tmp_base_path
        # load_and_build the module to load the correct content into root_state.script.script
        sm.root_state.script.load_and_build_module()
        self.load_script_file(sm.root_state)
        for p in os.listdir(tmp_base_path):
            if os.path.isdir(os.path.join(tmp_base_path, p)):
                elem = os.path.join(tmp_base_path, p)
                logger.debug("Going down the statemachine hierarchy recursively to state %s" % str(elem))
                self.load_state_recursively(sm.root_state, elem)
                logger.debug("Going up back to state %s" % str(tmp_base_path))

        sm.base_path = self.base_path
        return [sm, version, creation_time]

    def load_state_recursively(self, root_state, state_path=None):
        """
        Recursively loads the state. It calls this method on each substate of a container state.
        :param root_state:  the root state of the last load call to which the loaded state will be added
        :param state_path: the path on the filesystem where to find eht meta file for the state
        :return:
        """
        state = self.load_object_from_yaml_abs(os.path.join(state_path, self.META_FILE))
        state.script.path = state_path
        # connect the missing function_handlers for setting the outcome names
        state.connect_all_outcome_function_handles()
        root_state.add_state(state)
        # the library state sets his script file to the script file of the root state of its library, thus it should
        # not be overwritten in this case
        if state.state_type is not StateType.LIBRARY:
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

    def write_dict_to_json(self, rel_path, tmp_dict):
        """
        Write a dictionary to a json file.
        :param rel_path: The relative path to save the dictionary to
        :param tmp_dict: The dictionary to get saved
        :return:
        """
        f = open(os.path.join(self.base_path, rel_path), 'w')
        json.dump(tmp_dict, f, indent=4)
        f.close()

    def load_dict_from_json(self, rel_path):
        """
        Loads a dictionary from a json file.
        :param rel_path: The relative path of the json file.
        :return: The dictionary specified in the json file
        """
        f = open(os.path.join(self.base_path, rel_path), 'r')
        result = json.load(f)
        f.close()
        return result

    def _create_path(self, path):
        """ Creats a absolute path in the file system.

        :param path: The path to be created
        :return:
        """
        if not self._exists_path(path):
            os.makedirs(path)

    def remove_path(self, path):
        """ Removes an absolute path in the file system

        :param path: The path to be removed
        :return:
        """
        shutil.rmtree(path)

    def _remove_file(self, path):
        """
        Removes a file in the file system.
        :param path: The absolute path of the file to be removed.
        :return:
        """
        os.remove(path)

    def _exists_path(self, path):
        """
        Checks if a certain path exists in the file system.
        :param path: The path to be checked
        :return:
        """
        return os.path.exists(path)

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