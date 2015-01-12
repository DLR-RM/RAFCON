"""
.. module:: storage
   :platform: Unix, Windows
   :synopsis: A module to store a statemachine on disk

.. moduleauthor:: Sebastian Brunner


"""

import os
import shutil
import json
import yaml
from time import gmtime, strftime

from statemachine.states.state import State, StateType, DataPort


class Storage():

    """This class implements the Storage interface by using a file system on the disk.

    The data storage format is a directory system and the history is currently not implemented..
    """

    GRAPHICS_FILE = 'gui_gtk.yaml'
    META_FILE = 'meta.yaml'
    SCRIPT_FILE = 'script.py'

    STATEMACHINE_FILE = 'statemachine.txt'
    #LIB_FILE = 'lib.txt'

    def __init__(self, base_path):
        self.base_path = os.path.abspath(base_path)
        print "Storage class initialized!"

    def save_file_as_yaml(self, state, rel_path):
        f = open(os.path.join(self.base_path, rel_path), 'w')
        yaml.dump(state, f, indent=4)
        f.close()

    def load_file_from_yaml_rel(self, rel_path):
        stream = file(os.path.join(self.base_path, rel_path), 'r')
        state = yaml.load(stream)
        return state

    def load_file_from_yaml_abs(self, abs_path):
        stream = file(abs_path, 'r')
        state = yaml.load(stream)
        return state

    def save_statemachine_as_yaml(self, root_state):
        # clean old path first
        if self._exists_path(self.base_path):
            self._remove_path(self.base_path)
        self._create_path(self.base_path)
        f = open(os.path.join(self.base_path, self.STATEMACHINE_FILE), 'w')
        statemachine_content = "creation_time: %s\nroot_state: %s" % (strftime("%Y-%m-%d %H:%M:%S", gmtime()),
                                                                      root_state.state_id)
        yaml.dump(yaml.load(statemachine_content), f, indent=4)
        f.close()
        # add root state recursively
        self.save_state_recursively(root_state, self.base_path)

    def save_state_recursively(self, root_state, base_path=None):
        #print "Save state %s recursively" % str(root_state.state_id)
        tmp_base_path = os.path.join(base_path, str(root_state.state_id))

        self._create_path(tmp_base_path)
        self.save_file_as_yaml(root_state, os.path.join(tmp_base_path, self.META_FILE))

        #create yaml files for all children
        if not root_state.state_type is StateType.EXECUTION:
            #print "length of root state: %s" % len(root_state.states)
            for key, state in root_state.states.iteritems():
                #print "tmp_base_path: %s" % str(tmp_base_path)
                self.save_state_recursively(state, tmp_base_path)

    def load_statemachine_from_yaml(self):
        print "Load state machine from path %s" % self.base_path
        stream = file(os.path.join(self.base_path, self.STATEMACHINE_FILE), 'r')
        dict = yaml.load(stream)
        root_state_id = dict['root_state']
        tmp_base_path = os.path.join(self.base_path, root_state_id)
        root_state = self.load_file_from_yaml_rel(os.path.join(tmp_base_path, self.META_FILE))
        for p in os.listdir(tmp_base_path):
            if os.path.isdir(os.path.join(tmp_base_path, p)):
                elem = os.path.join(tmp_base_path, p)
                self.load_state_recursively(root_state, elem)

        return root_state

    def load_state_recursively(self, root_state, base_path=None):
        #print "Path of next state to add: %s" % base_path
        state = self.load_file_from_yaml_abs(os.path.join(base_path, self.META_FILE))
        root_state.add_state(state)
        for p in os.listdir(base_path):
            if os.path.isdir(os.path.join(base_path, p)):
                elem = os.path.join(base_path, p)
                self.load_state_recursively(root_state, elem)

    def store_dict(self, rel_path, dict):
        f = open(os.path.join(self.base_path, rel_path), 'w')
        json.dump(dict, f, indent=4)
        f.close()

    def load_dict(self, rel_path):
        f = open(os.path.join(self.base_path, rel_path), 'r')
        result = json.load(f)
        f.close()
        return result

    def _create_path(self, path):
        os.makedirs(path)

    def _remove_path(self, path):
        shutil.rmtree(path)

    def _remove_file(self, path):
        os.remove(path)

    def _exists_path(self, path):
        return os.path.exists(path)

    def is_statemachine_path(self, path=None):
        if not path:
            path = self.base_path
        return os.path.exists(os.path.join(path, self.STATEMACHINE_FILE))

    def get_statemachine_root(self, path):
        """Walk up the path until a statemachine file was found

        """
        while not self.is_statemachine_path(path):
            path_components = os.path.split(path)
            if len(path_components[1]) == 0:
                return None
            path = path_components[0]
        return path


    def get_path(self, state):
        """Returns the relative path for this state

        """
        path = ''
        current_state = state
        while current_state:
            path = os.path.join(self.get_state_dir_name(current_state), path)
            current_state = current_state.metaparent

        return os.path.join(self.base_path, path)

    #===========================================================
    def remove(self, path):
        """ Remove the state machine at the given path
        """
        for p in os.listdir(path):
            elem = os.path.join(path, p)
            if os.path.isdir(elem):
                shutil.rmtree(elem)
            else:
                os.remove(elem)