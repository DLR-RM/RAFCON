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

import statemachine.states.state
from utils import log

logger = log.get_logger(__name__)

##############################################################################
from cStringIO import StringIO
import sys

#hack to omit output of the yaml.dump function; did not find another way
#found at: http://stackoverflow.com/questions/16571150/how-to-capture-stdout-output-from-a-python-function-call
class Capturing(list):

    def __enter__(self):
        self._stdout = sys.stdout
        sys.stdout = self._stringio = StringIO()
        return self

    def __exit__(self, *args):
        self.extend(self._stringio.getvalue().splitlines())
        sys.stdout = self._stdout
##############################################################################


class Storage(Observable):

    """This class implements the Storage interface by using a file system on the disk.

    The data storage format is a directory system.

    :ivar base_path: the base path to resolve all relative paths
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

    def save_object_to_yaml_rel(self, state, rel_path):
        f = open(os.path.join(self.base_path, rel_path), 'w')
        with Capturing() as output:
            yaml.dump(state, f, indent=4)
        #stream = yaml.dump(state, indent=4)
        #f.write(stream)
        f.close()

    def save_object_to_yaml_abs(self, state, abs_path):
        f = open(abs_path, 'w')
        with Capturing() as output:
            yaml.dump(state, f, indent=4)
        f.close()

    def load_object_from_yaml_rel(self, rel_path):
        stream = file(os.path.join(self.base_path, rel_path), 'r')
        state = yaml.load(stream)
        return state

    def write_dict_to_yaml(self, dict_to_write, path):
        f = open(path, 'w')
        yaml.dump(dict_to_write, f, indent=4)
        f.close()

    def load_dict_from_yaml(self, path):
        stream = file(path, 'r')
        yaml_object = yaml.load(stream)
        return yaml_object

    def load_object_from_yaml_abs(self, abs_path):
        stream = file(abs_path, 'r')
        state = yaml.load(stream)
        return state

    def save_statemachine_as_yaml(self, root_state, base_path=None, version=None):
        if base_path is not None:
            self.base_path = base_path
        # clean old path first
        if self._exists_path(self.base_path):
            self._remove_path(self.base_path)
        if not self._exists_path(self.base_path):
            self._create_path(self.base_path)
        f = open(os.path.join(self.base_path, self.STATEMACHINE_FILE), 'w')
        statemachine_content = "root_state: %s\nversion: %s\ncreation_time: %s"\
                               % (root_state.state_id, version, strftime("%Y-%m-%d %H:%M:%S", gmtime()))
        yaml.dump(yaml.load(statemachine_content), f, indent=4)
        f.close()
        # add root state recursively
        self.save_state_recursively(root_state, "")
        logger.debug("Successfully saved statemachine!")

    def save_script_file_for_state(self, state, state_path):
        state_path_full = os.path.join(self.base_path, state_path)
        shutil.copyfile(os.path.join(state.script.path, state.script.filename),
                        os.path.join(state_path_full, self.SCRIPT_FILE))
        state.script.path = state_path
        state.script.filename = self.SCRIPT_FILE

    def save_state_recursively(self, state, parent_path):
        state_path = os.path.join(parent_path, str(state.state_id))
        state_path_full = os.path.join(self.base_path, state_path)
        self._create_path(state_path_full)
        self.save_script_file_for_state(state, state_path)
        self.save_object_to_yaml_abs(state, os.path.join(state_path_full, self.META_FILE))
        state.script.path = state_path_full
        state.script.filename = self.SCRIPT_FILE

        #create yaml files for all children
        if not state.state_type is statemachine.states.state.StateType.EXECUTION:
            for key, state in state.states.iteritems():
                self.save_state_recursively(state, state_path)

    def load_statemachine_from_yaml(self, base_path=None):
        if not base_path is None:
            self.base_path = base_path
        logger.debug("Load state machine from path %s" % str(base_path))
        stream = file(os.path.join(self.base_path, self.STATEMACHINE_FILE), 'r')
        tmp_dict = yaml.load(stream)
        root_state_id = tmp_dict['root_state']
        version = tmp_dict['version']
        creation_time = tmp_dict['creation_time']
        tmp_base_path = os.path.join(self.base_path, root_state_id)
        logger.debug("Loading root state from path %s" % tmp_base_path)
        root_state = self.load_object_from_yaml_abs(os.path.join(tmp_base_path, self.META_FILE))
        # set path after loading the state, as the yaml parser did not know the path during state creation
        root_state.script.path = tmp_base_path
        # load_and_build the module to load the correct content into root_state.script.script
        root_state.script.load_and_build_module()
        for p in os.listdir(tmp_base_path):
            if os.path.isdir(os.path.join(tmp_base_path, p)):
                elem = os.path.join(tmp_base_path, p)
                logger.debug("Going down the statemachine hierarchy recursively to state %s" % str(elem))
                self.load_state_recursively(root_state, elem)
                logger.debug("Going up back to state %s" % str(tmp_base_path))

        return [root_state, version, creation_time]

    def load_state_recursively(self, root_state, state_path=None):
        state = self.load_object_from_yaml_abs(os.path.join(state_path, self.META_FILE))
        state.script.path = state_path
        # connect the missing function_handlers for setting the outcome names
        state.connect_all_outcome_function_handles()
        root_state.add_state(state)
        for p in os.listdir(state_path):
            if os.path.isdir(os.path.join(state_path, p)):
                elem = os.path.join(state_path, p)
                self.load_state_recursively(state, elem)

    def write_dict_to_json(self, rel_path, tmp_dict):
        f = open(os.path.join(self.base_path, rel_path), 'w')
        json.dump(tmp_dict, f, indent=4)
        f.close()

    def load_dict_from_json(self, rel_path):
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