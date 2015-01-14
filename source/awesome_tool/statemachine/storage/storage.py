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

    def save_file_as_yaml_rel(self, state, rel_path):
        f = open(os.path.join(self.base_path, rel_path), 'w')
        with Capturing() as output:
            yaml.dump(state, f, indent=4)
        #stream = yaml.dump(state, indent=4)
        #f.write(stream)
        f.close()

    def save_file_as_yaml_abs(self, state, abs_path):
        f = open(abs_path, 'w')
        with Capturing() as output:
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

    def save_statemachine_as_yaml(self, root_state, base_path=None, version=None):
        if not base_path is None:
            self.base_path = base_path
        # clean old path first
        if self._exists_path(self.base_path):
            self._remove_path(self.base_path)
        self._create_path(self.base_path)
        f = open(os.path.join(self.base_path, self.STATEMACHINE_FILE), 'w')
        statemachine_content = "creation_time: %s\nroot_state: %s\nstatemachine_name: %s\nversion: %s"\
                               % (strftime("%Y-%m-%d %H:%M:%S", gmtime()), root_state.state_id, root_state.name,
                                  str(version))
        yaml.dump(yaml.load(statemachine_content), f, indent=4)
        f.close()
        # add root state recursively
        self.save_state_recursively(root_state, self.base_path)
        logger.debug("Successfully saved statemachine!")

    def save_script_file_for_state(self, state, path):
        shutil.copyfile(os.path.join(state.script.path, state.script.filename), os.path.join(path, self.SCRIPT_FILE))
        state.script.path = path
        state.script.filename = self.SCRIPT_FILE

    def save_state_recursively(self, root_state, parent_path=None):
        #print "Save state %s recursively" % str(root_state.state_id)
        state_path = os.path.join(parent_path, str(root_state.state_id))
        self._create_path(state_path)
        self.save_file_as_yaml_abs(root_state, os.path.join(state_path, self.META_FILE))
        self.save_script_file_for_state(root_state, state_path)

        #create yaml files for all children
        if not root_state.state_type is statemachine.states.state.StateType.EXECUTION:
            #print "length of root state: %s" % len(root_state.states)
            for key, state in root_state.states.iteritems():
                #print "state_path: %s" % str(state_path)
                self.save_state_recursively(state, state_path)

    def load_statemachine_from_yaml(self, base_path=None):
        if not base_path is None:
            self.base_path = base_path
        #print "Load state machine from path %s" % self.base_path
        stream = file(os.path.join(self.base_path, self.STATEMACHINE_FILE), 'r')
        tmp_dict = yaml.load(stream)
        root_state_id = tmp_dict['root_state']
        tmp_base_path = os.path.join(self.base_path, root_state_id)
        root_state = self.load_file_from_yaml_abs(os.path.join(tmp_base_path, self.META_FILE))
        for p in os.listdir(tmp_base_path):
            if os.path.isdir(os.path.join(tmp_base_path, p)):
                elem = os.path.join(tmp_base_path, p)
                self.load_state_recursively(root_state, elem)

        return root_state

    def load_state_recursively(self, root_state, parent_path=None):
        #print "Path of next state to add: %s" % base_path
        state = self.load_file_from_yaml_abs(os.path.join(parent_path, self.META_FILE))
        root_state.add_state(state)
        for p in os.listdir(parent_path):
            if os.path.isdir(os.path.join(parent_path, p)):
                elem = os.path.join(parent_path, p)
                self.load_state_recursively(root_state, elem)

    def store_dict(self, rel_path, tmp_dict):
        f = open(os.path.join(self.base_path, rel_path), 'w')
        json.dump(tmp_dict, f, indent=4)
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

    #######################################################
    # Functions that are not needed right now
    #######################################################
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

    def remove(self, path):
        """ Remove the state machine at the given path
        """
        for p in os.listdir(path):
            elem = os.path.join(path, p)
            if os.path.isdir(elem):
                shutil.rmtree(elem)
            else:
                os.remove(elem)