"""
.. module:: script
   :platform: Unix, Windows
   :synopsis: A module to represent the script file for each state in a state machine

.. moduleauthor:: Sebastian Brunner
"""

import os
import imp
import yaml
from gtkmvc import Observable

from rafcon.statemachine.enums import DEFAULT_SCRIPT_PATH
from rafcon.statemachine.id_generator import *
import rafcon.statemachine.singleton

from rafcon.utils import filesystem
from rafcon.utils import log
logger = log.get_logger(__name__)


DEFAULT_SCRIPT_FILE = "default_script.py"

DEFAULT_SCRIPT = filesystem.read_file(os.path.dirname(__file__), DEFAULT_SCRIPT_FILE)


class Script(Observable, yaml.YAMLObject):
    """A class for representing the script file for each state in a state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar path: the path where the script resides
    :ivar filename: the full name of the script file
    :ivar _compiled_module: the compiled module
    :ivar _script_id: the id of the script
    :ivar check_path: a flag to indicate if the path should be checked for existence

    """

    yaml_tag = u'!Script'

    def __init__(self, path=None, filename=None, check_path=True, state=None):

        Observable.__init__(self)

        self._path = path
        self._filename = filename
        self._compiled_module = None
        self._script_id = generate_script_id()
        self._state = state
        self.script = DEFAULT_SCRIPT

        if path is None:
            self._path = os.path.join(DEFAULT_SCRIPT_PATH, state.get_path())
            if not os.path.exists(self._path):
                os.makedirs(self._path)
            if not filename:
                self._filename = "tmp_script.file"
            filesystem.write_file(os.path.join(self._path, self._filename), self.script)

        if check_path:
            if not os.path.exists(self._path):
                raise RuntimeError("Script path '{}' does not exist".format(self._path))
            if not os.path.exists(os.path.join(self._path, self._filename)):
                raise RuntimeError("Script '{}' does not exist".format(os.path.join(self._path, self._filename)))

            # load and build the module per default else the default scripts will be loaded in self.script
            self._load_script()
            self.build_module()

    def reload_path(self, filename=None):
        self._path = self._state.get_file_system_path()
        if filename:
            self._filename = filename

    def execute(self, state, inputs=None, outputs=None, backward_execution=False):
        """Execute the user 'execute' function specified in the script

        :param ExecutionState state: the state belonging to the execute function, refers to 'self'
        :param dict inputs: the input data of the script
        :param dict outputs: the output data of the script
        :param bool backward_execution: Flag whether to run the script in backwards mode
        :return: Return value of the execute script
        :rtype: str | int
        """
        if not outputs:
            outputs = {}
        if not inputs:
            inputs = {}
        if backward_execution:
            if hasattr(self._compiled_module, "backward_execute"):
                return self._compiled_module.backward_execute(
                    state, inputs, outputs, rafcon.statemachine.singleton.global_variable_manager
                )
            else:
                logger.debug("No backward execution method found for state %s" % state.name)
                return None
        else:
            return self._compiled_module.execute(state, inputs, outputs,
                                                 rafcon.statemachine.singleton.global_variable_manager)

    def _load_script(self):
        """Loads the script from the filesystem
        """
        script_text = filesystem.read_file(self._path, self._filename)

        if not script_text:
            raise IOError("Script file could not be opened or was empty: {0}".format(os.path.join(self._path,
                                                                                                  self._filename)))
        self.script = script_text

    def build_module(self):
        """Builds a temporary module from the script file
        """
        try:
            imp.acquire_lock()
            module_name = os.path.splitext(self._filename)[0] + str(self._script_id)

            # load module
            tmp_module = imp.new_module(module_name)

            code = compile(self.script, '%s (%s)' % (self._filename, self._script_id), 'exec')

            try:
                exec code in tmp_module.__dict__
            except RuntimeError, e:
                raise IOError("The compilation of the script module failed - error message: %s" % str(e))

            # return the module
            self.compiled_module = tmp_module
        finally:
            imp.release_lock()

    @classmethod
    def to_yaml(cls, dumper, data):
        #TODO:implement
        dict_representation={}
        node = dumper.represent_mapping(u'!Script', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        #TODO:implement
        return None

#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @property
    def filename(self):
        """Property for the _filename field

        """
        return self._filename

    @property
    def compiled_module(self):
        """Return the compiled module
        """
        return self._compiled_module

    # this setter should actually never be called as the module will be compiled by the build_module() function
    @compiled_module.setter
    @Observable.observed
    def compiled_module(self, compiled_module):
        self._compiled_module = compiled_module
