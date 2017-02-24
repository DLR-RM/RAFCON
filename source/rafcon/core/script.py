import os
import imp
import yaml
from gtkmvc import Observable

from rafcon.core.id_generator import generate_script_id
import rafcon.core.singleton

from rafcon.utils import filesystem
from rafcon.core.storage.storage import SCRIPT_FILE
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

    def __init__(self, path=None, filename=None, check_path=True, parent=None):

        Observable.__init__(self)
        self._path = None
        self._filename = None
        self._compiled_module = None
        self._script_id = generate_script_id()
        self._parent = None
        self._check_path = check_path

        self.script = DEFAULT_SCRIPT
        self.filename = filename
        self.path = path
        self.parent = parent

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
                    state, inputs, outputs, rafcon.core.singleton.global_variable_manager
                )
            else:
                logger.debug("No backward execution method found for state %s" % state.name)
                return None
        else:
            return self._compiled_module.execute(state, inputs, outputs,
                                                 rafcon.core.singleton.global_variable_manager)

    def _load_script(self):
        """Loads the script from the filesystem

        :raises exceptions.IOError: if the script file could not be opened
        """
        script_text = filesystem.read_file(self.path, self.filename)

        if not script_text:
            raise IOError("Script file could not be opened or was empty: {0}".format(os.path.join(self.path,
                                                                                                  self.filename)))
        self.script = script_text

    def build_module(self):
        """Builds a temporary module from the script file

        :raises exceptions.IOError: if the compilation of the script module failed
        """
        try:
            imp.acquire_lock()
            module_name = os.path.splitext(self.filename)[0] + str(self._script_id)

            # load module
            tmp_module = imp.new_module(module_name)

            code = compile(self.script, '%s (%s)' % (self.filename, self._script_id), 'exec')

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
        dict_representation = {}
        node = dumper.represent_mapping(u'!Script', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        #TODO:implement
        return None

    @property
    def parent(self):
        """Property for the _parent field

        """
        return self._parent() if self._parent is not None else None

    @parent.setter
    def parent(self, value):
        from weakref import ref
        from rafcon.core.states.execution_state import ExecutionState
        if value is not None and not isinstance(value, ExecutionState):
            raise TypeError("The parent of a script has to be a ExecutionState or None.")
        self._parent = ref(value) if value is not None else None

    @property
    def filename(self):
        """Property for the _filename field

        """
        return self._filename if self._filename is not None else SCRIPT_FILE

    @filename.setter
    def filename(self, value):
        if value is not None and not isinstance(value, str):
            raise TypeError("The filename of a script has to be a string or None to use the default value.")
        self._filename = value

    @property
    def path(self):
        if self._path is not None or self.parent is None:
            return self._path
        else:
            return self.parent.get_file_system_path()

    @path.setter
    def path(self, value):
        if value is not None:
            if not isinstance(value, basestring):
                raise TypeError("The path of a script has to be a string or None to use the default value.")

            if self._check_path:
                if not os.path.exists(value):
                    raise RuntimeError("Script path '{}' does not exist".format(value))
                if not os.path.exists(os.path.join(value, self._filename)):
                    raise RuntimeError("Script '{}' does not exist".format(os.path.join(value, self._filename)))

                # load and build the module per default else the default scripts will be loaded in self.script
                old_path = self._path
                self._path = value
                try:
                    self._load_script()
                    self.build_module()
                except:
                    self._path = old_path
                    raise
        else:
            if self.parent is not None:
                value = self.parent.get_file_system_path()

        self._path = value

#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @property
    def compiled_module(self):
        """Return the compiled module
        """
        return self._compiled_module

    @compiled_module.setter
    @Observable.observed
    def compiled_module(self, compiled_module):
        self._compiled_module = compiled_module
