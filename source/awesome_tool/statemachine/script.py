"""
.. module:: script
   :platform: Unix, Windows
   :synopsis: A module to represent the script file for each state in a state machine

.. moduleauthor:: Sebastian Brunner


"""

import os
import imp
import sys
import yaml
from gtkmvc import Observable
from enum import Enum

from statemachine.id_generator import *
import statemachine.singleton


ScriptType = Enum('SCRIPT_TYPE', 'EXECUTION CONTAINER')

class Script(Observable, yaml.YAMLObject):

    """A class for representing the script file for each state in a state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar path: the path where the script resides
    :ivar filename: the full name of the script file
    :ivar _compiled_module: the compiled module
    :ivar _script_id: the id of the script
    :ivar script_type: the type of the script i.e. Execution or Container
    :ivar check_path: a flag to indicate if the path should be checked for existence

    """

    DEFAULT_SCRIPT_EXECUTION = """

def execute(self, inputs, outputs, external_modules, gvm):
    print "Hello world"
    return 0

"""
    DEFAULT_SCRIPT_CONTAINER = """

def enter(self, scoped_variables, external_modules, gvm):
    pass

def exit(self, scoped_variables, external_modules, gvm):
    pass

"""

    yaml_tag = u'!Script'

    def __init__(self, path=None, filename=None, script_type=None, check_path=True):

        Observable.__init__(self)

        self._path = None
        self.path = path
        self._filename = None
        self.filename = filename
        self._compiled_module = None
        self._script_id = generate_script_id()
        if script_type == ScriptType.EXECUTION:
            self.script = Script.DEFAULT_SCRIPT_EXECUTION
        else:
            self.script = Script.DEFAULT_SCRIPT_CONTAINER
        if path is None:
            self.path = "/tmp/DFC"
            if not os.path.exists(self.path):
                os.makedirs(self.path)
            self.filename = "Script_%s.file" % str(self._script_id)
            script_file = open(os.path.join(self.path, self.filename), "w")
            script_file.write(self.script)
            script_file.close()

        if check_path:
            if not os.path.exists(self.path):
                raise RuntimeError("Path %s does not exist" % self.path)
            else:
                if not os.path.exists(os.path.join(self.path, self.filename)):
                    raise RuntimeError("Path %s does not exist" % os.path.join(self.path, self.filename))

            # load and build the module per default else the default scripts will be loaded in self.script
            self.load_and_build_module()


    def execute(self, state, inputs={}, outputs={}):
        """
        Execute the custom "execute" function specified in the script.
        :param state: the state to which runs the execute function
        :param inputs: the input data of the script
        :param outputs: the output data of the script
        :return:
        """
        return self._compiled_module.execute(state, inputs, outputs,
                                             statemachine.singleton.external_module_manager.external_modules,
                                             statemachine.singleton.global_variable_manager)

    def enter(self, state, scoped_variables={}):
        """
        Execute the custom "enter" function specified in the script.
        :param state: the state to which runs the execute function
        :param inputs: the input data of the script
        :param outputs: the output data of the script
        :return:
        """
        return self._compiled_module.enter(state, scoped_variables,
                                           statemachine.singleton.external_module_manager.external_modules,
                                           statemachine.singleton.global_variable_manager)

    def exit(self, state, scoped_variables={}):
        """
        Execute the custom "exit" function specified in the script.
        :param state: the state to which runs the execute function
        :param inputs: the input data of the script
        :param outputs: the output data of the script
        :return:
        """
        return self._compiled_module.exit(state, scoped_variables,
                                          statemachine.singleton.external_module_manager.external_modules,
                                          statemachine.singleton.global_variable_manager)

    def load_and_build_module(self):
        """Loads and builds the module given by the path and the filename
        """
        script_path = os.path.join(self.path, self.filename)

        try:
            script_file = open(script_path, 'r')
        except:
            raise IOError("File could not be opened!")

        self.script = script_file.read()
        script_file.close()

        module_name = os.path.splitext(self.filename)[0] + str(self._script_id)

        # here is the pretty way for loading a module
        tmp_module = imp.new_module(module_name)

        sys.modules[module_name] = tmp_module

        # maybe substitute some variables in the script

        code = compile(self.script, '%s (%s)' % (self.filename, self._script_id), 'exec')

        try:
            exec code in tmp_module.__dict__
        except RuntimeError, e:
            raise IOError("Something went wrong during compilation of the module - error message: %s" % str(e))

        # return the module
        self._compiled_module = tmp_module

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'path': data.path,
            'filename': data.filename,
        }
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
    def path(self):
        """Property for the _path field

        """
        return self._path

    @path.setter
    @Observable.observed
    def path(self, path):
        if not path is None:
            if not isinstance(path, str):
                raise TypeError("path must be of type str")

        self._path = path

    @property
    def filename(self):
        """Property for the _filename field

        """
        return self._filename

    @filename.setter
    @Observable.observed
    def filename(self, filename):
        if not filename is None:
            if not isinstance(filename, str):
                raise TypeError("filename must be of type str")

        self._filename = filename

    @property
    def compiled_module(self):
        """Property for the _compiled_module field

        """
        return self._compiled_module

    # this setter should actually never be called as the module will be compiled by the build_module() function
    @compiled_module.setter
    @Observable.observed
    def compiled_module(self, compiled_module):
        self._compiled_module = compiled_module