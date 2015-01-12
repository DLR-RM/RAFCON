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

from statemachine.id_generator import *
import statemachine.singleton


class Script(Observable, yaml.YAMLObject):

    """A class for representing the script file for each state in a state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar _path: the path where the script resides
    :ivar _filename: the full name of the script file
    :ivar _compiled_module: holds the compiled module defined in the script file

    """

    EMPTY_SCRIPT = """
def entry(self):
    pass

def execute(self):
    return 0

def exit(self):
def exit(self):
def exit(self):
    pass
"""

    yaml_tag = u'!Script'

    def __init__(self, path=None, filename=None):

        Observable.__init__(self)

        self._path = path
        self._filename = filename
        self._compiled_module = None
        self._script_id = generate_script_id()
        self.script = Script.EMPTY_SCRIPT

    def execute(self, state, inputs={}, outputs={}):
        return self._compiled_module.execute(state, inputs, outputs,
                                             statemachine.singleton.external_module_manager.external_modules,
                                             statemachine.singleton.global_variable_manager)

    def enter(self, state, scoped_variables={}):
        return self._compiled_module.enter(state, scoped_variables,
                                           statemachine.singleton.external_module_manager.external_modules,
                                           statemachine.singleton.global_variable_manager)

    def exit(self, state, scoped_variables={}):
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
        #print self.script

        module_name = os.path.splitext(self.filename)[0] + str(self._script_id)
        #print module_name

        # here is the pretty way for loading a module
        tmp_module = imp.new_module(module_name)

        sys.modules[module_name] = tmp_module

        # maybe substitute some variables in the script

        code = compile(self.script, '%s (%s)' % (self.filename, self._script_id), 'exec')

        try:
            exec code in tmp_module.__dict__
        except:
            raise IOError("Something went wrong during compilation of the module")

        # return the module
        self._compiled_module = tmp_module

        script_file.close()

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'path': data.path,
            'filename': data.filename,
        }
        print dict_representation
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