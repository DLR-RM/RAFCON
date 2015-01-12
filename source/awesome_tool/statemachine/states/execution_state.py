"""
.. module:: execution_state
   :platform: Unix, Windows
   :synopsis: A module to represent a state for executing arbitrary functions

.. moduleauthor:: Sebastian Brunner


"""

import yaml

from statemachine.states.state import State, StateType
from utils import log
logger = log.get_logger(__name__)
from statemachine.outcome import Outcome


class ExecutionState(State, yaml.YAMLObject):

    """A class to represent a state for executing arbitrary functions

    This kind of state does not have any child states.
    """

    yaml_tag = u'!ExecutionState'

    def __init__(self, name=None, state_id=None, input_keys=None, output_keys=None, outcomes=None, sm_status=None,
                 path=None, filename=None):

        State.__init__(self, name, state_id, input_keys, output_keys, outcomes, sm_status, path, filename)
        self.state_type = StateType.EXECUTION

    def print_state_information(self):
        """Prints information about the state

        """
        print "---  \nState information of state: %s" % self.name
        print "Id of the state: " + self.state_id
        print "---"

    def _execute(self, execute_inputs, execute_outputs):
        """Calls the custom execute function of the script.py of the state

        """
        self.script.load_and_build_module()
        outcome_id = self.script.execute(self, execute_inputs, execute_outputs)
        return self.outcomes[outcome_id]

    def run(self):
        try:

            #initialize data structures
            input_data = self.input_data
            #print input_data
            output_data = self.output_data
            if not isinstance(input_data, dict):
                raise TypeError("states must be of type dict")
            if not isinstance(output_data, dict):
                raise TypeError("states must be of type dict")

            self.check_input_data_type(input_data)

            logger.debug("Starting state with id %s" % self._state_id)
            outcome = self._execute(input_data, output_data)

            #check output data
            self.check_output_data_type(output_data)

            if self.concurrency_queue:
                self.concurrency_queue.put(self.state_id)

            self.final_outcome = outcome
            return

        except RuntimeError:
            self.final_outcome = Outcome(1, "aborted")
            return

    def __str__(self):
        return "state type: %s\n%s" % (self.state_type, State.__str__(self))

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'name': data.name,
            'state_id': data.state_id,
            'state_type': str(data.state_type),
            'input_data_ports': data.input_data_ports,
            'output_data_ports': data.output_data_ports,
            'outcomes': data.outcomes,
            'path': data.script.path,
            'filename': data.script.filename
        }
        print dict_representation
        node = dumper.represent_mapping(u'!ExecutionState', dict_representation)
        return node


    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        name = dict_representation['name']
        state_id = dict_representation['state_id']
        state_type = dict_representation['state_type']
        input_data_ports = dict_representation['input_data_ports']
        output_data_ports = dict_representation['output_data_ports']
        outcomes = dict_representation['outcomes']
        path = dict_representation['path']
        filename = dict_representation['filename']
        return ExecutionState(name, state_id, input_data_ports, output_data_ports, outcomes, None, path, filename)