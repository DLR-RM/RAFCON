"""
.. module:: execution_state
   :platform: Unix, Windows
   :synopsis: A module to represent a state for executing arbitrary functions

.. moduleauthor:: Sebastian Brunner


"""

import yaml
import traceback

from awesome_tool.statemachine.enums import StateType
from awesome_tool.statemachine.states.state import State
from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.enums import MethodName
from awesome_tool.statemachine.execution.execution_history import ReturnItem


class ExecutionState(State, yaml.YAMLObject):

    """A class to represent a state for executing arbitrary functions

    This kind of state does not have any child states.
    """

    yaml_tag = u'!ExecutionState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 path=None, filename=None, check_path=True):

        State.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, path, filename,
                       state_type=StateType.EXECUTION, check_path=check_path)
        self.logger = log.get_logger(self.name)

    def print_state_information(self):
        """Prints information about the state

        """
        print "---  \nState information of state: %s" % self.name
        print "Id of the state: " + self.state_id
        print "---"

    def _execute(self, execute_inputs, execute_outputs, backward_execution=False):
        """Calls the custom execute function of the script.py of the state

        """
        self.script.load_and_build_module()

        outcome_item = self.script.execute(self, execute_inputs, execute_outputs, backward_execution)

        if backward_execution:
            return  # in the case of backward execution the outcome is not relevant

        if outcome_item in self.outcomes.keys():
            return self.outcomes[outcome_item]

        for outcome_id, outcome in self.outcomes.iteritems():
            if outcome.name == outcome_item:
                return self.outcomes[outcome_id]

        logger.error("No valid outcome for execution state %s returned. Outcome item is %s.", self.name, outcome_item)
        return Outcome(-1, "aborted")

    def run(self):
        """ This defines the sequence of actions that are taken when the execution state is executed

        :return:
        """
        if self.backward_execution:
            self.setup_backward_run()
        else:
            self.setup_run()
        try:

            if self.backward_execution:
                logger.debug("Backward executing state with id %s and name %s" % (self._state_id, self.name))
                outcome = self._execute(self.input_data, self.output_data, backward_execution=True)
                # outcome handling is not required as we are in backward mode and the execution order is fixed
                self.active = False
                logger.debug("Finished backward executing state with id %s and name %s" % (self._state_id, self.name))
                return

            else:
                logger.debug("Executing state with id %s and name %s" % (self._state_id, self.name))
                outcome = self._execute(self.input_data, self.output_data)

                # check output data
                self.check_output_data_type()

                if self.concurrency_queue:
                    self.concurrency_queue.put(self.state_id)

                if self.preempted:
                    self.final_outcome = Outcome(-2, "preempted")
                    self.active = False
                    return

                self.final_outcome = outcome
                self.active = False
                return

        except Exception, e:
            logger.error("State %s had an internal error: %s %s" % (self.name, str(e), str(traceback.format_exc())))
            traceback.format_exc()
            self.final_outcome = Outcome(-1, "aborted")
            self.active = False
            return

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
        node = dumper.represent_mapping(u'!ExecutionState', dict_representation)
        return node


    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        name = dict_representation['name']
        state_id = dict_representation['state_id']
        # state_type = dict_representation['state_type']
        input_data_ports = dict_representation['input_data_ports']
        output_data_ports = dict_representation['output_data_ports']
        outcomes = dict_representation['outcomes']
        path = dict_representation['path']
        filename = dict_representation['filename']
        return ExecutionState(name, state_id, input_data_ports, output_data_ports, outcomes, path, filename,
                              check_path=False)

    @staticmethod
    def copy_state(source_state):
        state_copy = ExecutionState()
        # TODO: copy fields from source_state into the state_copy
        return state_copy