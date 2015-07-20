"""
.. module:: execution_state
   :platform: Unix, Windows
   :synopsis: A module to represent a state for executing arbitrary functions

.. moduleauthor:: Sebastian Brunner


"""

import traceback

from gtkmvc import Observable
from awesome_tool.statemachine.states.state import State
from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.script import Script, ScriptType
from awesome_tool.statemachine.enums import StateExecutionState


class ExecutionState(State):

    """A class to represent a state for executing arbitrary functions

    This kind of state does not have any child states.
    """

    yaml_tag = u'!ExecutionState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 path=None, filename=None, check_path=True):

        State.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes)
        self.script = Script(path, filename, script_type=ScriptType.EXECUTION, check_path=check_path, state=self)
        self.logger = log.get_logger(self.name)
        # here all persistent variables that should be available for the next state run should be stored
        self.persistent_variables = {}

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
                self._execute(self.input_data, self.output_data, backward_execution=True)
                # outcome handling is not required as we are in backward mode and the execution order is fixed
                self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
                logger.debug("Finished backward executing state with id %s and name %s" % (self._state_id, self.name))
                return self.finalize()

            else:
                logger.debug("Executing state with id %s and name %s" % (self._state_id, self.name))
                outcome = self._execute(self.input_data, self.output_data)

                self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
                # check output data
                self.check_output_data_type()

                if self.preempted:
                    outcome = Outcome(-2, "preempted")

                return self.finalize(outcome)

        except Exception, e:
            logger.error("State {0} had an internal error: {1}\n{2}".format(self.name,
                                                                            str(e), str(traceback.format_exc())))
            # write error to the output_data of the state
            self.output_data["error"] = e
            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
            return self.finalize(Outcome(-1, "aborted"))

    def get_execution_state_yaml_dict(data):
        dict_representation = {
            'name': data.name,
            'state_id': data.state_id,
            'description': data.description,
            'input_data_ports': data.input_data_ports,
            'output_data_ports': data.output_data_ports,
            'outcomes': data.outcomes,
            'path': data.script.path,
            'filename': data.script.filename
        }
        return dict_representation

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = ExecutionState.get_execution_state_yaml_dict(data)
        node = dumper.represent_mapping(cls.yaml_tag, dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        name = dict_representation['name']
        state_id = dict_representation['state_id']
        input_data_ports = dict_representation['input_data_ports']
        output_data_ports = dict_representation['output_data_ports']
        outcomes = dict_representation['outcomes']
        path = dict_representation['path']
        filename = dict_representation['filename']
        state = ExecutionState(name, state_id, input_data_ports, output_data_ports, outcomes, path, filename,
                              check_path=False)
        try:
            state.description = dict_representation['description']
        except (ValueError, TypeError, KeyError):
            pass
        return state

    @State.name.setter
    @Observable.observed
    def name(self, name):
        State.name.fset(self, name)
        self.logger = log.get_logger(self.name)
