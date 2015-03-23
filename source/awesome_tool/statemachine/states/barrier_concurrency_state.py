"""
.. module:: barrier_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a barrier concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

import yaml

from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.outcome import Outcome
from concurrency_state import ConcurrencyState
from container_state import ContainerState
from awesome_tool.statemachine.enums import StateType


class BarrierConcurrencyState(ConcurrencyState, yaml.YAMLObject):
    """ The barrier concurrency holds a list of states that are executed in parallel. It waits until all states
        finished their execution before it returns.
    """
    yaml_tag = u'!BarrierConcurrencyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None, check_path=True):

        ConcurrencyState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes,
                                  states, transitions, data_flows, start_state, scoped_variables, v_checker, path,
                                  filename, state_type = StateType.BARRIER_CONCURRENCY, check_path=check_path)

    def run(self):
        """ This defines the sequence of actions that are taken when the barrier concurrency state is executed

        :return:
        """
        self.setup_run()

        self.add_input_data_to_scoped_data(self.input_data, self)
        self.add_default_values_of_scoped_variables_to_scoped_data()

        try:
            logger.debug("Starting preemptive concurrency state with id %s" % self._state_id)

            #handle data for the entry script
            scoped_variables_as_dict = {}
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
            self.enter(scoped_variables_as_dict)
            self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)

            self.child_execution = True

            #start all threads
            for key, state in self.states.iteritems():
                state_input = self.get_inputs_for_state(state)
                state_output = self.get_outputs_for_state(state)
                state.input_data = state_input
                state.output_data = state_output
                state.start()

            #wait for all threads
            for key, state in self.states.iteritems():
                state.join()
                self.add_state_execution_output_to_scoped_data(state.output_data, state)
                self.update_scoped_variables_with_output_dictionary(state.output_data, state)

            self.child_execution = False

            #handle data for the exit script
            scoped_variables_as_dict = {}
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
            self.exit(scoped_variables_as_dict)
            self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)

            self.write_output_data()

            self.check_output_data_type()

            if self.concurrency_queue:
                self.concurrency_queue.put(self.state_id)

            # check the outcomes of all states for aborted or preempted
            # check as well if the states were stopped
            for key, state in self.states.iteritems():
                #This is the case if execution was stopped
                if state.final_outcome is None:
                    quit()
                if state.final_outcome.outcome_id == -1:
                    self.final_outcome = Outcome(-1, "preempted")
                    self.active = False
                    return
                if state.final_outcome.outcome_id == -2:
                    self.final_outcome = Outcome(-2, "aborted")
                    self.active = False
                    return

            if self.preempted:
                self.final_outcome = Outcome(-2, "preempted")
                self.active = False
                return

            self.final_outcome = Outcome(0, "success")
            self.active = False
            return

        except Exception, e:
            logger.error("Runtime error %s" % e)
            self.final_outcome = Outcome(-1, "aborted")
            self.active = False
            self.child_execution = False
            return

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = ContainerState.get_container_state_yaml_dict(data)
        node = dumper.represent_mapping(u'!BarrierConcurrencyState', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        return BarrierConcurrencyState(name=dict_representation['name'],
                              state_id=dict_representation['state_id'],
                              input_data_ports=dict_representation['input_data_ports'],
                              output_data_ports=dict_representation['output_data_ports'],
                              outcomes=dict_representation['outcomes'],
                              states=None,
                              transitions=dict_representation['transitions'],
                              data_flows=dict_representation['data_flows'],
                              start_state=dict_representation['start_state'],
                              scoped_variables=dict_representation['scoped_variables'],
                              v_checker=None,
                              path=dict_representation['path'],
                              filename=dict_representation['filename'],
                              check_path=False)

    @staticmethod
    def copy_state(source_state):
        state_copy = BarrierConcurrencyState()
        # TODO: copy fields from source_state into the state_copy
        return state_copy