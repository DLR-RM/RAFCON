"""
.. module:: hierarchy_state
   :platform: Unix, Windows
   :synopsis: A module to represent a hierarchy state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

from statemachine.states.container_state import ContainerState
from utils import log
logger = log.get_logger(__name__)
import time

from statemachine.outcome import Outcome


class HierarchyState(ContainerState):

    """A class tto represent a hierarchy state for the state machine

    The hierarchy state holds several child states, that can be container states again
    """

    def __init__(self, name=None, state_id=None, input_keys=None, output_keys=None, outcomes=None, sm_status=None,
                 states=None, transitions=None, data_flows=None, start_state=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None):

        ContainerState.__init__(self, name, state_id, input_keys, output_keys, outcomes, sm_status, states, transitions,
                                data_flows, start_state, scoped_variables, v_checker, path, filename)

    def run(self):

        #initialize data structures
        input_data = self.input_data
        output_data = self.output_data
        if not isinstance(input_data, dict):
            raise TypeError("states must be of type dict")
        if not isinstance(output_data, dict):
            raise TypeError("states must be of type dict")
        self.scoped_variables = {}
        self.scoped_results = {}

        self.check_input_data_type(input_data)
        self.add_dict_to_scope_variables(input_data)

        try:
            logger.debug("Starting hierarchy state with id %s" % self._state_id)
            self.enter()
            transition = None

            state = self.get_start_state()

            while not state is self:
                state_input = self.get_inputs_for_state(state)
                state_output = self.get_outputs_for_state(state)
                state.input_data = state_input
                state.output_data = state_output
                state.run()
                self.add_dict_to_scoped_results(state_output, state)
                #print "Final outcome of state is " + str(state.final_outcome)
                # not explicitly connected preempted outcomes are implicit connected to parent preempted outcome
                transition = self.get_transition_for_outcome(state, state.final_outcome)

                while not transition:
                    self._transitions_cv.wait(3.0)
                    if self.preempted:
                        self.final_outcome = Outcome(2, "preempted")
                        return
                    transition = self.get_transition_for_outcome(state, state.final_outcome)
                state = self.get_state_for_transition(transition)

            self.exit()
            #print transition

            #write output data back to the dictionary
            for output_key, value in output_data.iteritems():
                for data_flow_key, data_flow in self.data_flows.iteritems():
                    if data_flow.to_state is self:
                        if data_flow.from_state is self:
                            output_data[output_key] = self.scoped_variables[output_key].value()
                        else:
                            #primary key for scoped_results is key+state_id
                            output_data[output_key] =\
                                self.scoped_results[output_key+data_flow.from_state.state_id].value()

            self.check_output_data_type(output_data)

            """
            for i in range(5):
                if self.preempted:
                    print str(self.__class__) + " was preempted"
                    return transition.to_outcome
                print i
                time.sleep(1.0)
            """

            if self.preempted:
                self.final_outcome = Outcome(2, "preempted")
                return

            self.final_outcome = self.outcomes[transition.to_outcome]
            return

        except RuntimeError:
            self.final_outcome = Outcome(1, "aborted")
            return