"""
.. module:: hierarchy_state
   :platform: Unix, Windows
   :synopsis: A module to represent a hierarchy state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

from statemachine.states.container_state import ContainerState
from utils import log
logger = log.get_logger(__name__)

from statemachine.outcome import Outcome
from statemachine.transition import Transition
from statemachine.scope import ScopedVariable, ScopedResult

class HierarchyState(ContainerState):

    """A class tto represent a hierarchy state for the state machine

    The hierarchy state holds several child states, that be hierarchy states themselves
    """

    def __init__(self, name=None, state_id=None, input_keys={}, output_keys={}, outcomes={}, sm_status=None,
                 states={}, transitions={}, data_flows={}, start_state=None, scope_variables={}, v_checker=None):

        ContainerState.__init__(self, name, state_id, input_keys, output_keys, outcomes, sm_status, states, transitions,
                                data_flows, start_state, scope_variables, v_checker)

    def get_start_state(self):
        if self._sm_status._dependency_tree is None:
            return self._start_state
        else:
            #do start with state provided by the dependency tree
            pass

    def get_inputs_for_state(self, state):
        result_dict = {}
        #TODO: implement
        for key, value in state.input_keys.iteritems():
            #check all data flows for the input keys of the state and remap the values into result_dict
            pass
        return result_dict

    def get_outputs_for_state(self, state):
        """Return data outputs for a state

        """
        result_dict = {}
        for key, value in state.output_keys.iteritems():
            result_dict[key] = None
        return result_dict

    def add_dict_to_scope_variables(self, dictionary):
        for key, value in dictionary.iteritems():
            self.scope_variables[key] = ScopedVariable(key, value)

    def add_dict_to_scoped_results(self, dictionary):
        for key, value in dictionary.iteritems():
            self.scope_variables[key] = ScopedResult(key, value)

    def get_state_for_transition(self, transition):
        #if not isinstance(transition, Transition):
        #    raise TypeError("transition must be of type Transition")
        if transition.to_state is None:
            return self
        else:
            return self.states[transition.to_state]

    def run(self, *args, **kwargs):

        #initialize data structures
        input_data = kwargs["inputs"]
        outputs_data = kwargs["outputs"]
        if not isinstance(input_data, dict):
            raise TypeError("states must be of type dict")
        if not isinstance(outputs_data, dict):
            raise TypeError("states must be of type dict")
        self.scope_variables = {}
        self.scoped_results = {}

        self.add_dict_to_scope_variables(input_data)
        self.add_dict_to_scope_variables(outputs_data)

        try:
            logger.debug("Starting hierarchy state with id %s" % self._state_id)
            self.enter()
            transition = None

            state = self.get_start_state()

            while not state is self:
                state_input = self.get_inputs_for_state(state)
                state_output = self.get_outputs_for_state(state)
                outcome = state.run(inputs=state_input, outputs=state_output)
                self.add_dict_to_scoped_results(state_output)
                # not explicitly connected preempted outcomes are implicit connected to parent preempted outcome
                transition = self.get_transition_for_outcome(state, outcome)

                while not transition:
                    self._transitions_cv.wait()
                    transition = self.get_transition_for_outcome(state, outcome)

                state = self.get_state_for_transition(transition)

            self.exit(transition)
            #write output data back to the dictionary
            for key, value in outputs_data.iteritems():
                #check all data flows for the output keys of the state and remap the values into the output_data
                #TODO: implement
                pass
                #kwargs["outputs"][key] = self.scope_variables[key].value()
            return transition.to_outcome

        except RuntimeError:
            return Outcome("aborted")