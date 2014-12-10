"""
.. module:: hierarchy_state
   :platform: Unix, Windows
   :synopsis: A module to represent a hierarchy state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

from statemachine.container_state import ContainerState
from utils import log
logger = log.get_logger(__name__)

from statemachine.outcome import Outcome
from statemachine.transition import Transition

class HierarchyState(ContainerState):

    """A class tto represent a hierarchy state for the state machine

    The hierarchy state holds several child states, that be hierarchy states themselves
    """

    def __init__(self, name=None, state_id=None, input_keys={}, output_keys={}, outcomes={}, sm_status=None,
                 states={}, transitions={}, data_flows={}, start_state=None, scope={},
                 scoped_keys=None, v_checker=None):

        ContainerState.__init__(self, name, state_id, input_keys, output_keys, outcomes, sm_status, states, transitions,
                                data_flows, start_state, scope, scoped_keys, v_checker)

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
        #TODO: implement
        #if not isinstance(state, State):
        #    raise TypeError("state must be of type State")
        result_dict = {}
        logger.debug("Return data outputs for container state with id %s", self._state_id)
        return result_dict

    def create_output_dict_for_state(self, state):
        result_dict = None
        for key, value in state.output_keys.iteritems():
            result_dict[key] = None
        return result_dict

    def add_dict_to_scope(self, dictionary):
        for key, value in dictionary.iteritems():
            self.scope[key] = dictionary[key]

    def get_state_for_transition(self, transition):
        if not isinstance(transition, Transition):
            raise TypeError("transition must be of type Transition")
        if transition.to_state is None:
            return self
        else:
            return self.states[transition.to_state]

    def get_outcome_for_transition(self, transition):
        if not isinstance(transition, Transition):
            raise TypeError("transition must be of type Transition")
        #TODO: return correct state
        return Outcome()


    def run(self, *args, **kwargs):

        input_data = kwargs["inputs"]
        outputs_data = kwargs["outputs"]
        #print input_data
        #print outputs_data

        if not isinstance(input_data, dict):
            raise TypeError("states must be of type dict")
        if not isinstance(outputs_data, dict):
            raise TypeError("states must be of type dict")
        self.add_dict_to_scope(input_data)
        self.add_dict_to_scope(outputs_data)

        try:
            logger.debug("Starting hierarchy state with id %s" % self._state_id)
            self.enter()
            transition = None

            state = self.get_start_state()

            while not state is self:
                state_input = self.get_inputs_for_state(state)
                state_output = self.get_outputs_for_state(state)
                outcome = state.run(inputs=state_input, outputs=state_output)
                self.add_dict_to_scope(state_output)
                # not explicitly connected preempted outcomes are implicit connected to parent preempted outcome
                transition = self.get_transition_for_outcome(state, outcome)

                #how does condition variables work in python
                #while not transition:
                #    transition = self.get_transition_for_outcome(state, outcome)
                #    self._transitions.wait()

                state = self.get_state_for_transition(transition)

            self.exit(transition)
            return self.get_outcome_for_transition(transition)

        except RuntimeError:
            return Outcome("aborted")

        #write output data back to the dictionary
        for key, value in outputs.iteritems():
            kwargs["outputs"][key] = scope[key].value()