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

    def __init__(self, name=None, state_id=None, input_keys=None, output_keys=None, outcomes=None, sm_status=None,
                 states=None, transitions=None, data_flows=None, start_state=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None):

        ContainerState.__init__(self, name, state_id, input_keys, output_keys, outcomes, sm_status, states, transitions,
                                data_flows, start_state, scoped_variables, v_checker, path, filename)

    def get_start_state(self):
        if self._sm_status.dependency_tree is None:
            return self._start_state
        else:
            #do start with state provided by the dependency tree
            pass

    def get_inputs_for_state(self, state):
        result_dict = {}
        for input_key, value in state.input_data_ports.iteritems():
            # for all input keys fetch the correct data_flow connection and read data into the result_dict
            for data_flow_key, data_flow in self.data_flows.iteritems():
                if data_flow.to_key is input_key:
                    if data_flow.to_state is state:
                        #data comes from scope variable of parent
                        if data_flow.from_state is self:
                            result_dict[input_key] = self.scoped_variables[data_flow.from_key].value
                        else:  # data comes from result from neighbouring state
                            #primary key for scoped_results is key+state_id
                            result_dict[input_key] =\
                                self.scoped_results[data_flow.from_key+data_flow.from_state.state_id].value
        return result_dict

    def get_outputs_for_state(self, state):
        """Return data outputs for a state

        """
        result_dict = {}
        for key, value in state.output_data_ports.iteritems():
            result_dict[key] = None
        return result_dict

    def add_dict_to_scope_variables(self, dictionary):
        for key, value in dictionary.iteritems():
            self.scoped_variables[key] = ScopedVariable(key, value, None, self)

    def add_dict_to_scoped_results(self, dictionary, state):
        for key, value in dictionary.iteritems():
            #primary key for scoped_results is key+state_id
            self.scoped_results[key+state.state_id] = ScopedResult(key, value, str(type(value)), state)

    def get_state_for_transition(self, transition):
        if not isinstance(transition, Transition):
            raise TypeError("transition must be of type Transition")
        if transition.to_state is None:
            return self
        else:
            return self.states[transition.to_state]

    def run(self, *args, **kwargs):

        #initialize data structures
        input_data = kwargs["inputs"]
        output_data = kwargs["outputs"]
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
                outcome = state.run(inputs=state_input, outputs=state_output)
                self.add_dict_to_scoped_results(state_output, state)
                # not explicitly connected preempted outcomes are implicit connected to parent preempted outcome
                transition = self.get_transition_for_outcome(state, outcome)

                while not transition:
                    self._transitions_cv.wait()
                    transition = self.get_transition_for_outcome(state, outcome)

                state = self.get_state_for_transition(transition)

            self.exit(transition)

            #write output data back to the dictionary
            for output_key, value in output_data.iteritems():
                for data_flow_key, data_flow in self.data_flows.iteritems():
                    if data_flow.to_state is self:
                        if data_flow.from_state is self:
                            kwargs["outputs"][output_key] = self.scoped_variables[output_key].value()
                        else:
                            #primary key for scoped_results is key+state_id
                            kwargs["outputs"][output_key] =\
                                self.scoped_results[output_key+data_flow.from_state.state_id].value()

            self.check_output_data_type(output_data)

            return transition.to_outcome

        except RuntimeError:
            return Outcome("aborted")