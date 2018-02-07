import pytest
import testing_utils

from gtkmvc.observer import Observer


class SignalCounter(Observer):
    """Observes a s signal and counts its emits"""

    def __init__(self, model, signal_name):
        super(SignalCounter, self).__init__()
        model.register_observer(self)
        self._signal_name = signal_name
        self._signal_counter = 0

    @Observer.observe("*_signal", signal=True)
    def increment_signal_counter(self, selection_m, signal_name, signal_msg):
        """ Increments counter if specified signal is emitted """
        if signal_name == self._signal_name:
            self._signal_counter += 1

    @property
    def count(self):
        """ Return the count of how often the signal was emitted """
        return self._signal_counter


def get_models():
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.gui.models.container_state import ContainerStateModel

    def get_outcome_with_name(state_m, outcome_name):
        return [outcome_m for outcome_m in state_m.outcomes if outcome_m.outcome.name == outcome_name][0]

    def get_state_with_name(parent_state_m, state_name):
        return [state_m for state_m in parent_state_m.states.itervalues() if state_m.state.name == state_name][0]

    execution_state = ExecutionState("ex1")
    execution_state.add_outcome("oe1")

    child_state = ExecutionState("ex2")

    hierarchy_state = HierarchyState("h1")
    hierarchy_state.add_outcome("oh1")
    hierarchy_state.add_state(child_state)

    root_state = HierarchyState("root")
    root_state.add_state(execution_state)
    root_state.add_state(hierarchy_state)

    root_state_m = ContainerStateModel(root_state)
    execution_state_m = get_state_with_name(root_state_m, "ex1")
    hierarchy_state_m = get_state_with_name(root_state_m, "h1")
    child_state_m = get_state_with_name(hierarchy_state_m, "ex2")

    outcome_e_success_m = get_outcome_with_name(execution_state_m, "success")
    outcome_e_1_m = get_outcome_with_name(execution_state_m, "oe1")
    outcome_h_success_m = get_outcome_with_name(hierarchy_state_m, "success")
    outcome_h_1_m = get_outcome_with_name(hierarchy_state_m, "oh1")

    return (root_state_m, execution_state_m, hierarchy_state_m, child_state_m), (outcome_e_success_m, outcome_e_1_m), \
        (outcome_h_success_m, outcome_h_1_m)


def test_add_set_remove_clear():
    testing_utils.dummy_gui(None)
    from rafcon.gui.models.selection import Selection
    selection = Selection()
    signal_observer = SignalCounter(selection, "selection_changed_signal")
    states_m, outcomes_e_m, outcomes_h_m = get_models()
    root_state_m, execution_state_m, hierarchy_state_m, child_state_m = states_m

    assert len(selection) == 0

    # Add 1st state
    selection.add(execution_state_m)
    assert len(selection) == 1
    assert len(selection.states) == 1
    assert selection.get_selected_state() is execution_state_m
    assert signal_observer.count == 1

    # Add 2nd state
    selection.add(hierarchy_state_m)
    assert len(selection) == 2
    assert len(selection.states) == 2
    assert signal_observer.count == 2

    # Add 1st state again => should not affect selection
    selection.add(execution_state_m)
    assert len(selection) == 2
    assert len(selection.states) == 2
    assert signal_observer.count == 2

    # Remove 1st state
    selection.remove(execution_state_m)
    assert len(selection) == 1
    assert len(selection.states) == 1
    assert selection.get_selected_state() is hierarchy_state_m
    assert signal_observer.count == 3

    # Clear selection
    selection.clear()
    assert len(selection) == 0
    assert len(selection.states) == 0
    assert selection.get_selected_state() is None
    assert signal_observer.count == 4

    # Clear selection again => should cause no effect
    selection.clear()
    assert len(selection) == 0
    assert len(selection.states) == 0
    assert selection.get_selected_state() is None
    assert signal_observer.count == 4

    # Set selection to two outcomes
    selection.set(outcomes_e_m)
    assert len(selection) == 2
    assert len(selection.states) == 0
    assert len(selection.outcomes) == 2
    assert selection.get_selected_state() is None
    assert signal_observer.count == 5

    # Set selection to two outcomes again => should cause no effect
    selection.set(outcomes_e_m)
    assert len(selection) == 2
    assert len(selection.outcomes) == 2
    assert signal_observer.count == 5


def test_all_models():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.state import State
    from rafcon.core.state_elements.data_port import InputDataPort, OutputDataPort
    from rafcon.core.state_elements.scope import ScopedVariable
    from rafcon.core.state_elements.outcome import Outcome
    from rafcon.core.state_elements.transition import Transition
    from rafcon.core.state_elements.data_flow import DataFlow

    from rafcon.gui.models.selection import Selection
    from rafcon.gui.models.data_port import DataPortModel
    from rafcon.gui.models.scoped_variable import ScopedVariableModel
    from rafcon.gui.models.transition import TransitionModel
    from rafcon.gui.models.data_flow import DataFlowModel

    selection = Selection()
    states_m, outcomes_e_m, outcomes_h_m = get_models()

    input_data_port = InputDataPort("i")
    input_data_port_m = DataPortModel(input_data_port, parent=None)
    output_data_port = OutputDataPort("o")
    output_data_port_m = DataPortModel(output_data_port, parent=None)
    scoped_variable = ScopedVariable("sv")
    scoped_variable_m = ScopedVariableModel(scoped_variable, parent=None)

    transition = Transition("0", 0, "1", 0, 0)
    transition_m = TransitionModel(transition, parent=None)
    data_flow = DataFlow("0", 0, "1", 0, 0)
    data_flow_m = DataFlowModel(data_flow, parent=None)

    selection.add(states_m[3])  # child_state_m
    selection.add(outcomes_e_m)
    selection.add(outcomes_h_m)
    selection.add((input_data_port_m, output_data_port_m, scoped_variable_m))
    selection.add(transition_m)
    selection.add(data_flow_m)

    assert len(selection) == 10
    assert len(selection.states) == 1
    assert len(selection.outcomes) == 4
    assert len(selection.input_data_ports) == 1
    assert len(selection.output_data_ports) == 1
    assert len(selection.scoped_variables) == 1
    assert len(selection.data_flows) == 1
    assert len(selection.transitions) == 1
    assert selection.states == selection.get_selected_elements_of_core_class(State)
    assert selection.outcomes == selection.get_selected_elements_of_core_class(Outcome)
    assert selection.input_data_ports == selection.get_selected_elements_of_core_class(InputDataPort)
    assert selection.output_data_ports == selection.get_selected_elements_of_core_class(OutputDataPort)
    assert selection.scoped_variables == selection.get_selected_elements_of_core_class(ScopedVariable)
    assert selection.data_flows == selection.get_selected_elements_of_core_class(DataFlow)
    assert selection.transitions == selection.get_selected_elements_of_core_class(Transition)

    selection.clear()

    assert 0 == len(selection) == len(selection.states) == len(selection.outcomes) == len(selection.input_data_ports)\
             == len(selection.output_data_ports) == len(selection.scoped_variables) == len(selection.data_flows)\
             == len(selection.transitions)


def test_invalid_model():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.state import State
    from rafcon.gui.models.selection import Selection
    from rafcon.gui.models.meta import MetaModel

    selection = Selection()
    meta_m = MetaModel()

    with pytest.raises(TypeError):
        selection.add(meta_m)
    with pytest.raises(TypeError):
        selection.set(meta_m)
    with pytest.raises(TypeError):
        selection.focus(meta_m)
    with pytest.raises(TypeError):
        selection.handle_new_selection(meta_m)
    with pytest.raises(TypeError):
        selection.handle_prepared_selection_of_core_class_elements(meta_m, State)


def test_adding_same_model_twice():
    testing_utils.dummy_gui(None)
    from rafcon.gui.models.selection import Selection, reduce_to_parent_states

    selection = Selection()
    states_m, outcomes_e_m, outcomes_h_m = get_models()
    root_state_m, execution_state_m, hierarchy_state_m, child_state_m = states_m
    duplicate_states = [root_state_m, execution_state_m, execution_state_m]

    selection.add(duplicate_states)
    assert len(selection) == 1

    selection.clear()

    selection.set(duplicate_states)
    assert len(selection) == 1

    reduced_states = reduce_to_parent_states(duplicate_states)
    assert len(reduced_states) == 1

    assert len(duplicate_states) == 3


def test_selection_reduction():
    testing_utils.dummy_gui(None)
    from rafcon.gui.models.selection import Selection
    selection = Selection()
    states_m, outcomes_e_m, outcomes_h_m = get_models()
    root_state_m, execution_state_m, hierarchy_state_m, child_state_m = states_m

    # Select outcomes
    selection.set(outcomes_e_m)
    assert len(selection) == 2
    assert len(selection.outcomes) == 2

    # Select parent state of outcomes
    selection.add(execution_state_m)
    assert len(selection) == 1
    assert len(selection.states) == 1
    assert len(selection.outcomes) == 0
    assert selection.get_selected_state() is execution_state_m

    # select root state
    selection.add(root_state_m)
    assert len(selection) == 1
    assert len(selection.states) == 1
    assert len(selection.outcomes) == 0
    assert selection.get_selected_state() is root_state_m

    # Select child state of root state
    selection.add(hierarchy_state_m)
    assert len(selection) == 1
    assert len(selection.states) == 1
    assert len(selection.outcomes) == 0
    assert selection.get_selected_state() is root_state_m

    # Select outcomes of child state
    selection.add(outcomes_h_m)
    assert len(selection) == 1
    assert len(selection.states) == 1
    assert len(selection.outcomes) == 0
    assert selection.get_selected_state() is root_state_m


def test_focus():
    testing_utils.dummy_gui(None)
    from rafcon.gui.models.selection import Selection

    selection = Selection()
    focus_signal_observer = SignalCounter(selection, "focus_signal")
    selection_signal_observer = SignalCounter(selection, "selection_changed_signal")
    states_m, outcomes_e_m, outcomes_h_m = get_models()
    root_state_m, execution_state_m, hierarchy_state_m, child_state_m = states_m

    assert len(selection) == 0

    # Set focus
    selection.focus = execution_state_m
    assert selection.focus is execution_state_m
    assert len(selection) == 1
    assert len(selection.states) == 1
    assert focus_signal_observer.count == 1
    assert selection_signal_observer.count == 1

    # Set focus to same element
    selection.focus = execution_state_m
    assert selection.focus is execution_state_m
    assert len(selection) == 1
    assert len(selection.states) == 1
    assert focus_signal_observer.count == 2
    assert selection_signal_observer.count == 1

    # Clear selection => causes focus to me removed
    selection.clear()
    assert selection.focus is None
    assert len(selection) == 0
    assert len(selection.states) == 0
    assert focus_signal_observer.count == 3
    assert selection_signal_observer.count == 2

if __name__ == '__main__':
    test_add_set_remove_clear()
    test_adding_same_model_twice()
    test_all_models()
    test_focus()
    test_invalid_model()
    # pytest.main(['-s', __file__])