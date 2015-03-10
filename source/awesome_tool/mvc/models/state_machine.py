from gtkmvc import ModelMT, Observable

from awesome_tool.statemachine.state_machine import StateMachine
from awesome_tool.statemachine.states.container_state import ContainerState
from awesome_tool.mvc.models import ContainerStateModel, StateModel, TransitionModel, DataFlowModel

from awesome_tool.utils.vividict import Vividict


class StateMachineModel(ModelMT):
    """This model class manages a StateMachine

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state machine).

    :param StateMachine state_machine: The state machine to be controlled and modified
    """

    state_machine = None
    selection = None
    root_state = None

    __observables__ = ("state_machine", "root_state", "selection")

    def __init__(self, state_machine, meta=None):
        """Constructor
        """
        ModelMT.__init__(self)  # pass columns as separate parameters

        assert isinstance(state_machine, StateMachine)

        self.state_machine = state_machine

        root_state = self.state_machine.root_state
        if isinstance(root_state, ContainerState):
            self.root_state = ContainerStateModel(root_state)
        else:
            self.root_state = StateModel(root_state)

        self.root_state.register_observer(self)

        self.selection = Selection()

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

    @ModelMT.observe("state", before=True)
    @ModelMT.observe("states", before=True)
    @ModelMT.observe("transitions", before=True)
    @ModelMT.observe("data_flows", before=True)
    @ModelMT.observe("input_data_ports", before=True)
    @ModelMT.observe("scoped_variables", before=True)
    def root_state_model_before_change(self, model, prop_name, info):
        self.state_machine.root_state_before_change(info['model'], info['prop_name'], info['instance'],
                                                    info['method_name'], info['args'], info['kwargs'])


    @ModelMT.observe("state", after=True)
    @ModelMT.observe("states", after=True)
    @ModelMT.observe("transitions", after=True)
    @ModelMT.observe("data_flows", after=True)
    @ModelMT.observe("input_data_ports", after=True)
    @ModelMT.observe("output_data_ports", after=True)
    @ModelMT.observe("scoped_variables", after=True)
    def root_state_model_after_change(self, model, prop_name, info):
        self.state_machine.root_state_after_change(info['model'], info['prop_name'], info['instance'],
                                                   info['method_name'], info['result'], info['args'], info['kwargs'])


class Selection(Observable):
    """
    This class contains the selected item (States, Transitions and Data Flows)
    of a state_machine

    :param set __selected: The state machine elements selected
    """
    __selected = None

    def __init__(self):
        Observable.__init__(self)

        self.__selected = set()

    @Observable.observed
    def add(self, item):
        #print item
        self.__selected.add(item)

    @Observable.observed
    def remove(self, item):
        if item in self.__selected:
            self.__selected.remove(item)

    @Observable.observed
    def append(self, selection):
        self.__selected.update(selection)

    @Observable.observed
    def set(self, selection):
        self.__selected.clear()
        self.__selected.update(selection)

    def __iter__(self):
        return self.__selected.__iter__()

    def __len__(self):
        return len(self.__selected)

    def get_all(self):
        return [s for s in self.__selected]

    def get_states(self):
        return [s for s in self.__selected if isinstance(s, StateModel)]

    def get_num_states(self):
        return sum((1 for s in self.__selected if isinstance(s, StateModel)))

    def get_transitions(self):
        return [s for s in self.__selected if isinstance(s, TransitionModel)]

    def get_num_transitions(self):
        return sum((1 for s in self.__selected if isinstance(s, TransitionModel)))

    def get_data_flows(self):
        return [s for s in self.__selected if isinstance(s, DataFlowModel)]

    def get_num_data_flows(self):
        return sum((1 for s in self.__selected if isinstance(s, DataFlowModel)))

    @Observable.observed
    def clear(self):
        self.set([])

    def get_selected_state(self):
        selected_states = self.get_states()
        if len(selected_states) != 1:
            return None
        else:
            return selected_states[0]
