from gtkmvc import ModelMT
from statemachine.state_machine import StateMachine
from mvc.models.container_state import ContainerStateModel
from utils.vividict import Vividict


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

        self.root_state = ContainerStateModel(self.state_machine.root_state)
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
    def root_state_model_changed(self, model, prop_name, info):
        self.notify_method_before_change("root_state", info['instance'], info['method_name'], None, kwargs=info)

    @ModelMT.observe("state", after=True)
    @ModelMT.observe("states", after=True)
    @ModelMT.observe("transitions", after=True)
    @ModelMT.observe("data_flows", after=True)
    @ModelMT.observe("input_data_ports", after=True)
    @ModelMT.observe("output_data_ports", after=True)
    @ModelMT.observe("scoped_variables", after=True)
    def root_state_model_changed(self, model, prop_name, info):
        self.notify_method_after_change("root_state", info['instance'], info['method_name'], None, kwargs=info)


from mvc.models import ContainerStateModel, StateModel, TransitionModel, DataFlowModel
from gtkmvc import Observable


#===========================================================
#    Selection
#===========================================================
class Selection(Observable):
    """
    @brief Contains the selected item (States, Transitions and Data Flows)
    of a state_machine

    :param set __selected: The state machine elements selected
    """
    __selected = None

    #===========================================================
    def __init__(self):
        Observable.__init__(self)

        self.__selected = set()

    #===========================================================
    @Observable.observed
    def add(self, item):
        self.__selected.add(item)

    #===========================================================
    @Observable.observed
    def remove(self, item):
        self.__selected.remove(item)

    #===========================================================
    @Observable.observed
    def append(self, selection):
        # for item in selection:
        #     item.select()

        self.__selected.update(selection)

    #===========================================================
    @Observable.observed
    def set(self, selection):
        # for item in self.__selected:
        #     item.deselect()

        self.__selected.clear()
        self.__selected.update(selection)

        # for item in self.__selected:
        #     item.select()

    #===========================================================
    def __iter__(self):
        return self.__selected.__iter__()

    #===========================================================
    def __len__(self):
        return len(self.__selected)

    #===========================================================
    def get_states(self):
        return [s for s in self.__selected if isinstance(s, StateModel)]

    #===========================================================
    def get_num_states(self):
        return sum((1 for s in self.__selected if isinstance(s, StateModel)))

    #===========================================================
    def get_transitions(self):
        return [s for s in self.__selected if isinstance(s, TransitionModel)]

    #===========================================================
    def get_num_transitions(self):
        return sum((1 for s in self.__selected if isinstance(s, TransitionModel)))

    #===========================================================
    def get_data_flows(self):
        return [s for s in self.__selected if isinstance(s, DataFlowModel)]

    #===========================================================
    def get_num_data_flows(self):
        return sum((1 for s in self.__selected if isinstance(s, DataFlowModel)))

    #===========================================================
    def get_first(self):
        try:
            return self.__selected.__iter__().next()
        except:
            return None

    #===========================================================
    @Observable.observed
    def clear(self):
        self.set([])

    #===========================================================
    def get_selected_state(self):
        selected_states = self.get_states()
        if len(selected_states) != 1:
            return None
        else:
            return selected_states[0]
