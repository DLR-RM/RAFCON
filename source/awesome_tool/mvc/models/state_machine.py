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

    __observables__ = ("state_machine",)

    def __init__(self, state_machine, meta=None):
        """Constructor
        """
        ModelMT.__init__(self)  # pass columns as separate parameters
        self.register_observer(self)

        assert isinstance(state_machine, StateMachine)

        self.state_machine = state_machine

        self.root_state = ContainerStateModel(self.state_machine.root_state)
        self.selection = Selection()

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

    @ModelMT.observe("state_machine_manager", after=True)
    def model_changed(self, model, prop_name, info):
        pass


from mvc.models import ContainerStateModel, StateModel, TransitionModel, DataFlowModel


#===========================================================
#    Selection
#===========================================================
class Selection(ModelMT):
    """
    @brief Contains the selected item (States, Transitions and Data Flows)
    of a state_machine

    :param set __selected: The state machine elements selected
    """
    __selected = None

    __observable__ = ("__selected",)

    #===========================================================
    def __init__(self):
        ModelMT.__init__(self)
        self.register_observer(self)

        self.__selected = set()

    #===========================================================
    def add(self, item):
        item.select()
        self.__selected.add(item)

    #===========================================================
    def remove(self, item):
        item.deselect()
        self.__selected.remove(item)

    #===========================================================
    def append(self, selection):
        for b in selection:
            b.select()

        self.__selected.update(selection)

    #===========================================================
    def set(self, selection):
        for b in self.__selected:
            b.deselect()

        self.__selected.clear()
        self.__selected.update(selection)

        for b in self.__selected:
            b.select()

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
        return (s for s in self.__selected if not isinstance(s, TransitionModel))

    #===========================================================
    def get_num_transitions(self):
        return sum((1 for s in self.__selected if not isinstance(s, TransitionModel)))

    #===========================================================
    def get_data_flows(self):
        return (s for s in self.__selected if not isinstance(s, DataFlowModel))

    #===========================================================
    def get_num_data_flows(self):
        return sum((1 for s in self.__selected if not isinstance(s, DataFlowModel)))

    #===========================================================
    def get_first(self):
        try:
            return self.__selected.__iter__().next()
        except:
            return None

    #===========================================================
    def clear(self):
        self.set([])

    #===========================================================
    def get_selected_state(self):
        selected_states = self.get_states()
        if len(selected_states) != 1:
            return None
        else:
            return selected_states[0]
