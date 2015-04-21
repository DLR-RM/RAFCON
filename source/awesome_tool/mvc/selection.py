from gtkmvc import Observable
from awesome_tool.mvc.models import StateModel, TransitionModel, DataFlowModel

def reduce_to_parent_states(models):
    models_to_remove = []
    for model in models:
        parent_m = model.parent
        while parent_m is not None:
            if parent_m in models:
                models_to_remove.append(model)
                break
            parent_m = parent_m.parent
    for model in models_to_remove:
        models.remove(model)
    return models

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

    def __str__(self):
        return_string = "Selected: "
        for item in self.__selected:
            return_string = "%s, %s" % (return_string, str(item))
        return return_string

    @Observable.observed
    def add(self, item):
        self.__selected.add(item)
        self.__selected = reduce_to_parent_states(self.__selected)

    @Observable.observed
    def remove(self, item):
        if item in self.__selected:
            self.__selected.remove(item)

    @Observable.observed
    def append(self, selection):
        self.__selected.update(selection)
        self.__selected = reduce_to_parent_states(self.__selected)

    @Observable.observed
    def set(self, selection):
        self.__selected.clear()
        if not isinstance(selection, list):
            selection = [selection]
        else:
            selection = StateMachineHelper.reduce_to_parent_states(selection)
        self.__selected.update(selection)

    def __iter__(self):
        return self.__selected.__iter__()

    def __len__(self):
        return len(self.__selected)

    def is_selected(self, item):
        return item in self.__selected

    def get_number_of_selected_items(self):
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
