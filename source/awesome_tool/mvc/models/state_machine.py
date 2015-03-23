from gtkmvc import ModelMT, Observable
from enum import Enum

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
    @ModelMT.observe("outcomes", before=True)
    @ModelMT.observe("is_start", before=True)
    @ModelMT.observe("states", before=True)
    @ModelMT.observe("transitions", before=True)
    @ModelMT.observe("data_flows", before=True)
    @ModelMT.observe("input_data_ports", before=True)
    @ModelMT.observe("scoped_variables", before=True)
    def root_state_model_before_change(self, model, prop_name, info):
        if not self._list_modified(prop_name, info):
            self.state_machine.root_state_before_change(model=info['model'], prop_name=info['prop_name'],
                                                        instance=info['instance'],
                                                        method_name=info['method_name'], args=info['args'],
                                                        kwargs=info['kwargs'])


    @ModelMT.observe("state", after=True)
    @ModelMT.observe("outcomes", after=True)
    @ModelMT.observe("is_start", after=True)
    @ModelMT.observe("states", after=True)
    @ModelMT.observe("transitions", after=True)
    @ModelMT.observe("data_flows", after=True)
    @ModelMT.observe("input_data_ports", after=True)
    @ModelMT.observe("output_data_ports", after=True)
    @ModelMT.observe("scoped_variables", after=True)
    def root_state_model_after_change(self, model, prop_name, info):
        if not self._list_modified(prop_name, info):
            self.state_machine.root_state_after_change(model=info['model'], prop_name=info['prop_name'],
                                                       instance=info['instance'],
                                                       method_name=info['method_name'], result=info['result'],
                                                       args=info['args'], info=info['kwargs'])

    @staticmethod
    def _list_modified(prop_name, info):
        """Check whether the given operation is a list operation

        The function checks whether the property that has been changed is a list. If so, the operation is investigated
        further. If the operation is a basic list operation, the function return True.
        :param prop_name: The property that was changed
        :param info: Dictionary with information about the operation
        :return: True if the operation was a list operation, False else
        """
        if prop_name in ["states", "transitions", "data_flows", "input_data_ports", "output_data_ports",
                         "scoped_variables"]:
            if info['method_name'] in ["append", "extend", "insert", "pop", "remove", "reverse", "sort",
                                       "__delitem__", "__setitem__"]:
                return True
        return False

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
        if not isinstance(selection, list):
            selection = [selection]
        self.__selected.update(selection)

    def __iter__(self):
        return self.__selected.__iter__()

    def __len__(self):
        return len(self.__selected)

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


ClipboardType = Enum('CLIPBOARD_TYPE', 'CUT COPY')


class Clipboard(Observable):
    """
    A class to hold a selection for later usage e.g. triggered by copy, or cut. Later usage is e.g. paste.
    """
    def __init__(self):
        Observable.__init__(self)
        self._selection = None
        self.selection = Selection()

        self._state_machine_id = None
        self._clipboard_type = None

    def __str__(self):
        return "Clipboard:\nselection: %s\nstate_machine_id: %s\nclipboard_type: %s" % (str(self.selection),
                                                                                     str(self.state_machine_id),
                                                                                     str(self.clipboard_type))

    def set_selection(self, selection):
        self.selection.set(selection)

    @property
    def selection(self):
        """Property for the _selection field

        """
        return self._selection

    @selection.setter
    @Observable.observed
    def selection(self, selection):
        if not isinstance(selection, Selection):
            raise TypeError("selection must be of type Selection")
        self._selection = selection

    @property
    def state_machine_id(self):
        """Property for the _state_machine_id field

        """
        return self._state_machine_id

    @state_machine_id.setter
    @Observable.observed
    def state_machine_id(self, state_machine_id):
        if not isinstance(state_machine_id, int):
            raise TypeError("state_machine_id must be of type int")
        self._state_machine_id = state_machine_id

    @property
    def clipboard_type(self):
        """Property for the _clipboard_type field

        """
        return self._clipboard_type

    @clipboard_type.setter
    @Observable.observed
    def clipboard_type(self, clipboard_type):
        if not isinstance(clipboard_type, ClipboardType):
            raise TypeError("clipboard_type must be of type ClipBoardType")
        self._clipboard_type = clipboard_type

