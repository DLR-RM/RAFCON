from gtkmvc import ModelMT, Signal

from rafcon.core.states.state import State
from rafcon.core.state_elements.outcome import Outcome
from rafcon.core.state_elements.data_port import InputDataPort, OutputDataPort
from rafcon.core.state_elements.scope import ScopedVariable
from rafcon.core.state_elements.transition import Transition
from rafcon.core.state_elements.data_flow import DataFlow

from rafcon.gui.models import AbstractStateModel, TransitionModel, DataFlowModel, DataPortModel, OutcomeModel, \
    ScopedVariableModel
from rafcon.gui.models.signals import SelectionChangedSignalMsg

from rafcon.utils import log
logger = log.get_logger(__name__)


def reduce_to_parent_states(models):
    """Remove all models of states that have a state model with parent relation in the list

    The function reduce multiple selection queueing over multiple hierarchy levels with respect to the parent tree
    to the parent model. All hierarchy level and its parent relations are checked recursively.
    This helps to have more reasonable selection for states if using rubber band selections.

    :param models:
    :return:
    """
    models_to_remove = []
    # check all models
    for model in models:
        parent_m = model.parent
        # while parent model is not None, a state model and in selection list then remove this child model
        while parent_m is not None and isinstance(model, AbstractStateModel):
            if parent_m in models:
                models_to_remove.append(model)
                logger.debug("Parent state selection -> the model of state is removed from selection -> state {0}"
                             "".format(model.state))
                break
            parent_m = parent_m.parent
    for model in models_to_remove:
        models.remove(model)
    return models


class Selection(ModelMT):
    """This class contains the selected item (States, Transitions and Data Flows) of a state_machine

    """
    __selected = None
    _input_data_ports = None
    _output_data_ports = None
    _scoped_variables = None
    _outcomes = None
    _data_flows = None
    _transitions = None
    _states = None
    selection_changed_signal = Signal()

    __observables__ = ("selection_changed_signal", )

    def __init__(self, parent_signal=None):
        ModelMT.__init__(self)

        self.__selected = set()
        self._input_data_ports = []
        self._output_data_ports = []
        self._outcomes = []
        self._data_flows = []
        self._transitions = []
        self._states = []
        self._scoped_variables = []
        self.selection_changed_signal = Signal()
        self.parent_signal = parent_signal
        # flag to enable new method to use list updates -> cause additional but unique notifications -> support for better code
        self.__with_updates = True

    def __str__(self):
        return_string = "Selected: "
        for item in self.__selected:
            return_string = "%s, %s" % (return_string, str(item))
        return return_string

    def add(self, item):
        self.__selected.add(item)
        self.__selected = reduce_to_parent_states(self.__selected)
        self.__update(method_name='add')

    def remove(self, item):
        if item in self.__selected:
            self.__selected.remove(item)
            self.__update(method_name='remove')
        # else:
        #     logger.warning("Can not remove item not in selection: {0}".format(item))

    def append(self, selection):
        self.__selected.update(selection)
        self.__selected = reduce_to_parent_states(self.__selected)
        self.__update(method_name='append')

    def set(self, selection):
        self.__selected.clear()
        # Do not add None values to selection
        if not selection:
            return
        if not isinstance(selection, list):
            selection = [selection]
        else:
            selection = reduce_to_parent_states(selection)
        self.__selected.update(selection)
        self.__update(method_name='set')

    def __iter__(self):
        return self.__selected.__iter__()

    def __len__(self):
        return len(self.__selected)

    def __contains__(self, item):
        return item in self.__selected

    def __getitem__(self, key):
        return [s for s in self.__selected][key]

    def __update(self, method_name):
        core_element_types_of_changed_lists = set()
        if not self._states == self.get_states():
            self._states = self.get_states()
            core_element_types_of_changed_lists.add(State)
        if not self._transitions == self.get_transitions():
            self._transitions = self.get_transitions()
            core_element_types_of_changed_lists.add(Transition)
        if not self._data_flows == self.get_data_flows():
            self._data_flows = self.get_data_flows()
            core_element_types_of_changed_lists.add(DataFlow)
        if not self._input_data_ports == self.get_input_data_ports():
            self._input_data_ports = self.get_input_data_ports()
            core_element_types_of_changed_lists.add(InputDataPort)
        if not self._output_data_ports == self.get_output_data_ports():
            self._output_data_ports = self.get_output_data_ports()
            core_element_types_of_changed_lists.add(OutputDataPort)
        if not self._scoped_variables == self.get_scoped_variables():
            self._scoped_variables = self.get_scoped_variables()
            core_element_types_of_changed_lists.add(ScopedVariable)
        if not self._outcomes == self.get_outcomes():
            self._outcomes = self.get_outcomes()
            core_element_types_of_changed_lists.add(Outcome)

        if core_element_types_of_changed_lists:
            # emit selection changed signal
            msg_namedtuple = SelectionChangedSignalMsg(method_name, core_element_types_of_changed_lists)
            self.selection_changed_signal.emit(msg_namedtuple)
            if self.parent_signal is not None:
                self.parent_signal.emit(msg_namedtuple)

    @property
    def states(self):
        return self._states

    @property
    def transitions(self):
        return self._transitions

    @property
    def data_flows(self):
        return self._data_flows

    @property
    def outcomes(self):
        return self._outcomes

    @property
    def input_data_ports(self):
        return self._input_data_ports

    @property
    def output_data_ports(self):
        return self._output_data_ports

    @property
    def scoped_variables(self):
        return self._scoped_variables

    def get_selection_of_core_element_type(self, core_element_type):
        if core_element_type is Outcome:
            return self.outcomes
        elif core_element_type is InputDataPort:
            return self.input_data_ports
        elif core_element_type is OutputDataPort:
            return self.output_data_ports
        elif core_element_type is ScopedVariable:
            return self.scoped_variables
        elif core_element_type is Transition:
            return self.transitions
        elif core_element_type is DataFlow:
            return self.data_flows
        elif core_element_type is State:
            return self.states
        else:
            logger.warning("There is no core element type '{0}' which selection could be requested".format(core_element_type))

    def is_selected(self, item):
        if item is None:
            return len(self.__selected) == 0
        return item in self.__selected

    def get_all(self):
        return [s for s in self.__selected]

    def get_states(self):
        return [s for s in self.__selected if isinstance(s, AbstractStateModel)]

    def get_num_states(self):
        return sum((1 for s in self.__selected if isinstance(s, AbstractStateModel)))

    def get_transitions(self):
        return [s for s in self.__selected if isinstance(s, TransitionModel)]

    def get_num_transitions(self):
        return sum((1 for s in self.__selected if isinstance(s, TransitionModel)))

    def get_data_flows(self):
        return [s for s in self.__selected if isinstance(s, DataFlowModel)]

    def get_outcomes(self):
        return [s for s in self.__selected if isinstance(s, OutcomeModel)]

    def get_num_outcomes(self):
        return sum((1 for s in self.__selected if isinstance(s, OutcomeModel)))

    def get_num_data_flows(self):
        return sum((1 for s in self.__selected if isinstance(s, DataFlowModel)))

    def get_input_data_ports(self):
        return [s for s in self.__selected if isinstance(s, DataPortModel) and isinstance(s.data_port, InputDataPort)]

    def get_num_input_data_ports(self):
        return sum((1 for s in self.__selected if isinstance(s, DataPortModel) and isinstance(s.data_port, InputDataPort)))

    def get_output_data_ports(self):
        return [s for s in self.__selected if isinstance(s, DataPortModel) and isinstance(s.data_port, OutputDataPort)]

    def get_num_output_data_ports(self):
        return sum((1 for s in self.__selected if isinstance(s, DataPortModel) and isinstance(s.data_port, OutputDataPort)))

    def get_scoped_variables(self):
        return [s for s in self.__selected if isinstance(s, ScopedVariableModel)]

    def get_num_scoped_variables(self):
        return sum((1 for s in self.__selected if isinstance(s, ScopedVariableModel)))

    def clear(self):
        self.set([])
        self.__update(method_name='clear')

    def get_selected_state(self):
        selected_states = self.get_states()
        if not selected_states:
            return None
        else:
            return selected_states[0]
