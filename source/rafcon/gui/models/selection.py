# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gtkmvc import ModelMT, Signal

from rafcon.core.states.state import State
from rafcon.core.state_elements.outcome import Outcome
from rafcon.core.state_elements.data_port import InputDataPort, OutputDataPort
from rafcon.core.state_elements.scope import ScopedVariable
from rafcon.core.state_elements.transition import Transition
from rafcon.core.state_elements.data_flow import DataFlow

from rafcon.gui.models import AbstractStateModel
from rafcon.gui.models.signals import SelectionChangedSignalMsg
from rafcon.gui.utils import constants

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


def updates_selection(update_selection):
    """ Decorator indicating that the decorated method could change the selection"""
    def handle_update(self, *args, **kwargs):
        """Check for changes in the selection

        If the selection is changed by the decorated method, the internal core element lists are updated and a signal is
        emitted with the old and new selection as well as the name of the method that caused the change..
        """
        old_selection = self.get_all()
        update_selection(self, *args, **kwargs)
        new_selection = self.get_all()

        affected_models = old_selection ^ new_selection
        if len(affected_models) != 0:  # The selection was updated
            # Observe models in the selection
            deselected_models = old_selection - new_selection
            selected_models = new_selection - old_selection
            map(self.relieve_model, deselected_models)
            map(self.observe_model, selected_models)

            # Maintain internal lists for fast access
            self.update_core_element_lists()

            # Send notifications about changes
            affected_classes = set(type(model.core_element) for model in affected_models)
            msg_namedtuple = SelectionChangedSignalMsg(update_selection.__name__, new_selection, old_selection,
                                                       affected_classes)
            self.selection_changed_signal.emit(msg_namedtuple)
            if self.parent_signal is not None:
                self.parent_signal.emit(msg_namedtuple)
    return handle_update


class Selection(ModelMT):
    """ This class contains the selected models of a state_machine """
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

    def __str__(self):
        return_string = "Selected: "
        for item in self.__selected:
            return_string = "%s, %s" % (return_string, str(item))
        return return_string

    @updates_selection
    def add(self, models):
        """ Adds the passed model(s) to the selection"""
        if not hasattr(models, "__iter__"):
            models = [models]
        self.__selected.update(models)
        self.__selected = reduce_to_parent_states(self.__selected)

    @updates_selection
    def remove(self, models):
        """ Removed the passed model(s) from the selection"""
        if not hasattr(models, "__iter__"):
            models = [models]
        for model in models:
            if model in self.__selected:
                self.__selected.remove(model)

    @updates_selection
    def set(self, models):
        """ Sets the selection to the passed model(s) """
        self.__selected.clear()
        # Do not add None values to selection
        if models is None:
            models = []

        if not hasattr(models, "__iter__"):
            models = [models]
            models = reduce_to_parent_states(models)
        self.__selected.update(models)

    @updates_selection
    def clear(self):
        """ Removes all models from the selection """
        self.__selected.clear()

    @updates_selection
    def handle_selection_of_core_class_elements(self, core_class, models):
        """Handles the selection for widgets maintaining lists of a specific `core_class` elements

        If a widgets hold a TreeStore with elements of a specific `core_class`, the local selection of that element
        type is handled by that widget. This method is called to integrate the local selection with the overall
        selection of the state machine.

        If no modifier key (indicating to extend the selection) is pressed, the state machine selection is set to the
        passed selection. If the selection is to be extended, the state machine collection will consist of the widget
        selection plus all previously selected elements not having the core class `core_class`.

        :param State | StateElement core_class: The core class of the elements the widget handles
        :param models: The list of models that are currently being selected locally
        """
        from rafcon.gui.singleton import main_window_controller
        currently_pressed_keys = main_window_controller.currently_pressed_keys if main_window_controller else set()
        if any(key in currently_pressed_keys for key in [constants.EXTEND_SELECTION_KEY,
                                                         constants.EXTEND_SELECTION_KEY_ALT]):
            self.__selected.difference_update(self.get_selected_elements_of_core_class(core_class))
        else:
            self.__selected.clear()
        self.__selected.update(models)

    def __iter__(self):
        return self.__selected.__iter__()

    def __len__(self):
        return len(self.__selected)

    def __contains__(self, item):
        return item in self.__selected

    def __getitem__(self, key):
        return [s for s in self.__selected][key]

    def update_core_element_lists(self):
        """ Maintains inner lists of selected elements with a specific core element class """
        def get_selected_elements_of_core_class(core_class):
            return set(element for element in self.__selected if isinstance(element.core_element, core_class))
        self._states = get_selected_elements_of_core_class(State)
        self._transitions = get_selected_elements_of_core_class(Transition)
        self._data_flows = get_selected_elements_of_core_class(DataFlow)
        self._input_data_ports = get_selected_elements_of_core_class(InputDataPort)
        self._scoped_variables = get_selected_elements_of_core_class(OutputDataPort)
        self._output_data_ports = get_selected_elements_of_core_class(ScopedVariable)
        self._outcomes = get_selected_elements_of_core_class(Outcome)

    @property
    def states(self):
        """Returns all selected states

        :return: Subset of the selection, only containing states
        :rtype: set
        """
        return self._states

    @property
    def transitions(self):
        """Returns all selected transitions

        :return: Subset of the selection, only containing transitions
        :rtype: set
        """
        return self._transitions

    @property
    def data_flows(self):
        """Returns all selected data flows

        :return: Subset of the selection, only containing data flows
        :rtype: set
        """
        return self._data_flows

    @property
    def outcomes(self):
        """Returns all selected outcomes

        :return: Subset of the selection, only containing outcomes
        :rtype: set
        """
        return self._outcomes

    @property
    def input_data_ports(self):
        """Returns all selected input data ports

        :return: Subset of the selection, only containing input data ports
        :rtype: set
        """
        return self._input_data_ports

    @property
    def output_data_ports(self):
        """Returns all selected output data ports

        :return: Subset of the selection, only containing output data ports
        :rtype: set
        """
        return self._output_data_ports

    @property
    def scoped_variables(self):
        """Returns all selected scoped variables

        :return: Subset of the selection, only containing scoped variables
        :rtype: set
        """
        return self._scoped_variables

    def get_selected_elements_of_core_class(self, core_element_type):
        """Returns all selected elements having the specified `core_element_type` as state element class

        :return: Subset of the selection, only containing elements having `core_element_type` as state element class
        :rtype: set
        """
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
        raise RuntimeError("Invalid core element type: " + core_element_type)

    def is_selected(self, model):
        """Checks whether the given model is selected

        :param model:
        :return: True if the model is within the selection, False else
        :rtype: bool
        """
        if model is None:
            return len(self.__selected) == 0
        return model in self.__selected

    def get_all(self):
        """Return a copy of the selection

        :return: Copy of the set of selected elements
        :rtype: set
        """
        return set(s for s in self.__selected)

    def get_selected_state(self):
        """Return the first state within the selection

        :return: First state within the selection or None if there is none
        :rtype: AbstractStateModel
        """
        if not self.states:
            return None
        else:
            return next(iter(self.states))  # sets don't support indexing

    @ModelMT.observe("destruction_signal", signal=True)
    def on_model_destruct(self, destructed_model, signal, info):
        """ Deselect models that are being destroyed """
        self.remove(destructed_model)
