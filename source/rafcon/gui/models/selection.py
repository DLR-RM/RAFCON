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

from gtkmvc3.model_mt import ModelMT
from gtkmvc3.observable import Signal
from past.builtins import map
from builtins import next, str

from rafcon.core.states.state import State
from rafcon.core.state_elements.logical_port import Income, Outcome
from rafcon.core.state_elements.data_port import InputDataPort, OutputDataPort
from rafcon.core.state_elements.scope import ScopedVariable
from rafcon.core.state_elements.transition import Transition
from rafcon.core.state_elements.data_flow import DataFlow

from rafcon.gui.models import AbstractStateModel
from rafcon.gui.models.state_element import StateElementModel
from rafcon.gui.models.signals import SelectionChangedSignalMsg, FocusSignalMsg
from rafcon.gui.utils import constants

from rafcon.utils import log
logger = log.get_logger(__name__)


def reduce_to_parent_states(models):
    """Remove all state models that also have their parents in the list

    The function filters the list of models, so that for no model in the list, one of it (grand-)parents is also in
    the list. E.g. if the input models consists of a hierarchy state with two of its child states, the resulting list
    only contains the hierarchy state.

    :param set models: The set of selected models
    :return: The reduced set of selected models
    :rtype: set
    """
    models = set(models)  # Ensure that models is a set and that we do not operate on the parameter itself
    models_to_remove = set()
    # check all models
    for model in models:
        parent_m = model.parent
        # check if any (grand-)parent is already in the selection, if so, remove the child
        while parent_m is not None:
            if parent_m in models:
                models_to_remove.add(model)
                break
            parent_m = parent_m.parent
    for model in models_to_remove:
        models.remove(model)
    if models_to_remove:
        logger.debug("The selection has been reduced, as it may not contain elements whose children are also selected")
    return models


def updates_selection(update_selection):
    """ Decorator indicating that the decorated method could change the selection"""
    def handle_update(selection, *args, **kwargs):
        """Check for changes in the selection

        If the selection is changed by the decorated method, the internal core element lists are updated and a signal is
        emitted with the old and new selection as well as the name of the method that caused the change..
        """
        old_selection = selection.get_all()
        update_selection(selection, *args, **kwargs)
        new_selection = selection.get_all()

        affected_models = old_selection ^ new_selection
        if len(affected_models) != 0:  # The selection was updated
            deselected_models = old_selection - new_selection
            selected_models = new_selection - old_selection
            map(selection.relieve_model, deselected_models)
            map(selection.observe_model, selected_models)

            # Maintain internal lists for fast access
            selection.update_core_element_lists()

            # Clear focus if no longer in selection
            if selection.focus and selection.focus not in new_selection:
                del selection.focus

            # Send notifications about changes
            affected_classes = set(model.core_element.__class__ for model in affected_models)
            msg_namedtuple = SelectionChangedSignalMsg(update_selection.__name__, new_selection, old_selection,
                                                       affected_classes)
            selection.selection_changed_signal.emit(msg_namedtuple)
            if selection.parent_signal is not None:
                selection.parent_signal.emit(msg_namedtuple)
    return handle_update


def extend_selection():
    """Checks is the selection is to be extended

    The selection is to be extended, if a special modifier key (typically <Ctrl>) is being pressed.

    :return: If to extend the selection
    :rtype: True
    """
    from rafcon.gui.singleton import main_window_controller
    currently_pressed_keys = main_window_controller.currently_pressed_keys if main_window_controller else set()
    if any(key in currently_pressed_keys for key in [constants.EXTEND_SELECTION_KEY,
                                                     constants.EXTEND_SELECTION_KEY_ALT]):
        return True
    return False


class Selection(ModelMT):
    """ This class contains the selected models of a state_machine """
    _selected = None
    _input_data_ports = None
    _output_data_ports = None
    _scoped_variables = None
    _incomes = None
    _outcomes = None
    _data_flows = None
    _transitions = None
    _states = None

    _focus = None

    selection_changed_signal = Signal()
    focus_signal = Signal()

    __observables__ = ("selection_changed_signal", "focus_signal")

    def __init__(self, parent_signal=None):
        ModelMT.__init__(self)

        self._selected = set()
        self._incomes = set()
        self._input_data_ports = set()
        self._output_data_ports = set()
        self._outcomes = set()
        self._data_flows = set()
        self._transitions = set()
        self._states = set()
        self._scoped_variables = set()
        self.selection_changed_signal = Signal()
        self.focus_signal = Signal()
        self.parent_signal = parent_signal

    def __str__(self):
        return_string = "Selected: "
        for item in self._selected:
            return_string = "%s, %s" % (return_string, str(item))
        return return_string

    def _check_model_types(self, models):
        """ Check types of passed models for correctness and in case raise exception

        :rtype: set
        :returns: set of models that are valid for the class"""
        if not hasattr(models, "__iter__"):
            models = {models}
        if not all([isinstance(model, (AbstractStateModel, StateElementModel)) for model in models]):
            raise TypeError("The selection supports only models with base class AbstractStateModel or "
                            "StateElementModel, see handed elements {0}".format(models))
        return models if isinstance(models, set) else set(models)

    @updates_selection
    def add(self, models):
        """ Adds the passed model(s) to the selection"""
        if models is None:
            return

        models = self._check_model_types(models)
        self._selected.update(models)
        self._selected = reduce_to_parent_states(self._selected)

    @updates_selection
    def remove(self, models):
        """ Removed the passed model(s) from the selection"""
        models = self._check_model_types(models)
        for model in models:
            if model in self._selected:
                self._selected.remove(model)

    @updates_selection
    def set(self, models):
        """ Sets the selection to the passed model(s) """
        # Do not add None values to selection
        if models is None:
            models = set()

        models = self._check_model_types(models)
        if len(models) > 1:
            models = reduce_to_parent_states(models)

        self._selected = set(models)

    @updates_selection
    def clear(self):
        """ Removes all models from the selection """
        self._selected.clear()

    @updates_selection
    def handle_prepared_selection_of_core_class_elements(self, core_class, models):
        """Handles the selection for TreeStore widgets maintaining lists of a specific `core_class` elements

        If widgets hold a TreeStore with elements of a specific `core_class`, the local selection of that element
        type is handled by that widget. This method is called to integrate the local selection with the overall
        selection of the state machine.

        If no modifier key (indicating to extend the selection) is pressed, the state machine selection is set to the
        passed selection. If the selection is to be extended, the state machine collection will consist of the widget's
        selection plus all previously selected elements not having the core class `core_class`.

        :param State | StateElement core_class: The core class of the elements the widget handles
        :param models: The list of models that are currently being selected locally
        """
        if extend_selection():
            self._selected.difference_update(self.get_selected_elements_of_core_class(core_class))
        else:
            self._selected.clear()

        models = self._check_model_types(models)
        if len(models) > 1:
            models = reduce_to_parent_states(models)

        self._selected.update(models)

    @updates_selection
    def handle_new_selection(self, models):
        """Handles the selection for generic widgets

        This is a helper method for generic widgets that want to modify the selection. These widgets can pass a list
        of newly selected (or clicked on) models.

        The method looks at the previous selection, the passed models and the list of pressed (modifier) keys:

        * If no modifier key is pressed, the previous selection is cleared and the new selection is set to the passed
          models
        * If the extend-selection modifier key is pressed, elements of `models` that are _not_ in the previous
          selection are selected, those that are in the previous selection are deselected

        :param models: The list of models that are newly selected/clicked on
        """
        models = self._check_model_types(models)

        if extend_selection():
            already_selected_elements = models & self._selected
            newly_selected_elements = models - self._selected
            self._selected.difference_update(already_selected_elements)
            self._selected.update(newly_selected_elements)
        else:
            self._selected = models
        self._selected = reduce_to_parent_states(self._selected)

    @property
    def focus(self):
        """ Returns the currently focused element """
        return self._focus

    @focus.setter
    def focus(self, model):
        """Sets the passed model as focused element

        :param ModelMT model: The element to be focused
        """
        if model is None:
            del self.focus
            return

        self._check_model_types(model)
        self.add(model)
        focus_msg = FocusSignalMsg(model, self._focus)
        self._focus = model
        self._selected.add(model)
        self._selected = reduce_to_parent_states(self._selected)
        self.focus_signal.emit(focus_msg)

    @focus.deleter
    def focus(self):
        """ Unsets the focused element """
        focus_msg = FocusSignalMsg(None, self._focus)
        self._focus = None
        self.focus_signal.emit(focus_msg)

    def __iter__(self):
        return self._selected.__iter__()

    def __len__(self):
        return len(self._selected)

    def __contains__(self, item):
        return item in self._selected

    def __getitem__(self, key):
        return [s for s in self._selected][key]

    def update_core_element_lists(self):
        """ Maintains inner lists of selected elements with a specific core element class """
        def get_selected_elements_of_core_class(core_class):
            return set(element for element in self._selected if isinstance(element.core_element, core_class))
        self._states = get_selected_elements_of_core_class(State)
        self._transitions = get_selected_elements_of_core_class(Transition)
        self._data_flows = get_selected_elements_of_core_class(DataFlow)
        self._input_data_ports = get_selected_elements_of_core_class(InputDataPort)
        self._output_data_ports = get_selected_elements_of_core_class(OutputDataPort)
        self._scoped_variables = get_selected_elements_of_core_class(ScopedVariable)
        self._incomes = get_selected_elements_of_core_class(Income)
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
    def incomes(self):
        """Returns all selected incomes

        :return: Subset of the selection, only containing incomes
        :rtype: set
        """
        return self._incomes

    @property
    def income(self):
        """Alias for ``incomes()``"""
        return self.incomes

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
            return len(self._selected) == 0
        return model in self._selected

    def get_all(self):
        """Return a copy of the selection

        :return: Copy of the set of selected elements
        :rtype: set
        """
        return set(s for s in self._selected)

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
