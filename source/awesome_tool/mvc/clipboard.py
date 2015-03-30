from enum import Enum
from gtkmvc import Observable
from awesome_tool.mvc.selection import Selection
from awesome_tool.statemachine.states.state_helper import StateHelper
from awesome_tool.mvc.models.container_state import ContainerStateModel
from awesome_tool.mvc.models.state import StateModel

ClipboardType = Enum('CLIPBOARD_TYPE', 'CUT COPY')


class Clipboard(Observable):
    """
    A class to hold a selection for later usage e.g. triggered by copy, or cut. Later usage is e.g. paste.
    """
    def __init__(self):
        Observable.__init__(self)
        self._selection = None

        self.selected_state_models = []
        self.state_core_object_copies = []
        # only to save the meta data
        self.state_model_copies = []

        self.selected_transition_models = []
        self.transition_core_object_copies = []
        self.transition_model_copies = []

        self.selected_data_flow_models = []
        self.data_flow_core_object_copies = []
        self.data_flow_model_copies = []

        # self._state_machine_id = None
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

    # @property
    # def state_machine_id(self):
    #     """Property for the _state_machine_id field
    #
    #     """
    #     return self._state_machine_id
    #
    # @state_machine_id.setter
    # @Observable.observed
    # def state_machine_id(self, state_machine_id):
    #     if not isinstance(state_machine_id, int):
    #         raise TypeError("state_machine_id must be of type int")
    #     self._state_machine_id = state_machine_id

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

    def get_states(self):
        """
        Get the all state core objects of the clipboard.
        :return: The list of all state core objects.
        """
        return self.state_core_object_copies

    def get_number_selected_items(self):
        """
        Gets the number of all objects in the clipboard.
        :return: The number of all objets.
        """
        return len(self.state_core_object_copies) + \
               len(self.transition_core_object_copies) + \
               len(self.data_flow_core_object_copies)

    def copy(self, selection):
        """
        Copies all selected items to the clipboard.
        Note: Multi selection is not implemented yet. Only one item allowe right now.
        :param selection: the current selection
        :return:
        """
        self.reset_clipboard()
        self.clipboard_type = ClipboardType.COPY
        self.create_core_object_copies(selection)

    def cut(self, selection):
        """
        Cuts all selected items and copy them to the clipboard.
        Note: Multi selection is not implemented yet. Only one item allowed right now.
        :param selection: the current selection
        :return:
        """
        self.reset_clipboard()
        self.clipboard_type = ClipboardType.CUT
        self.create_core_object_copies(selection)

    def reset_clipboard(self):
        """
        Resets the clipboard, so that old elements do not pollute the new selection that is copied into the clipboard.
        :return:
        """
        # reset states
        self.selected_state_models = []
        self.state_core_object_copies = []
        self.state_model_copies = []

        # reset transitions
        self.selected_transition_models = []
        self.transition_core_object_copies = []
        self.transition_model_copies = []

        # reset data flows
        self.selected_data_flow_models = []
        self.data_flow_core_object_copies = []
        self.data_flow_model_copies = []

    def create_core_object_copies(self, selection):
        """
        Copies all elements of a selection.
        Note: Only states are supported right now.
        :param selection: an arbitrary selection, whose elements should be copied
        :return:
        """
        self.selected_state_models = selection.get_states()
        self.selected_transition_models = selection.get_transitions()
        self.selected_data_flow_models = selection.get_data_flows()
        # create state copies
        for state_model in self.selected_state_models:
            # copy core object
            core_state = state_model.state
            state_copy = StateHelper.get_state_copy(core_state)
            self.state_core_object_copies.append(state_copy)

            # copy meta data
            if isinstance(state_model, ContainerStateModel):
                state_model_copy = ContainerStateModel(state_copy)
            elif isinstance(state_model, StateModel):
                state_model_copy = StateModel(state_copy)
            state_model_copy.copy_meta_data_from_state_model(state_model)
            self.state_model_copies.append(state_model_copy)
        # TODO: create transition copies, only relevant in multi-selection scenario
        # TODO: create data flow copies, only relevant in multi-selection scenario

# To enable copy, cut and paste between state machines a global clipboard is used
global_clipboard = Clipboard()