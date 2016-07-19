from copy import copy
from rafcon.utils import log

logger = log.get_logger(__name__)

from enum import Enum
from gtkmvc import Observable
from rafcon.mvc.selection import Selection
from rafcon.mvc.models.container_state import ContainerStateModel
from rafcon.statemachine.id_generator import state_id_generator

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
        self.__create_core_object_copies(selection)

    def cut(self, selection):
        """
        Cuts all selected items and copy them to the clipboard.
        Note: Multi selection is not implemented yet. Only one item allowed right now.
        :param selection: the current selection
        :return:
        """
        self.reset_clipboard()
        self.clipboard_type = ClipboardType.CUT
        self.__create_core_object_copies(selection)

    def prepare_new_copy(self):
        # in future
        # self.state_model_copies[0] = copy(self.state_model_copies[0])
        # self.state_core_object_copies[0] = self.state_model_copies[0].state

        # at the moment
        # old = self.state_core_object_copies[0]
        self.state_core_object_copies[0] = copy(self.state_core_object_copies[0])
        # assert not id(old) == id(self.state_core_object_copies[0])
        # assert old == self.state_core_object_copies[0]

    def paste(self, target_state_m):
        assert isinstance(target_state_m, ContainerStateModel)

        # check if the clipboard is valid
        if self.get_number_selected_items() > 1:
            logger.error("Only one single item is allowed to be copied yet!")
            return None, None
        if not len(self.get_states()) == 1:
            logger.error("Only states are allowed to be copied yet!")
            return None, None

        target_state = target_state_m.state

        orig_state_copy = self.state_core_object_copies[0]
        self.prepare_new_copy()  # threaded in future
        orig_state_copy_m = self.state_model_copies[0]

        # secure that state_id is not target state state_id or of one state in its sub-hierarchy level
        old_state_id = orig_state_copy.state_id
        new_state_id = old_state_id
        while new_state_id in target_state.states.iterkeys() or new_state_id == target_state.state_id:
            new_state_id = state_id_generator()

        if not new_state_id == old_state_id:
            logger.debug("Change state_id of pasted state from '{0}' to '{1}'".format(old_state_id, new_state_id))
            orig_state_copy.change_state_id(new_state_id)

        target_state.add_state(orig_state_copy)

        # TODO define a way to hand model copy directly to target model to save resources (takes half of the copy time)
        # The models can be pre-generated in threads while editing is still possible -> scales better
        new_state_copy_m = target_state_m.states[orig_state_copy.state_id]

        new_state_copy_m.copy_meta_data_from_state_m(orig_state_copy_m)

        if self.clipboard_type is ClipboardType.CUT:
            # delete the original state
            # Note: change this when implementing multi selection
            # TODO cut now can be realized directly after the copy has been generated
            if len(self.selected_state_models) == 1:
                source_state_id = self.selected_state_models[0].state.state_id
                parent_of_source_state = self.selected_state_models[0].state.parent
                parent_of_source_state.remove_state(source_state_id)
                self.selected_state_models.remove(self.selected_state_models[0])

        return new_state_copy_m, orig_state_copy_m

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

    def __create_core_object_copies(self, selection):
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

            # direct model copy -> copy meta data and core object
            state_model_copy = copy(state_model)
            self.state_core_object_copies.append(state_model_copy.state)
            self.state_model_copies.append(state_model_copy)

        # TODO: create transition copies, only relevant in multi-selection scenario
        # TODO: create data flow copies, only relevant in multi-selection scenario


# To enable copy, cut and paste between state machines a global clipboard is used
global_clipboard = Clipboard()
