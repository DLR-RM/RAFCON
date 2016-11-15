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
    """A class to hold models and selection for later usage in cut/paste or copy/paste actions.
    In cut/paste action the selection stored is used while later paste. In a copy/paste actions
    """

    def __init__(self):
        Observable.__init__(self)
        self._selection = None

        self.selected_state_models = []
        # only to save the meta data
        self.state_model_copies = []

        self.selected_transition_models = []
        self.transition_model_copies = []

        self.selected_data_flow_models = []
        self.data_flow_model_copies = []

        self.selected_outcome_models = []
        self.outcome_model_copies = []

        self.selected_input_data_port_models = []
        self.input_data_port_model_copies = []

        self.selected_output_data_port_models = []
        self.output_data_port_model_copies = []

        self.selected_scoped_variable_models = []
        self.scoped_variable_model_copies = []

        # self._state_machine_id = None
        self._clipboard_type = None

    def __str__(self):
        return "Clipboard:\nselection: %s\nstate_machine_id: %s\nclipboard_type: %s" % (str(self.selection),
                                                                                        str(self.state_machine_id),
                                                                                        str(self.clipboard_type))

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

    def copy(self, selection):
        """
        Copies all selected items to the clipboard.
        Note: Multi selection is not implemented yet. Only one item allowe right now.
        :param selection: the current selection
        :return:
        """
        assert isinstance(selection, Selection)
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
        assert isinstance(selection, Selection)
        self.reset_clipboard()
        self.clipboard_type = ClipboardType.CUT
        self.__create_core_object_copies(selection)

    def prepare_new_copy(self):
        # in future
        self.state_model_copies = [copy(model) for model in self.state_model_copies]
        # self.state_core_object_copies[0] = self.state_model_copies[0].state

        # at the moment
        # old = self.state_core_object_copies[0]
        # self.state_core_object_copies = [copy(core_object) for core_object in self.state_core_object_copies]
        # assert not id(old) == id(self.state_core_object_copies[0])
        # assert old == self.state_core_object_copies[0]

    def paste(self, target_state_m, cursor_position=None, limited=None):
        """Paste objects to target state

        The method checks whether the target state is a execution state or a container state and inserts respective
        elements and notifies the user if the parts can not be insert to the target state.
        - for ExecutionStates outcomes, input- and output-data ports can be insert (outcomes outcome_id > 0 are ignored)
        - for ContainerState additional states, scoped variables and data flows and/or transitions (if related) can be insert

        Related data flows and transitions are determined by origin and target keys and respective objects which has to
        be in the state machine selection, too. So transitions or data flows without the related objects are not copied.
        :param target_state_m: state in which the copied/cut elements should be insert
        :param cursor_position: cursor position used to adapt meta data positioning of elements e.g states and via points
        :return:
        """
        assert isinstance(target_state_m, ContainerStateModel) # in future Execution states can be used, too
        selection = target_state_m.get_sm_m_for_state_m().selection if target_state_m.get_sm_m_for_state_m() else None

        # update meta data of clipboard elements to adapt for new parent state
        logger.info("PASTE -> meta data adaptation has to be implemented")

        def insert_elements_from_model_copies_list(element_str):
            for orig_m_copy in getattr(self, '{0}_model_copies'.format(element_str)):
                new_m_copy, orig_m_copy = self.insert_state(target_state_m, orig_m_copy)

            return new_m_copy, orig_m_copy


        # if limited:
        #
        # else:
        new_state_copy_m, orig_state_copy_m = insert_elements_from_model_copies_list('state')

        def remove_selected_elements_of_type(element_str, model_attr_str=None, id_attr_str=None):
            model_attr_str = element_str if model_attr_str is None else model_attr_str
            id_attr_str = element_str + '_id' if id_attr_str is None else id_attr_str
            for model in getattr(self, 'selected_{0}_models'.format(element_str)):
                # remove model from selection to avoid conflicts
                # -> selection is not observing state machine changes and state machine model is not updating it
                if model in getattr(selection, element_str + 's'):
                    selection.remove(model)
                # remove element
                element_id = getattr(getattr(model, model_attr_str), id_attr_str)
                parent_of_source_state = getattr(getattr(model, model_attr_str), 'parent')
                getattr(parent_of_source_state, 'remove_{0}'.format(element_str))(element_id)

        if self.clipboard_type is ClipboardType.CUT:
            # delete the original state
            # Note: change this when implementing multi selection
            # TODO cut now can be realized directly after the copy has been generated
            remove_selected_elements_of_type('data_flow')
            remove_selected_elements_of_type('input_data_port', 'data_port', 'data_port_id')
            remove_selected_elements_of_type('output_data_port', 'data_port', 'data_port_id')
            remove_selected_elements_of_type('scoped_variable', 'scoped_variable', 'data_port_id')
            remove_selected_elements_of_type('transition')
            remove_selected_elements_of_type('outcome')
            remove_selected_elements_of_type('state')

        return new_state_copy_m, orig_state_copy_m

    def insert_state(self, target_state_m, orig_state_copy_m):
        target_state = target_state_m.state
        orig_state_copy = orig_state_copy_m.state
        self.prepare_new_copy()  # threaded in future

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
        # TODO -> maybe by simply blind deposit model
        # TODO -> maybe by signal send by target model and catch by clipboard that is registered as observer
        # The models can be pre-generated in threads while editing is still possible -> scales better
        new_state_copy_m = target_state_m.states[orig_state_copy.state_id]

        new_state_copy_m.copy_meta_data_from_state_m(orig_state_copy_m)
        return new_state_copy_m, orig_state_copy_m

    def reset_clipboard(self):
        """ Resets the clipboard, so that old elements do not pollute the new selection that is copied into the clipboard.
        :return:
        """
        # reset states
        self.selected_state_models = []
        # self.state_core_object_copies = []
        self.state_model_copies = []

        # reset transitions
        self.selected_transition_models = []
        # self.transition_core_object_copies = []
        self.transition_model_copies = []

        # reset data flows
        self.selected_data_flow_models = []
        # self.data_flow_core_object_copies = []
        self.data_flow_model_copies = []

    def __create_core_object_copies(self, selection):
        """Copy all elements of a selection.

         The method copies all objects and checks and ignores directly data flows and transitions which are selected
         without related origin or targets. Additional the method copies elements linkage (data flows and transitions)
         if those origins and targets are covered by the selected elements. Therefore the selection it self is manipulated
         to provide direct feedback to the user.

        :param selection: an arbitrary selection, whose elements should be copied
        :return:
        """
        # reduce linkage selection by not fully by selection covered linkage
        logger.info("COPY/CUT -> reduce linkage has to be implemented")
        # extend linkage selection by fully by selection covered linkage
        logger.info("COPY/CUT -> extend linkage has to be implemented")
        # check if all elements that are copied are on one hierarchy level -> TODO or in future are parts of sibling
        logger.info("COPY/CUT -> hierarchy level for copy has to be implemented")

        self.selected_state_models = selection.states
        self.selected_outcome_models = selection.outcomes
        self.selected_input_data_port_models = selection.input_data_ports
        self.selected_output_data_port_models = selection.output_data_ports
        self.selected_transition_models = selection.transitions
        self.selected_data_flow_models = selection.data_flows
        self.selected_scoped_variable_models = selection.scoped_variables

        def copy_list_of_elements_of_type(element_str):
            print element_str, ": ", getattr(self, 'selected_{0}_models'.format(element_str))
            # create state copies
            for model in getattr(self, 'selected_{0}_models'.format(element_str)):

                # direct model copy -> copy meta data and core object
                model_copy = copy(model)
                getattr(self, '{0}_model_copies'.format(element_str)).append(model_copy)

        copy_list_of_elements_of_type('state')
        copy_list_of_elements_of_type('input_data_port')
        copy_list_of_elements_of_type('output_data_port')
        copy_list_of_elements_of_type('scoped_variable')
        copy_list_of_elements_of_type('transition')
        copy_list_of_elements_of_type('data_flow')


# To enable copy, cut and paste between state machines a global clipboard is used
global_clipboard = Clipboard()
