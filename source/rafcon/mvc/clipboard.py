from copy import deepcopy
from rafcon.utils import log

logger = log.get_logger(__name__)

from enum import Enum
from gtkmvc import Observable
from rafcon.mvc.selection import Selection
from rafcon.mvc.models.state import StateModel
from rafcon.mvc.models.scoped_variable import ScopedVariableModel
from rafcon.statemachine.id_generator import state_id_generator
from rafcon.statemachine.states.container_state import ContainerState

ClipboardType = Enum('CLIPBOARD_TYPE', 'CUT COPY')

def get_port_m_core_element(port_m):
    if isinstance(port_m, ScopedVariableModel):
        return port_m.scoped_variable
    else:
        return port_m.data_port


class Clipboard(Observable):
    """A class to hold models and selection for later usage in cut/paste or copy/paste actions.
    In cut/paste action the selection stored is used while later paste. In a copy/paste actions
    """

    def __init__(self):
        Observable.__init__(self)
        self._selection = None

        self.selected_outcome_models = []
        self.outcome_model_copies = []

        self.selected_input_data_port_models = []
        self.input_data_port_model_copies = []

        self.selected_output_data_port_models = []
        self.output_data_port_model_copies = []

        self.selected_scoped_variable_models = []
        self.scoped_variable_model_copies = []

        self.selected_state_models = []
        self.state_model_copies = []

        self.selected_transition_models = []
        self.transition_model_copies = []

        self.selected_data_flow_models = []
        self.data_flow_model_copies = []

        self.copy_parent_state_id = None
        # TODO finish implementation of outcome id mapping
        self.outcome_id_mapping_dict = {}
        self.port_id_mapping_dict = {}
        # TODO check if it is secure that new state ids don't interfere with old state ids
        self.state_id_mapping_dict = {}

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
        self.outcome_model_copies = [deepcopy(model) for model in self.outcome_model_copies]
        self.input_data_port_model_copies = [deepcopy(model) for model in self.input_data_port_model_copies]
        self.output_data_port_model_copies = [deepcopy(model) for model in self.output_data_port_model_copies]
        self.scoped_variable_model_copies = [deepcopy(model) for model in self.scoped_variable_model_copies]
        self.state_model_copies = [deepcopy(model) for model in self.state_model_copies]
        self.transition_model_copies = [deepcopy(model) for model in self.transition_model_copies]
        self.data_flow_model_copies = [deepcopy(model) for model in self.data_flow_model_copies]

    def paste(self, target_state_m, cursor_position=None, limited=None, convert=False):
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
        assert isinstance(target_state_m, StateModel) # in future Execution states can be used, too

        # update meta data of clipboard elements to adapt for new parent state
        logger.info("PASTE -> meta data adaptation has to be implemented {0}".format(self.clipboard_type))

        oc_m_copy_list = self.outcome_model_copies
        ip_m_copy_list = self.input_data_port_model_copies
        op_m_copy_list = self.output_data_port_model_copies
        sv_m_copy_list = self.scoped_variable_model_copies
        state_m_copy_list = self.state_model_copies
        t_m_copy_list = self.transition_model_copies
        df_m_copy_list = self.data_flow_model_copies

        self.prepare_new_copy()  # threaded in future -> important that the copy is prepared here!!!

        self.state_id_mapping_dict[self.copy_parent_state_id] = target_state_m.state.state_id

        def insert_elements_from_model_copies_list(model_list, element_str):
            new_and_copy_models = []
            for orig_m_copy in model_list:
                try:
                    new_and_copy_models.append(getattr(self, 'insert_{0}'.format(element_str))(target_state_m, orig_m_copy))
                except (ValueError, AttributeError, TypeError) as e:
                    logger.warning("While inserting a {0} a failure was detected, exception: {1}.".format(element_str, e))

            return new_and_copy_models

        # prepare list of lists to copy for limited or converted paste of objects
        execution_state_unlimited = ['outcomes', 'input_data_ports', 'output_data_ports']
        container_state_unlimited = execution_state_unlimited + ['scoped_variables', 'states', 'transitions', 'data_flows']
        if isinstance(target_state_m.state, ContainerState):
            tolerated_lists = container_state_unlimited
        else:
            tolerated_lists = execution_state_unlimited
        if limited and all([list_name in tolerated_lists for list_name in limited]):
            if len(limited) == 1 and limited[0] in ['input_data_ports', 'output_data_ports', 'scoped_variables'] and convert:
                ip_m_copy_list = op_m_copy_list = sv_m_copy_list = ip_m_copy_list + op_m_copy_list + sv_m_copy_list
            lists_to_insert = limited
        else:
            lists_to_insert = tolerated_lists

        # check list order and put transitions and data flows to the end
        for list_name in ['transitions', 'data_flows']:
            if list_name in lists_to_insert:
                lists_to_insert.remove(list_name)
                lists_to_insert.append(list_name)

        # insert all lists and there elements into target state
        insert_dict = dict()
        lists = {'outcomes': oc_m_copy_list, 'input_data_ports': ip_m_copy_list, 'output_data_ports': op_m_copy_list,
                 'scoped_variables': sv_m_copy_list, 'states': state_m_copy_list, 'transitions': t_m_copy_list,
                 'data_flows': df_m_copy_list}
        for list_name in lists_to_insert:
            insert_dict[list_name] = insert_elements_from_model_copies_list(lists[list_name], list_name[:-1])

        if self.clipboard_type is ClipboardType.CUT:
            # delete original elements
            # TODO cut now can be realized directly after the copy has been generated -> check which one is appropriate
            self.do_cut_removal()

        self.reset_clipboard()

        return insert_dict

    def do_cut_removal(self):
        def remove_selected_elements_of_type(element_str, model_attr_str=None, id_attr_str=None):
            model_attr_str = element_str if model_attr_str is None else model_attr_str
            id_attr_str = element_str + '_id' if id_attr_str is None else id_attr_str
            for model in getattr(self, 'selected_{0}_models'.format(element_str)):
                # remove model from selection to avoid conflicts
                # -> selection is not observing state machine changes and state machine model is not updating it
                if model.parent is None and isinstance(model.state, StateModel) and model.state.is_root_state:
                    selection = model.get_sm_m_for_state_m().selection if model.get_sm_m_for_state_m() else None
                else:
                    selection = model.parent.get_sm_m_for_state_m().selection if model.parent.get_sm_m_for_state_m() else None
                if selection and model in getattr(selection, element_str + 's'):
                    selection.remove(model)
                # remove element
                element_id = getattr(getattr(model, model_attr_str), id_attr_str)
                parent_of_source_state = getattr(getattr(model, model_attr_str), 'parent')
                getattr(parent_of_source_state, 'remove_{0}'.format(element_str))(element_id)

        remove_selected_elements_of_type('data_flow')
        remove_selected_elements_of_type('input_data_port', 'data_port', 'data_port_id')
        remove_selected_elements_of_type('output_data_port', 'data_port', 'data_port_id')
        remove_selected_elements_of_type('scoped_variable', 'scoped_variable', 'data_port_id')
        remove_selected_elements_of_type('transition')
        remove_selected_elements_of_type('outcome')
        remove_selected_elements_of_type('state')

    def insert_state(self, target_state_m, orig_state_copy_m):
        target_state = target_state_m.state
        orig_state_copy = orig_state_copy_m.state

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
        self.state_id_mapping_dict[old_state_id] = new_state_id
        return new_state_copy_m, orig_state_copy_m

    def insert_transition(self, target_state_m, orig_transition_copy_m):
        t = orig_transition_copy_m.transition
        from_state = self.state_id_mapping_dict[t.from_state]
        to_state = self.state_id_mapping_dict[t.to_state]
        t_id = target_state_m.state.add_transition(from_state, t.from_outcome, to_state, t.to_outcome)
        target_state_m.get_transition_m(t_id).meta = orig_transition_copy_m.meta
        return target_state_m.get_transition_m(t_id), orig_transition_copy_m

    def insert_data_flow(self, target_state_m, orig_data_flow_copy_m):
        df = orig_data_flow_copy_m.data_flow
        from_state = self.state_id_mapping_dict[df.from_state]
        from_key = self.port_id_mapping_dict.get((df.from_state, df.from_key), df.from_key)
        to_state = self.state_id_mapping_dict[df.to_state]
        to_key = self.port_id_mapping_dict.get((df.to_state, df.to_key), df.to_key)
        df_id = target_state_m.state.add_data_flow(from_state, from_key, to_state, to_key)
        target_state_m.get_data_flow_m(df_id).meta = orig_data_flow_copy_m.meta
        return target_state_m.get_data_flow_m(df_id), orig_data_flow_copy_m

    def insert_outcome(self, target_state_m, orig_outcome_copy_m):
        oc = orig_outcome_copy_m.outcome
        oc_id = target_state_m.state.add_outcome(oc.name)
        target_state_m.get_outcome_m(oc_id).meta = orig_outcome_copy_m.meta
        return target_state_m.get_outcome_m(oc_id), orig_outcome_copy_m

    def insert_input_data_port(self, target_state_m, orig_input_data_port_copy_m):
        ip = get_port_m_core_element(orig_input_data_port_copy_m)
        old_port_tuple = (self.copy_parent_state_id, ip.data_port_id)
        data_port_id = target_state_m.state.add_input_data_port(ip.name, ip.data_type, ip.default_value)
        self.port_id_mapping_dict[old_port_tuple] = data_port_id
        target_state_m.get_data_port_m(data_port_id).meta = orig_input_data_port_copy_m.meta
        return target_state_m.get_data_port_m(data_port_id), orig_input_data_port_copy_m

    def insert_output_data_port(self, target_state_m, orig_output_data_port_copy_m):
        op = get_port_m_core_element(orig_output_data_port_copy_m)
        old_port_tuple = (self.copy_parent_state_id, op.data_port_id)
        data_port_id = target_state_m.state.add_output_data_port(op.name, op.data_type, op.default_value)
        self.port_id_mapping_dict[old_port_tuple] = data_port_id
        target_state_m.get_data_port_m(data_port_id).meta = orig_output_data_port_copy_m.meta
        return target_state_m.get_data_port_m(data_port_id), orig_output_data_port_copy_m

    def insert_scoped_variable(self, target_state_m, orig_scoped_variable_copy_m):
        sv = get_port_m_core_element(orig_scoped_variable_copy_m)
        old_port_tuple = (self.copy_parent_state_id, sv.data_port_id)
        data_port_id = target_state_m.state.add_scoped_variable(sv.name, sv.data_type, sv.default_value)
        self.port_id_mapping_dict[old_port_tuple] = data_port_id
        target_state_m.get_data_port_m(data_port_id).meta = orig_scoped_variable_copy_m.meta
        return target_state_m.get_data_port_m(data_port_id), orig_scoped_variable_copy_m

    def reset_clipboard(self):
        """ Resets the clipboard, so that old elements do not pollute the new selection that is copied into the clipboard.
        :return:
        """
        # reset outcomes
        self.selected_outcome_models = []
        self.outcome_model_copies = []

        # reset input data ports
        self.selected_input_data_port_models = []
        self.input_data_port_model_copies = []

        # reset output data ports
        self.selected_output_data_port_models = []
        self.output_data_port_model_copies = []

        # reset scoped variables
        self.selected_scoped_variable_models = []
        self.scoped_variable_model_copies = []

        # reset states
        self.selected_state_models = []
        self.state_model_copies = []

        # reset transitions
        self.selected_transition_models = []
        self.transition_model_copies = []

        # reset data flows
        self.selected_data_flow_models = []
        self.data_flow_model_copies = []

        # reset mapping dictionaries
        self.outcome_id_mapping_dict = {}
        self.port_id_mapping_dict = {}
        self.state_id_mapping_dict = {}

    def __create_core_object_copies(self, selection):
        """Copy all elements of a selection.

         The method copies all objects and checks and ignores directly data flows and transitions which are selected
         without related origin or targets. Additional the method copies elements linkage (data flows and transitions)
         if those origins and targets are covered by the selected elements. Therefore the selection it self is manipulated
         to provide direct feedback to the user.

        :param selection: an arbitrary selection, whose elements should be copied
        :return:
        """
        self.copy_parent_state_id = None

        def get_ports_related_to_data_flow(data_flow):
            from_port = data_flow.parent.get_data_port(data_flow.from_state, data_flow.from_key)
            to_port = data_flow.parent.get_data_port(data_flow.to_state, data_flow.to_key)
            return from_port, to_port

        def get_states_related_to_transition(transition):
            if transition.from_state == transition.parent.state_id or transition.from_state is None:
                from_state = transition.parent
            else:
                from_state = transition.parent.states[transition.from_state]
            if transition.to_state == transition.parent.state_id:
                to_state = transition.parent
            else:
                to_state = transition.parent.states[transition.to_state]
            return from_state, to_state

        all_models_selected = selection.get_all()
        if not all_models_selected:
            # print "no selection"
            return
        # check if all elements that are copied are on one hierarchy level -> TODO or in future are parts of sibling
        # logger.info("COPY/CUT -> hierarchy level for copy has to be implemented")
        parent_m_count_dict = {}
        for model in selection.get_all():
            parent_m_count_dict[model.parent] = parent_m_count_dict[model.parent] + 1 if model.parent in parent_m_count_dict else 1
        parent_m = None
        current_count_parent = 0
        for possible_parent_m, count in parent_m_count_dict.iteritems():
            parent_m = possible_parent_m if current_count_parent < count else parent_m
        # if root no parent exist and only on model can be selected
        if len(selection.states) == 1 and selection.states[0].state.is_root_state:
            parent_m = None
            # kick all selection except root_state
            if len(all_models_selected) > 1:
                selection.set(selection.states[0])
        if parent_m is not None:
            # check and reduce selection
            for model in all_models_selected:
                if model.parent is not parent_m:
                    selection.remove(model)
            self.copy_parent_state_id = parent_m.state.state_id
        # reduce linkage selection by not fully by selection covered linkage
        # logger.info("COPY/CUT -> reduce linkage has to be implemented")
        possible_states = [state_m.state for state_m in selection.states]
        for data_flow_m in selection.data_flows:
            from_port, to_port = get_ports_related_to_data_flow(data_flow_m.data_flow)
            if from_port.parent not in possible_states or to_port not in possible_states:
                selection.remove(data_flow_m)
        for transition_m in selection.transitions:
            from_state, to_state = get_states_related_to_transition(transition_m.transition)
            if from_state not in possible_states or to_state not in possible_states:
                selection.remove(transition_m)
        # extend linkage selection by fully by selection covered linkage
        # logger.info("COPY/CUT -> extend linkage has to be implemented")
        if parent_m and isinstance(parent_m.state, ContainerState):
            state_ids = [state.state_id for state in possible_states]
            port_ids = [sv_m.scoped_variable.data_port_id for sv_m in selection.scoped_variables] + \
                [ip_m.data_port.data_port_id for ip_m in selection.input_data_ports] + \
                [op_m.data_port.data_port_id for op_m in selection.output_data_ports]
            ports = [sv_m.scoped_variable for sv_m in selection.scoped_variables] + \
                [ip_m.data_port for ip_m in selection.input_data_ports] + \
                [op_m.data_port for op_m in selection.output_data_ports]
            related_transitions, related_data_flows = \
                parent_m.state.related_linkage_states_and_scoped_variables(state_ids, port_ids)
            # extend by selected states or a port and a state enclosed data flows
            for data_flow in related_data_flows['enclosed']:
                data_flow_m = parent_m.get_data_flow_m(data_flow.data_flow_id)
                if data_flow_m not in selection.data_flows:
                    selection.add(data_flow_m)
            # extend by selected ports enclosed data flows
            for data_flow_id, data_flow in parent_m.state.data_flows.iteritems():
                from_port, to_port = get_ports_related_to_data_flow(data_flow)
                if from_port in ports and to_port in ports:
                    selection.add(parent_m.get_data_flow_m(data_flow_id))
            # extend by selected states enclosed transitions
            for transition in related_transitions['enclosed']:
                transition_m = parent_m.get_transition_m(transition.transition_id)
                if transition_m not in selection.transitions:
                    selection.add(transition_m)
            # TODO extend by selected state and parent outcome enclosed transitions

        self.selected_state_models = selection.states
        self.selected_outcome_models = selection.outcomes
        self.selected_input_data_port_models = selection.input_data_ports
        self.selected_output_data_port_models = selection.output_data_ports
        self.selected_transition_models = selection.transitions
        self.selected_data_flow_models = selection.data_flows
        self.selected_scoped_variable_models = selection.scoped_variables

        def copy_list_of_elements_of_type(element_str):
            # print element_str, ": ", getattr(self, 'selected_{0}_models'.format(element_str))
            # create state copies
            for model in getattr(self, 'selected_{0}_models'.format(element_str)):
                # direct model copy -> copy meta data and core object
                model_copy = deepcopy(model)
                getattr(self, '{0}_model_copies'.format(element_str)).append(model_copy)

        copy_list_of_elements_of_type('outcome')
        copy_list_of_elements_of_type('input_data_port')
        copy_list_of_elements_of_type('output_data_port')
        copy_list_of_elements_of_type('state')
        copy_list_of_elements_of_type('scoped_variable')
        copy_list_of_elements_of_type('transition')
        copy_list_of_elements_of_type('data_flow')


# To enable copy, cut and paste between state machines a global clipboard is used
global_clipboard = Clipboard()
