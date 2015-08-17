import gtk
import gobject

from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.models import ContainerStateModel
from rafcon.mvc.models.state_machine_manager import StateMachineManagerModel
from rafcon.mvc.models.state_machine import StateMachineModel
from rafcon.statemachine.states.state import State
from rafcon.utils import log
logger = log.get_logger(__name__)

#TODO: comment


class StateMachineTreeController(ExtendedController):

    def __init__(self, model, view):
        """Constructor
        :param model StateMachineModel should be exchangeable
        """
        assert isinstance(model, StateMachineManagerModel)

        ExtendedController.__init__(self, model, view)
        # self.relieve_model(model)
        # self.observe_model(model.root_state)
        # model.root_state.register_observer(self)
        self.view_is_registered = False
        self.tree_store = gtk.TreeStore(str, str, str, gobject.TYPE_PYOBJECT)
        view.set_model(self.tree_store)
        #view.set_hover_expand(True)
        self.path_store = {}
        self.__my_selected_sm_id = None
        self._selected_sm_model = None

        self.__buffered_root_state = None  # needed to handle exchange of root_state

        self.register()

    @ExtendedController.observe("selected_state_machine_id", assign=True)
    def state_machine_manager_notification(self, model, property, info):
        self.register()

    def register(self):
        """
        Change the state machine that is observed for new selected states to the selected state machine.
        :return:
        """
        # print "state_machine_tree register state_machine"
        # relieve old models
        if self.__my_selected_sm_id is not None:  # no old models available
            self.relieve_model(self.__buffered_root_state)
            self.relieve_model(self._selected_sm_model)
        # set own selected state machine id
        self.__my_selected_sm_id = self.model.selected_state_machine_id
        if self.__my_selected_sm_id is not None:
            # observe new models
            self._selected_sm_model = self.model.state_machines[self.__my_selected_sm_id]
            # logger.debug("NEW SM SELECTION %s" % self._selected_sm_model)
            self.__buffered_root_state = self._selected_sm_model.root_state
            self.observe_model(self._selected_sm_model.root_state)
            self.observe_model(self._selected_sm_model)  # for selection
            self.update()
        else:
            self.tree_store.clear()

    def notification_assign_new_root_state(self):
        pass

    @ExtendedController.observe("states", after=True)
    def states_update(self, model, property, info):
        self.update()

    def register_view(self, view):
        self.view.connect('cursor-changed', self.on_cursor_changed)
        self.view_is_registered = True
        self.update()

    def register_adapters(self):
        pass

    def update(self, changed_state_model=None):
        """
        Function checks if all states are in tree and if tree has states which were deleted
        :param changed_state_model:
        :return:
        """
        if not self.view_is_registered:
            return

        if changed_state_model:
            if not isinstance(changed_state_model.state.parent, State):
                parent_iter = self.path_store[changed_state_model.state.get_path()]
            else:
                parent_iter = self.path_store[changed_state_model.parent.state.get_path()]
        else:
            parent_iter = None
            self.path_store.clear()
            self.tree_store.clear()
            if self._selected_sm_model:
                changed_state_model = self._selected_sm_model.root_state
            else:
                return

        if not changed_state_model.state.get_path() in self.path_store:
            parent_iter = self.tree_store.insert_before(parent_iter, None,
                                                        (self._selected_sm_model.root_state.state.name,
                                                         self._selected_sm_model.root_state.state.state_id,
                                                         type(self._selected_sm_model.root_state.state),
                                                         self._selected_sm_model.root_state))
            #self.tree_store.row_inserted(path=self.tree_store.get_path(parent_iter), iter=parent_iter)
            self.path_store[self._selected_sm_model.root_state.state.get_path()] = parent_iter
        else:
            parent_iter = self.path_store[self._selected_sm_model.root_state.state.get_path()]

        # check if child are all in
        if isinstance(self._selected_sm_model.root_state, ContainerStateModel):
            for state_id, smodel in self._selected_sm_model.root_state.states.items():
                self.insert_rec(parent_iter, smodel)

        # check if child should not be in
        for n in reversed(range(self.tree_store.iter_n_children(parent_iter))):
            child_iter = self.tree_store.iter_nth_child(parent_iter, n)
            path = self.tree_store.get_path(child_iter)
            model = self.view.get_model()
            if isinstance(self._selected_sm_model.root_state, ContainerStateModel):
                if not model[path][1] in self._selected_sm_model.root_state.states:
                    self.tree_store.remove(child_iter)
                    #self.tree_store.row_deleted(self.tree_store.get_path(child_iter))

    def insert_rec(self, parent_iter, state_model):
        # check if in
        if not state_model.state.get_path() in self.path_store:
            parent_iter = self.tree_store.insert_before(parent_iter, None,
                                                        (state_model.state.name,
                                                         state_model.state.state_id,
                                                         type(state_model.state),
                                                         state_model))
            #self.tree_store.row_inserted(path=self.tree_store.get_path(parent_iter), iter=parent_iter)
            self.path_store[state_model.state.get_path()] = parent_iter
        else:
            parent_iter = self.path_store[state_model.state.get_path()]
            path = self.tree_store.get_path(parent_iter)
            model = self.view.get_model()
            if not type(state_model.state) == model[path][2] or not state_model.state.name == model[path][0]:
                model[path][0] = state_model.state.name
                model[path][2] = type(state_model.state)
                model[path][3] = state_model

        # check if child are all in
        if type(state_model) is ContainerStateModel:
            for state_id, smodel in state_model.states.items():
                self.insert_rec(parent_iter, smodel)

        # check if child should not be in
        for n in reversed(range(self.tree_store.iter_n_children(parent_iter))):
            child_iter = self.tree_store.iter_nth_child(parent_iter, n)
            path = self.tree_store.get_path(child_iter)
            model = self.view.get_model()
            # check if there are left over rows of old states (switch from HS or CS to S and so on)
            if not type(state_model) is ContainerStateModel or not model[path][1] in state_model.states:
                self.tree_store.remove(child_iter)
                #self.tree_store.row_deleted(self.tree_store.get_path(child_iter))

    def on_cursor_changed(self, widget):
        (model, row) = self.view.get_selection().get_selected()
        logger.debug("The view jumps to the selected state and the zoom should be adjusted as well")
        if row is not None:
            state_model = model[row][3]
            self._selected_sm_model.selection.clear()

            self._selected_sm_model.selection.add(state_model)

    # TODO should observe the states changes, too, for changed names or types
    @ExtendedController.observe("selection", after=True)
    def assign_notification_selection(self, model, prop_name, info):
        if self._selected_sm_model.selection.get_selected_state():

            # work around to avoid already selected but not insert state rows
            if not self._selected_sm_model.selection.get_selected_state().state.get_path() in self.path_store:
                self.update(self._selected_sm_model.root_state)

            (model, actual_iter) = self.view.get_selection().get_selected()
            selected_iter = self.path_store[self._selected_sm_model.selection.get_selected_state().state.get_path()]
            # logger.debug("TreeSelectionPaths actual %s and in state_machine.selection %s " % (actual_iter, selected_iter))
            selected_path = self.tree_store.get_path(selected_iter)
            if actual_iter is None:
                actual_path = None
            else:
                actual_path = self.tree_store.get_path(actual_iter)
            # logger.debug("TreeSelectionPaths actual %s and in state_machine.selection %s " % (actual_path, selected_path))
            if not selected_path == actual_path:
                # logger.debug("reselect state machine tree-selection")
                # if single selection-mode is set no unselect is needed
                #self.view.get_selection().unselect_path(actual_path)
                self.view.expand_to_path(selected_path)
                self.view.get_selection().select_iter(selected_iter)
                # work around to force selection to state-editor
                self._selected_sm_model.selection.set([self._selected_sm_model.selection.get_selected_state()])

        # if hasattr(info.kwargs, 'model'):
        #     self.update(info.kwargs.model)
        # else:
        # TODO make it work without the next line
        self.update(self._selected_sm_model.root_state)

    # # TEST ARENA
    # @Controller.observe("root_state", after=True)
    # def assign_notification_root_state(self, model, prop_name, info):
    #     print "ROOT_STATE_CHANGED"
    #
    # @Controller.observe("states", after=True)
    # def assign_notification_states(self, model, prop_name, info):
    #     print "STATES_CHANGED"
    #
    # @Controller.observe("selection", after=True)
    # def assign_notification_selections(self, model, prop_name, info):
    #     print "STATES_SELECTIONS"
    # # @Controller.observe("selection", after=True)
    # # def assign_notification(self, model, prop_name, info):
    # #     self.update(self.model.root_state)
