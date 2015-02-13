import gtk
import gobject
from gtkmvc import Controller

from mvc.models import ContainerStateModel
from mvc.models.state_machine import StateMachineModel
from utils import log
logger = log.get_logger(__name__)

#TODO: comment


class StateMachineTreeController(Controller):

    def __init__(self, model, view):
        """Constructor
        :param model StateMachineModel should be exchangeable
        """
        assert isinstance(model, StateMachineModel)

        Controller.__init__(self, model, view)
        # self.relieve_model(model)
        # self.observe_model(model.root_state)
        # model.root_state.register_observer(self)
        self.tree_store = gtk.TreeStore(str, str, str, gobject.TYPE_PYOBJECT)
        view.set_model(self.tree_store)
        self.path_store = {}

    def register_view(self, view):
        self.view.connect('cursor-changed', self.on_cursor_changed)
        self.update()

    def register_adapters(self):
        pass

    def update(self, changed_state_model=None):
        """
        Function checks if all states are in tree and if tree has states which were deleted
        :param changed_state_model:
        :return:
        """
        gtk.gdk.threads_enter()

        if changed_state_model:
            if changed_state_model.parent is None:
                parent_iter = self.path_store[changed_state_model.state.get_path()]
            else:
                parent_iter = self.path_store[changed_state_model.parent.state.get_path()]
        else:
            changed_state_model = self.model.root_state
            parent_iter = None
            self.path_store.clear()
            self.tree_store.clear()

        if not changed_state_model.state.get_path() in self.path_store:
            own_root_state__model = ContainerStateModel(self.model.root_state.state)
            parent_iter = self.tree_store.insert_before(parent_iter, None,
                                                   (self.model.root_state.state.name,
                                                    self.model.root_state.state.state_id,
                                                    self.model.root_state.state.state_type,
                                                    self.model.root_state))
            #self.tree_store.row_inserted(path=self.tree_store.get_path(parent_iter), iter=parent_iter)
            self.path_store[self.model.root_state.state.get_path()] = parent_iter
        else:
            parent_iter = self.path_store[self.model.root_state.state.get_path()]

        # check if child are all in
        for state_id, smodel in self.model.root_state.states.items():
            self.insert_rec(parent_iter, smodel)

        # check if child should not be in
        for n in reversed(range(self.tree_store.iter_n_children(parent_iter))):
            child_iter = self.tree_store.iter_nth_child(parent_iter, n)
            path = self.tree_store.get_path(child_iter)
            model = self.view.get_model()
            if not model[path][1] in self.model.root_state.states:
                self.tree_store.remove(child_iter)
                #self.tree_store.row_deleted(self.tree_store.get_path(child_iter))

        gtk.gdk.threads_leave()

    def insert_rec(self, parent_iter, state_model):
        # check if in
        if not state_model.state.get_path() in self.path_store:
            parent_iter = self.tree_store.insert_before(parent_iter, None,
                                                   (state_model.state.name,
                                                    state_model.state.state_id,
                                                    state_model.state.state_type,
                                                    state_model))
            #self.tree_store.row_inserted(path=self.tree_store.get_path(parent_iter), iter=parent_iter)
            self.path_store[state_model.state.get_path()] = parent_iter
        else:
            parent_iter = self.path_store[state_model.state.get_path()]
            path = self.tree_store.get_path(parent_iter)
            model = self.view.get_model()
            if not state_model.state.state_type == model[path][2] or not state_model.state.name == model[path][0]:
                model[path][0] = state_model.state.name
                model[path][2] = state_model.state.state_type

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
        logger.debug("The view should jump to the selected state and the zoom should be adjusted as well")
        if row is not None:
            state_model = model[row][3]
            self.model.selection.clear()

            self.model.selection.add(state_model)


    @Controller.observe("selection", after=True)
    # # @Controller.observe("states", after=True)
    # @Controller.observe("state_machine", after=True)
    def assign_notification_state(self, model, prop_name, info):
        # logger.debug("SM Tree State %s call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %
        #             (self.model.root_state.state.state_id, prop_name, info.instance, info.method_name, info))
        # print "info: ", info
        # if hasattr(info.kwargs, 'model'):
        #     self.update(info.kwargs.model)
        # else:
        self.update(self.model.root_state)

    # @Controller.observe("selection", after=True)
    # def assign_notification(self, model, prop_name, info):
    #     self.update(self.model.root_state)
