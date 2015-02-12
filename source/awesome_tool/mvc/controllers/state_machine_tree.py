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

        Controller.__init__(self, model, view, spurious=True)
        #self.relieve_model(model)
        self.observe_model(model.root_state)
        self.tree_store = gtk.TreeStore(str, str, str, gobject.TYPE_PYOBJECT)
        view.set_model(self.tree_store)
        self.path_store = {}

    def register_view(self, view):
        self.view.connect('cursor-changed', self.on_cursor_changed)
        self.update()

    def register_adapters(self):
        pass

    # TODO the update
    def update(self, changed_state_model=None):
        # if changed_state_model:
        #     print "PATH of container_changed: ", changed_state_model.state.get_path(), changed_state_model.state.state_id
        #     #find_state
        #     # find the changed state and modify tree
        #     print "search: %s" % changed_state_model.state.name
        #     for row in self.tree_store:
        #         if row[3] is changed_state_model:
        #             print "found changed container_state %s" % row
        #             for child in row:
        #                 print child
        #             # for state_model in changed_state_model.states:
        #             #     # check if missing
        #             #     # check if in
        # #else:
        # self.path_store.clear()
        self.tree_store.clear()
        parent = self.tree_store.insert_before(None, None,
                                               (self.model.root_state.state.name,
                                                self.model.root_state.state.state_id,
                                                self.model.root_state.state.state_type,
                                                self.model.root_state))
        # self.path_store[self.model.root_state.state.get_path()] = parent
        for state_id, smodel in self.model.root_state.states.items():
            self.insert_rec(parent, smodel)
        print "PATH_STORE: ", self.path_store

    def insert_rec(self, parent, state_model):
        #print 'Inserting %s' % str((state.title, state.id, utils.const2str(Point.Point, state.type)))
        parent = self.tree_store.insert_before(parent, None,
                                               (state_model.state.name,
                                                state_model.state.state_id,
                                                state_model.state.state_type,
                                                state_model))
        # self.path_store[state_model.state.get_path()] = parent
        if type(state_model) is ContainerStateModel:
            #print "Insert container state %s recursively" % state_model.state.name
            for state_id, smodel in state_model.states.items():
                self.insert_rec(parent, smodel)

    def on_cursor_changed(self, widget):
        (model, row) = self.view.get_selection().get_selected()
        #print "SM_Tree state selected: %s, %s" % (model, row)
        logger.debug("The view should jump to the selected state and the zoom should be adjusted as well")
        if row is not None:
            state_model = model[row][3]
            self.model.selection.clear()

            self.model.selection.add(state_model)

    @Controller.observe("states", after=True)
    def assign_notification_state(self, model, prop_name, info):
        logger.debug("SM Tree State %s call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %
                    (self.model.root_state.state.state_id, prop_name, info.instance, info.method_name, info))
        print "info: ", info
        if hasattr(info.kwargs, 'model'):
            self.update(info.kwargs.model)
        else:
            self.update()
