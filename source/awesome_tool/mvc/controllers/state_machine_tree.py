import gtk
import gobject
from gtkmvc import Controller
from gtkmvc import Observer

from mvc.models import ContainerStateModel
from utils import log
logger = log.get_logger(__name__)

#TODO: comment

class StateMachineTreeController(Controller):

    def __init__(self, model, view):
        """Constructor
        :param StateMachineModel model should be exchangeable
        """
        Controller.__init__(self, model, view)
        # self.model = model
        self.tree_store = gtk.TreeStore(str, str, str, gobject.TYPE_PYOBJECT)
        view.set_model(self.tree_store)

    def register_view(self, view):
        self.view.connect('cursor-changed', self.on_cursor_changed)
        self.update()

    def register_adapters(self):
        pass

    def update(self):
        self.tree_store.clear()
        parent = self.tree_store.insert_before(None, None,
                                               (self.model.root_state.state.name,
                                                self.model.root_state.state.state_id,
                                                self.model.root_state.state.state_type,
                                                self.model.root_state))
        for state_id, smodel in self.model.root_state.states.items():
            self.insert_rec(parent, smodel)

    def insert_rec(self, parent, state_model):
        parent = self.tree_store.insert_before(parent, None,
                                               (state_model.state.name,
                                                state_model.state.state_id,
                                                state_model.state.state_type,
                                                state_model))
        if type(state_model) is ContainerStateModel:
            for state_id, smodel in state_model.states.items():
                self.insert_rec(parent, smodel)

    def on_cursor_changed(self, widget):
        (model, row) = self.view.get_selection().get_selected()
        logger.debug("The view should jump to the selected state and the zoom should be adjusted as well")
        if row is not None:
            state_model = model[row][3]
            self.model.selection.clear()

            self.model.selection.add(state_model)

    # TODO check if delete works for the state_machine_tree, too
    @Observer.observe("root_state.state", after=True)
    def assign_notification_state(self, model, prop_name, info):
        self.update()
