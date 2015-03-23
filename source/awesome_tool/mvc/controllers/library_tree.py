import gtk
import gobject

from awesome_tool.mvc.controllers.extended_controller import ExtendedController
import awesome_tool.statemachine.singleton
from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.enums import StateType
from awesome_tool.statemachine.states.library_state import LibraryState


class LibraryTreeController(ExtendedController):  # (Controller):

    """

    """
    def __init__(self, model=None, view=None):
        ExtendedController.__init__(self, model, view)
        self.library_tree_store = gtk.TreeStore(str, gobject.TYPE_PYOBJECT, str)
        view.set_model(self.library_tree_store)

        self.update()

    def register_adapters(self):
        pass

    def register_view(self, view):
        self.view.connect('cursor-changed', self.on_cursor_changed)

    @ExtendedController.observe("library_manager", after=True)
    def model_changed(self, model, prop_name, info):
        self.update()

    def update(self):
        #logger.debug("Update of library_tree controller called")
        self.library_tree_store.clear()
        for library_key, library_item in awesome_tool.statemachine.singleton.library_manager.libraries.iteritems():
            self.insert_rec(None, library_key, library_item, "")

    def insert_rec(self, parent, library_key, library_item, library_path):
        #logger.debug("Add new library to tree store: %s" % library_key)
        tree_item = self.library_tree_store.insert_before(parent, None, (library_key, library_item, library_path))
        if library_path == "":
            library_path = library_key
        else:
            library_path = library_path + "/" + library_key
        if isinstance(library_item, dict):
            #logger.debug("Found library container: %s" % library_key)
            for child_key, child_item in library_item.iteritems():
                self.insert_rec(tree_item, child_key, child_item, library_path)

    def on_cursor_changed(self, widget):
        (model, row) = self.view.get_selection().get_selected()
        library_key = model[row][0]
        library = model[row][1]
        #logger.debug("The library state should be inserted into the statemachine")

    def add_link_button_clicked(self, widget, smm):
        (model, row) = self.view.get_selection().get_selected()
        library_key = model[row][0]
        library = model[row][1]
        library_path = model[row][2]

        current_selection = smm.state_machines[smm.selected_state_machine_id].selection
        if len(current_selection.get_states()) > 1 or len(current_selection.get_states()) == 0:
            logger.error("Wrong number of selected states %s" % str(len(current_selection.get_states())))
            return

        current_state = current_selection.get_states()[0].state
        state_type = current_state.state_type
        if state_type is not (StateType.HIERARCHY or StateType.BARRIER_CONCURRENCY or StateType.PREEMPTION_CONCURRENCY):
            logger.error("State of wrong type selected %s" % str(state_type))
            return

        logger.debug("Link library state %s (with file path %s, and library path %s) into the state machine" %
                     (str(library_key), str(library), str(library_path)))

        library_state = LibraryState(library_path, library_key, "0.1", "new_library_state")

        current_state.add_state(library_state)

    def add_template_button_clicked(self, widget, smm):
        (model, row) = self.view.get_selection().get_selected()
        library_key = model[row][0]
        library = model[row][1]
        library_path = model[row][2]

        current_selection = smm.state_machines[smm.selected_state_machine_id].selection
        if len(current_selection.get_states()) > 1 or len(current_selection.get_states()) == 0:
            logger.error("Wrong number of selected states %s" % str(len(current_selection.get_states())))
            return

        current_state = current_selection.get_states()[0].state
        state_type = current_state.state_type
        if state_type is not (StateType.HIERARCHY or StateType.BARRIER_CONCURRENCY or StateType.PREEMPTION_CONCURRENCY):
            logger.error("State of wrong type selected %s" % str(state_type))
            return

        logger.debug("Insert library template state %s (with file path %s, and library path %s) into the state machine" %
                     (str(library_key), str(library), str(library_path)))

        library_name = library_key

        path_list = library_path.split("/")
        target_lib_dict = awesome_tool.statemachine.singleton.library_manager.libraries
        for element in path_list:
            target_lib_dict = target_lib_dict[element]
        logger.debug("Load state to which this library state links")
        state_machine, lib_version, creationtime = awesome_tool.statemachine.singleton.library_manager.storage.\
            load_statemachine_from_yaml(target_lib_dict[library_name])

        state_machine.root_state.change_state_id()

        current_state.add_state(state_machine.root_state)




