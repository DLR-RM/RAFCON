import gtk
import gobject

from awesome_tool.mvc.controllers.extended_controller import ExtendedController
import awesome_tool.statemachine.singleton
from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.states.container_state import ContainerState
from awesome_tool.statemachine.states.library_state import LibraryState


class LibraryTreeController(ExtendedController):

    """

    """
    def __init__(self, model=None, view=None, state_machine_manager_model=None):
        ExtendedController.__init__(self, model, view)
        self.library_tree_store = gtk.TreeStore(str, gobject.TYPE_PYOBJECT, str)
        view.set_model(self.library_tree_store)

        self.state_machine_manager_model = state_machine_manager_model

        self.update()

    def register_adapters(self):
        pass

    def register_view(self, view):
        self.view.connect('cursor-changed', self.on_cursor_changed)

        self.view.connect('button_press_event', self.right_click)

    def right_click(self, widget, event=None):
        if event.type == gtk.gdk.BUTTON_PRESS and event.button == 3:
            menu = gtk.Menu()
            add_link_menu_item = gtk.ImageMenuItem(gtk.STOCK_ADD)
            add_link_menu_item.set_label("Add as library (link)")
            add_link_menu_item.connect("activate", self.add_link_button_clicked)

            add_template_menu_item = gtk.ImageMenuItem(gtk.STOCK_COPY)
            add_template_menu_item.set_label("Add as template (copy)")
            add_template_menu_item.connect("activate", self.add_template_button_clicked)

            open_menu_item = gtk.ImageMenuItem(gtk.STOCK_OPEN)
            open_menu_item.set_label("Open")
            open_menu_item.connect("activate", self.open_button_clicked)

            open_run_menu_item = gtk.MenuItem("Open and run")
            open_run_menu_item.connect("activate", self.open_run_button_clicked)

            menu.append(add_link_menu_item)
            menu.append(add_template_menu_item)
            menu.append(gtk.SeparatorMenuItem())
            menu.append(open_menu_item)
            menu.append(open_run_menu_item)

            menu.show_all()
            x = int(event.x)
            y = int(event.y)
            time = event.time
            pthinfo = self.view.get_path_at_pos(x, y)
            if pthinfo is not None:
                path, col, cellx, celly = pthinfo
                self.view.grab_focus()
                self.view.set_cursor(path, col, 0)

                (model, row) = self.view.get_selection().get_selected()
                if isinstance(model[row][1], dict):  # right click on folder, not library
                    return False

                menu.popup(None, None, None, event.button, time)
            return True

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
        # The user clicked on an entry in the tree store
        return

    def add_link_button_clicked(self, widget):
        smm_m = self.state_machine_manager_model
        (model, row) = self.view.get_selection().get_selected()
        library_key = model[row][0]
        library = model[row][1]
        library_path = model[row][2]

        current_selection = smm_m.state_machines[smm_m.selected_state_machine_id].selection
        if len(current_selection.get_states()) > 1 or len(current_selection.get_states()) == 0:
            logger.error("Please select exactly one state for the insertion of a library")
            return

        current_state = current_selection.get_states()[0].state
        if not isinstance(current_state, ContainerState):
            logger.error("Libraries can only be inserted in container states")
            return

        logger.debug("Link library state %s (with file path %s, and library path %s) into the state machine" %
                     (str(library_key), str(library), str(library_path)))

        library_state = LibraryState(library_path, library_key, "0.1", library_key)

        current_state.add_state(library_state)

    def add_template_button_clicked(self, widget):
        smm_m = self.state_machine_manager_model
        (model, row) = self.view.get_selection().get_selected()
        library_key = model[row][0]
        library = model[row][1]
        library_path = model[row][2]

        current_selection = smm_m.state_machines[smm_m.selected_state_machine_id].selection
        if len(current_selection.get_states()) > 1 or len(current_selection.get_states()) == 0:
            logger.error("Please select exactly one state for the insertion of a library template")
            return

        current_state = current_selection.get_states()[0].state
        if not isinstance(current_state, ContainerState):
            logger.error("Templates can only be inserted in container states")
            return

        logger.debug("Insert library template state %s (with file path %s, and library path %s) into the state machine" %
                     (str(library_key), str(library), str(library_path)))

        library_name = library_key

        path_list = library_path.split("/")
        target_lib_dict = awesome_tool.statemachine.singleton.library_manager.libraries
        for element in path_list:
            target_lib_dict = target_lib_dict[element]
        logger.debug("Load state to which this library state links")
        state_machine, lib_version, creation_time = awesome_tool.statemachine.singleton.library_manager.storage.\
            load_statemachine_from_yaml(target_lib_dict[library_name])

        state_machine.root_state.change_state_id()

        current_state.add_state(state_machine.root_state)
        current_state_model = current_selection.get_states()[0]
        current_state_model.states[state_machine.root_state.state_id].load_meta_data_for_state()
        current_state_model.states[state_machine.root_state.state_id].temp['gui']['editor']['recalc'] = True

    def open_button_clicked(self, widget):
        self.open_library_as_state_machine()

    def open_run_button_clicked(self, widget):
        state_machine = self.open_library_as_state_machine()
        awesome_tool.statemachine.singleton.state_machine_execution_engine.start(state_machine.state_machine_id)

    def open_library_as_state_machine(self):
        from awesome_tool.statemachine.singleton import global_storage
        smm_m = self.state_machine_manager_model
        (model, row) = self.view.get_selection().get_selected()
        library_path = model[row][1]

        logger.debug("Opening library as state-machine from path '{0}'".format(library_path))
        state_machine, version, creation_time = global_storage.load_statemachine_from_yaml(library_path)

        smm_m.state_machine_manager.add_state_machine(state_machine)
        return state_machine



