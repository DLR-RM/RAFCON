import gtk
import gobject
from functools import partial

from rafcon.mvc.controllers.extended_controller import ExtendedController

from rafcon.statemachine.states.container_state import ContainerState
from rafcon.statemachine.states.library_state import LibraryState
from rafcon.statemachine.singleton import library_manager

from rafcon.utils import log
logger = log.get_logger(__name__)


class LibraryTreeController(ExtendedController):

    def __init__(self, model=None, view=None, state_machine_manager_model=None):
        ExtendedController.__init__(self, model, view)
        self.library_tree_store = gtk.TreeStore(str, gobject.TYPE_PYOBJECT, str)
        view.set_model(self.library_tree_store)

        self.state_machine_manager_model = state_machine_manager_model

        self.library_row_iter_dict_by_library_path = {}
        self.__expansion_state = None

        self.update()

    def register_adapters(self):
        pass

    def register_view(self, view):
        self.view.connect('cursor-changed', self.on_cursor_changed)

        self.view.connect('button_press_event', self.mouse_click)

    def mouse_click(self, widget, event=None):
        # logger.info("press id: {0}, type: {1} goal: {2} {3} {4}".format(event.button, gtk.gdk.BUTTON_PRESS, event.type == gtk.gdk._2BUTTON_PRESS, event.type == gtk.gdk.BUTTON_PRESS, event.button == 1))
        if event.type == gtk.gdk._2BUTTON_PRESS and event.button == 1:
            (model, row) = self.view.get_selection().get_selected()
            if isinstance(model[row][1], dict):  # double click on folder, not library
                return False
            logger.info("left double click event detected -> open library: {0}/{1}".format(model[row][2], model[row][0]))
            self.open_button_clicked(None)
            return True
        if event.type == gtk.gdk.BUTTON_PRESS and event.button == 3:
            menu = gtk.Menu()
            add_link_menu_item = gtk.ImageMenuItem(gtk.STOCK_ADD)
            add_link_menu_item.set_label("Add as library (link)")
            add_link_menu_item.connect("activate", partial(self.insert_button_clicked, as_template=False))

            add_template_menu_item = gtk.ImageMenuItem(gtk.STOCK_COPY)
            add_template_menu_item.set_label("Add as template (copy)")
            add_template_menu_item.connect("activate", partial(self.insert_button_clicked, as_template=True))

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

    def store_expansion_state(self):
        # print "\n\n store of state machine {0} \n\n".format(self.__my_selected_sm_id)
        try:
            act_expansion_library = {}
            for library_path, library_row_iter in self.library_row_iter_dict_by_library_path.iteritems():
                library_row_path = self.library_tree_store.get_path(library_row_iter)
                act_expansion_library[library_path] = self.view.row_expanded(library_row_path)
                # if act_expansion_library[library_path]:
                #     print library_path
            self.__expansion_state = act_expansion_library
        except TypeError:
            logger.warn("expansion state of library could not be stored")

    def redo_expansion_state(self):
        if self.__expansion_state:
            # print "\n\n redo of state machine {0} \n\n".format(self.__my_selected_sm_id)
            try:
                for library_path, library_row_expanded in self.__expansion_state.iteritems():
                    library_row_iter = self.library_row_iter_dict_by_library_path[library_path]
                    if library_row_iter:  # may elements are missing afterwards
                        library_row_path = self.library_tree_store.get_path(library_row_iter)
                        if library_row_expanded:
                            self.view.expand_to_path(library_row_path)
                            # print library_path
            except (TypeError, KeyError):
                logger.warn("expansion state of library tree could not be re-done")

    def update(self):
        logger.info("Update of library_tree controller called")
        self.store_expansion_state()
        self.library_tree_store.clear()
        self.library_row_iter_dict_by_library_path.clear()
        for library_key, library_item in library_manager.libraries.iteritems():
            self.insert_rec(None, library_key, library_item, "")
        self.redo_expansion_state()

    def insert_rec(self, parent, library_key, library_item, library_path):
        #logger.debug("Add new library to tree store: %s" % library_key)
        tree_item = self.library_tree_store.insert_before(parent, None, (library_key, library_item, library_path))
        if library_path == "":
            library_path = library_key
        else:
            library_path = library_path + "/" + library_key
        self.library_row_iter_dict_by_library_path[library_path] = tree_item
        if isinstance(library_item, dict):
            #logger.debug("Found library container: %s" % library_key)
            for child_key, child_item in library_item.iteritems():
                self.insert_rec(tree_item, child_key, child_item, library_path)

    def on_cursor_changed(self, widget):
        # The user clicked on an entry in the tree store
        return

    def insert_button_clicked(self, widget, as_template=False):
        smm_m = self.state_machine_manager_model
        (model, row) = self.view.get_selection().get_selected()
        library_key = model[row][0]
        library = model[row][1]
        library_path = model[row][2]

        current_selection = smm_m.state_machines[smm_m.selected_state_machine_id].selection
        if len(current_selection.get_states()) > 1 or len(current_selection.get_states()) == 0:
            logger.error("Please select exactly one state for the insertion of a library")
            return

        current_state_m = current_selection.get_states()[0]
        current_state = current_state_m.state
        if not isinstance(current_state, ContainerState):
            logger.error("Libraries can only be inserted in container states")
            return

        logger.debug("Link library state %s (with file path %s, and library path %s) into the state machine" %
                     (str(library_key), str(library), str(library_path)))

        library_state = LibraryState(library_path, library_key, "0.1", library_key)

        # If inserted as library, we can just insert the library state
        if not as_template:
            current_state.add_state(library_state)
        # If inserted as template, we have to extract the state_copy and load the meta data manually
        else:
            template = library_state.state_copy
            current_state.add_state(template)

            from os.path import join
            lib_os_path, _, _ = library_manager.get_os_path_to_library(library_state.library_path,
                                                                       library_state.library_name)
            root_state_path = join(lib_os_path, template.state_id)
            template_m = current_state_m.states[template.state_id]
            template_m.load_meta_data(root_state_path)
            # Causes the template to be resized
            template_m.temp['gui']['editor']['template'] = True

    def open_button_clicked(self, widget):
        try:
            self.open_library_as_state_machine()
        except (AttributeError, ValueError) as e:
            logger.error("Could not open library: {0}".format(e))

    def open_run_button_clicked(self, widget):
        from rafcon.statemachine.singleton import state_machine_execution_engine
        state_machine = self.open_library_as_state_machine()
        state_machine_execution_engine.start(state_machine.state_machine_id)

    def open_library_as_state_machine(self):
        from rafcon.statemachine.singleton import global_storage
        smm_m = self.state_machine_manager_model
        (model, row) = self.view.get_selection().get_selected()
        library_path = model[row][1]

        logger.debug("Opening library as state-machine from path '{0}'".format(library_path))
        state_machine, version, creation_time = global_storage.load_statemachine_from_yaml(library_path)

        smm_m.state_machine_manager.add_state_machine(state_machine)
        return state_machine



