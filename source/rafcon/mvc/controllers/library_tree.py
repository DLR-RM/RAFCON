"""
.. module:: library_tree
   :platform: Unix, Windows
   :synopsis: The module holds controller tree overview of all mounted libraries and access to general
     functionality to add element or templates of those to a state-machine.

.. moduleauthor:: Rico Belder, Sebastain Brunner, Franz Steinmetz


"""

import os
import gtk
import gobject
from functools import partial

import rafcon.mvc.statemachine_helper as statemachine_helper
from rafcon.mvc.controllers.extended_controller import ExtendedController

from rafcon.statemachine.states.library_state import LibraryState
from rafcon.statemachine.singleton import library_manager

from rafcon.utils import log
logger = log.get_logger(__name__)


class LibraryTreeController(ExtendedController):
    def __init__(self, model=None, view=None, state_machine_manager_model=None):
        ExtendedController.__init__(self, model, view)
        self.library_tree_store = gtk.TreeStore(str, gobject.TYPE_PYOBJECT, str)
        view.set_model(self.library_tree_store)

        view.drag_source_set(gtk.gdk.BUTTON1_MASK, [('STRING', 0, 0)], gtk.gdk.ACTION_COPY)

        self.state_machine_manager_model = state_machine_manager_model

        self.library_row_iter_dict_by_library_path = {}
        self.__expansion_state = None

        self.update()

    def register_adapters(self):
        pass

    def register_view(self, view):
        self.view.connect('button_press_event', self.mouse_click)

        self.view.connect("drag-data-get", self.on_drag_data_get)
        self.view.connect("drag-begin", self.on_drag_begin)

    def mouse_click(self, widget, event=None):
        # Double click with left mpuse button
        if event.type == gtk.gdk._2BUTTON_PRESS and event.button == 1:
            (model, row) = self.view.get_selection().get_selected()
            if isinstance(model[row][1], dict):  # double click on folder, not library
                state_row_path = self.library_tree_store.get_path(row)
                if state_row_path is not None:
                    if self.view.row_expanded(state_row_path):
                        self.view.collapse_row(state_row_path)
                    else:
                        self.view.expand_to_path(state_row_path)
                return False
            self.open_button_clicked(None)
            return True

        # Single right click
        if event.type == gtk.gdk.BUTTON_PRESS and event.button == 3:
            menu = gtk.Menu()
            add_link_menu_item = gtk.ImageMenuItem(gtk.STOCK_ADD)
            add_link_menu_item.set_label("Add as library (link)")
            add_link_menu_item.connect("activate", partial(self.insert_button_clicked, as_template=False))
            add_link_menu_item.set_always_show_image(True)

            add_template_menu_item = gtk.ImageMenuItem(gtk.STOCK_COPY)
            add_template_menu_item.set_label("Add as template (copy)")
            add_template_menu_item.connect("activate", partial(self.insert_button_clicked, as_template=True))
            add_template_menu_item.set_always_show_image(True)

            open_menu_item = gtk.ImageMenuItem(gtk.STOCK_OPEN)
            open_menu_item.set_label("Open")
            open_menu_item.connect("activate", self.open_button_clicked)
            open_menu_item.set_always_show_image(True)

            open_run_menu_item = gtk.ImageMenuItem(gtk.STOCK_MEDIA_PLAY)
            open_run_menu_item.set_label("Open and run")
            open_run_menu_item.connect("activate", self.open_run_button_clicked)
            open_run_menu_item.set_always_show_image(True)

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
        self.store_expansion_state()
        self.library_tree_store.clear()
        self.library_row_iter_dict_by_library_path.clear()
        for library_key, library_item in library_manager.libraries.iteritems():
            self.insert_rec(None, library_key, library_item, "")
        self.redo_expansion_state()
        logger.info("Libraries have been updated")

    def insert_rec(self, parent, library_key, library_item, library_path):
        tree_item = self.library_tree_store.insert_before(parent, None, (library_key, library_item, library_path))
        if isinstance(library_item, dict) and not library_item:
            return
        if not library_path:
            library_path = library_key
        else:
            library_path = os.path.join(library_path, library_key)
        self.library_row_iter_dict_by_library_path[library_path] = tree_item
        if isinstance(library_item, dict):
            for child_library_key, child_library_item in library_item.iteritems():
                self.insert_rec(tree_item, child_library_key, child_library_item, library_path)

    def on_drag_data_get(self, widget, context, data, info, time):
        """dragged state is inserted and its state_id sent to the receiver

        :param widget:
        :param context:
        :param data: SelectionData: contains state_id
        :param info:
        :param time:
        """
        library_state = self._get_selected_library_state()
        if statemachine_helper.insert_state(library_state, False):
            data.set_text(library_state.state_id)

    def on_drag_begin(self, widget, context):
        """replace drag icon

        :param widget:
        :param context:
        """
        self.view.drag_source_set_icon_stock(gtk.STOCK_NEW)

    def insert_button_clicked(self, widget, as_template=False):
        statemachine_helper.insert_state(self._get_selected_library_state(), as_template)

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
        from rafcon.statemachine.storage import storage
        smm_m = self.state_machine_manager_model
        (model, row) = self.view.get_selection().get_selected()
        library_path = model[row][1]

        logger.debug("Opening library as state-machine from path '{0}'".format(library_path))
        state_machine = storage.load_statemachine_from_path(library_path)

        smm_m.state_machine_manager.add_state_machine(state_machine)
        return state_machine

    def _get_selected_library_state(self):
        """Returns the LibraryState which was selected in the LibraryTree

        :return: selected state in TreeView
        :rtype: LibraryState
        """
        (model, row) = self.view.get_selection().get_selected()
        library_key = model[row][0]
        library = model[row][1]
        library_path = model[row][2]

        if isinstance(library, dict):
            return None

        logger.debug("Link library state %s (with file path %s, and library path %s) into the state machine" %
                     (str(library_key), str(library), str(library_path)))

        return LibraryState(library_path, library_key, "0.1", library_key)