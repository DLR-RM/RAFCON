# Copyright (C) 2016-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gi.repository import GLib
from gi.repository import Gtk
from gi.repository import Gdk
from builtins import range
from builtins import str

from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.helpers.label import react_to_event, is_event_of_key_string
from rafcon.gui.models.selection import Selection, reduce_to_parent_states
from rafcon.utils import log

module_logger = log.get_logger(__name__)


class AbstractTreeViewController(ExtendedController):
    """Abstract base class for controller having a Gtk.Tree view with a Gtk.ListStore or a Gtk.TreeStore

    The class implements methods for e.g. handling editable cells and offers default callback methods for various
    signals and the skeleton structure for the multi-selection methods so that those can be reused finally in ListViews
    and TreeViews.

    :ivar store: List/Tree store that set by inherit class
    :ivar Gtk.TreeView tree_view: Tree view that set by inherit class
    :ivar int ID_STORAGE_ID: Index of core element id represented by row in list store and
        used to select entries set by inherit class
    :ivar int MODEL_STORAGE_ID: Index of model represented by row in list store and
        used to update selections in state machine or tree view set by inherit class
    """
    ID_STORAGE_ID = None
    MODEL_STORAGE_ID = None
    CORE_ELEMENT_CLASS = None
    _logger = None

    def __init__(self, model, view, tree_view, store, logger=None):
        super(AbstractTreeViewController, self).__init__(model, view)
        self._logger = logger if logger is not None else module_logger
        self._do_selection_update = False
        self._last_path_selection = None

        # setup_tree_view
        self.tree_view = tree_view
        self.tree_view.set_model(store)
        self.store = store
        self._tree_selection = self.tree_view.get_selection()

        self.active_entry_widget = None
        self.widget_columns = self.tree_view.get_columns()
        self.signal_handlers = []
        self.expose_event_count_after_key_release = 0
        self.__attached_renderers = list()

    def destroy(self):
        super(AbstractTreeViewController, self).destroy()
        # self.tree_view.destroy() # does not help
        # self._tree_selection.destroy() # creates problems with selection update notification
        # print("disconnect in", self.__class__.__name__, self.signal_handlers)
        for widget, handler_id in self.signal_handlers:
            # print("disconnect", widget, handler_id)
            widget.disconnect(handler_id)
            # widget.destroy() # creates problems with selection update notification
        self.signal_handlers = []
        # GTK Todo: check if necessary and search for replacement
        # for column in self.widget_columns:
        #     renderers = column.get_cells()
        #     for r in renderers:
        #         r.ctrl = None
        #         r.destroy()
        # delete reference to the controllers of renderers
        for renderer in self.__attached_renderers:
            renderer.ctrl = None
        del self.__attached_renderers[:]

    def register_view(self, view):
        """Register callbacks for button press events and selection changed"""
        super(AbstractTreeViewController, self).register_view(view)
        self.signal_handlers.append((self._tree_selection,
                                     self._tree_selection.connect('changed', self.selection_changed)))
        # self.handler_ids.append((self.tree_view,
        #                          self.tree_view.connect('key-press-event', self.tree_view_keypress_callback)))
        self.tree_view.connect('key-release-event', self.on_key_release_event)
        self.tree_view.connect('button-release-event', self.tree_view_keypress_callback)
        # key press is needed for tab motion but needs to be registered already here TODO why?
        self.tree_view.connect('key-press-event', self.tree_view_keypress_callback)
        self._tree_selection.set_mode(Gtk.SelectionMode.MULTIPLE)
        self.update_selection_sm_prior()

    def get_view_selection(self):
        """Get actual tree selection object and all respective models of selected rows"""
        if not self.MODEL_STORAGE_ID:
            return None, None

        # avoid selection requests on empty tree views -> case warnings in gtk3
        if len(self.store) == 0:
            paths = []
        else:
            model, paths = self._tree_selection.get_selected_rows()

        # get all related models for selection from respective tree store field
        selected_model_list = []
        for path in paths:
            model = self.store[path][self.MODEL_STORAGE_ID]
            selected_model_list.append(model)
        return self._tree_selection, selected_model_list

    def get_state_machine_selection(self):
        """An abstract getter method for state machine selection

        The method has to be implemented by inherit classes and hands generally a filtered set of selected elements.

        :return: selection object, filtered set of selected elements
        :rtype: rafcon.gui.selection.Selection, set
        """
        raise NotImplementedError

    def get_selections(self):
        """Get current model selection status in state machine selection (filtered according the purpose of the widget)
        and tree selection of the widget"""
        sm_selection, sm_filtered_selected_model_set = self.get_state_machine_selection()
        tree_selection, selected_model_list = self.get_view_selection()
        return tree_selection, selected_model_list, sm_selection, sm_filtered_selected_model_set

    def update_selection_sm_prior(self):
        """State machine prior update of tree selection"""
        raise NotImplementedError

    @ExtendedController.observe("sm_selection_changed_signal", signal=True)
    def state_machine_selection_changed(self, state_machine_m, signal_name, signal_msg):
        """Notify tree view about state machine selection"""
        if self.CORE_ELEMENT_CLASS in signal_msg.arg.affected_core_element_classes:
            self.update_selection_sm_prior()

    def update_selection_self_prior(self):
        """Tree view prior update of state machine selection"""
        raise NotImplementedError

    def selection_changed(self, widget, event=None):
        """Notify state machine about tree view selection"""
        # print("self prior", type(self).__name__, self._do_selection_update, "select changed", widget, event, self)
        self.update_selection_self_prior()

    def on_right_click_menu(self):
        """An abstract method called after right click events"""
        raise NotImplementedError

    def on_add(self, widget, data=None):
        """An abstract add method for a respective new core element and final selection of those"""
        raise NotImplementedError

    def add_action_callback(self, *event, **kwargs):
        """Callback method for add action"""
        if react_to_event(self.view, self.tree_view, event) and self.active_entry_widget is None:
            self.on_add(None)
            return True

    def remove_core_element(self, model):
        """An abstract remove method that removes respective core element by handed model or object

        The method has to be implemented by inherit classes

        :param StateElementModel model: Model which core element should be removed
        :return:
        """
        raise NotImplementedError

    def remove_core_elements(self, models):
        """An abstract remove method that removes respective core element by handed models or objects

        :param list models: Model which core element should be removed
        :return:
        """
        for model in models:
            self.remove_core_element(model)

    def on_remove(self, widget, data=None):
        """Remove respective selected core elements and select the next one"""
        raise NotImplementedError

    def remove_action_callback(self, *event, **kwargs):
        """Callback method for remove action

        The method checks whether a shortcut ('Delete') is in the gui config model which shadow the delete functionality
        of maybe active a entry widget. If a entry widget is active the remove callback return with None.
        """
        if react_to_event(self.view, self.tree_view, event) and \
                not (self.active_entry_widget and not is_event_of_key_string(event, 'Delete')):
            self.on_remove(None)
            return True

    def copy_action_callback(self, *event, **kwargs):
        """Callback method for copy action"""
        if react_to_event(self.view, self.tree_view, event) and self.active_entry_widget is None:
            sm_selection, sm_selected_model_list = self.get_state_machine_selection()
            # only list specific elements are copied by widget
            if sm_selection is not None:
                sm_selection.set(sm_selected_model_list)
                global_clipboard.copy(sm_selection)
                return True

    def cut_action_callback(self, *event, **kwargs):
        """Callback method for copy action"""
        if react_to_event(self.view, self.tree_view, event) and self.active_entry_widget is None:
            sm_selection, sm_selected_model_list = self.get_state_machine_selection()
            # only list specific elements are cut by widget
            if sm_selection is not None:
                sm_selection.set(sm_selected_model_list)
                global_clipboard.cut(sm_selection)
                return True

    def get_path(self):
        """Get path to the currently selected entry row

        :return: path to the tree view cursor row, None if there is no selection
        :rtype: tuple
        """
        # the cursor is a tuple containing the current path and the focused column
        return self.tree_view.get_cursor()[0]

    def _apply_value_on_edited_and_focus_out(self, renderer, apply_method):
        """Sets up the renderer to apply changed when loosing focus

        The default behaviour for the focus out event dismisses the changes in the renderer. Therefore we setup
        handlers for that event, applying the changes.

        :param Gtk.CellRendererText renderer: The cell renderer who's changes are to be applied on focus out events
        :param apply_method: The callback method applying the newly entered value
        """
        assert isinstance(renderer, Gtk.CellRenderer)

        def remove_all_handler(renderer):
            """Remove all handler for given renderer and its editable

            :param renderer: Renderer of the respective column which is edit by a entry widget, at the moment
            """
            def remove_handler(widget, data_name):
                """Remove handler from given widget

                :param Gtk.Widget widget: Widget from which a handler is to be removed
                :param data_name: Name of the data of the widget in which the handler id is stored
                """
                handler_id = getattr(widget, data_name)
                if widget.handler_is_connected(handler_id):
                    widget.disconnect(handler_id)

            editable = getattr(renderer, "editable")
            remove_handler(editable, "focus_out_handler_id")
            remove_handler(editable, "cursor_move_handler_id")
            remove_handler(editable, "insert_at_cursor_handler_id")
            remove_handler(editable, "entry_widget_expose_event_handler_id")
            remove_handler(renderer, "editing_cancelled_handler_id")

        def on_focus_out(entry, event):
            """Applies the changes to the entry

            :param Gtk.Entry entry: The entry that was focused out
            :param Gtk.Event event: Event object with information about the event
            """
            renderer.remove_all_handler(renderer)
            if renderer.ctrl.get_path() is None:
                return
            # Originally we had to use idle_add to prevent core dumps:
            # https://mail.gnome.org/archives/gtk-perl-list/2005-September/msg00143.html
            # Gtk TODO: use idle_add if there are problems
            apply_method(renderer.ctrl.get_path(), entry.get_text())

        def on_cursor_move_in_entry_widget(entry, step, count, extend_selection):
            """Trigger scroll bar adjustments according active entry widgets cursor change
            """
            self.tree_view_keypress_callback(entry, None)

        def on_editing_started(renderer, editable, path):
            """Connects the a handler for the focus-out-event of the current editable

            :param Gtk.CellRendererText renderer: The cell renderer who's editing was started
            :param Gtk.CellEditable editable: interface for editing the current TreeView cell
            :param str path: the path identifying the edited cell
            """
            # secure scrollbar adjustments on active cell
            ctrl = renderer.ctrl
            [path, focus_column] = ctrl.tree_view.get_cursor()
            if path:
                ctrl.tree_view.scroll_to_cell(path, ctrl.widget_columns[ctrl.widget_columns.index(focus_column)],
                                              use_align=False)

            editing_cancelled_handler_id = renderer.connect('editing-canceled', on_editing_canceled)
            focus_out_handler_id = editable.connect('focus-out-event', on_focus_out)
            cursor_move_handler_id = editable.connect('move-cursor', on_cursor_move_in_entry_widget)
            insert_at_cursor_handler_id = editable.connect("insert-at-cursor", on_cursor_move_in_entry_widget)
            entry_widget_expose_event_handler_id = editable.connect("draw", self.on_entry_widget_draw_event)
            # Store reference to editable and signal handler ids for later access when removing the handlers
            # see https://gitlab.gnome.org/GNOME/accerciser/commit/036689e70304a9e98ce31238dfad2432ad4c78ea
            # originally with renderer.set_data()
            # renderer.set_data("editable", editable)
            setattr(renderer, "editable", editable)
            setattr(renderer, "editing_cancelled_handler_id", editing_cancelled_handler_id)
            # editable
            setattr(editable, "focus_out_handler_id", focus_out_handler_id)
            setattr(editable, "cursor_move_handler_id", cursor_move_handler_id)
            setattr(editable, "insert_at_cursor_handler_id", insert_at_cursor_handler_id)
            setattr(editable, "entry_widget_expose_event_handler_id", entry_widget_expose_event_handler_id)
            ctrl.active_entry_widget = editable

        def on_edited(renderer, path, new_value_str):
            """Calls the apply method with the new value

            :param Gtk.CellRendererText renderer: The cell renderer that was edited
            :param str path: The path string of the renderer
            :param str new_value_str: The new value as string
            """
            renderer.remove_all_handler(renderer)
            apply_method(path, new_value_str)
            renderer.ctrl.active_entry_widget = None

        def on_editing_canceled(renderer):
            """Disconnects the focus-out-event handler of cancelled editable

            :param Gtk.CellRendererText renderer: The cell renderer who's editing was cancelled
            """
            remove_all_handler(renderer)
            renderer.ctrl.active_entry_widget = None

        renderer.remove_all_handler = remove_all_handler
        renderer.ctrl = self
        # renderer.connect('editing-started', on_editing_started)
        # renderer.connect('edited', on_edited)
        self.__attached_renderers.append(renderer)
        self.connect_signal(renderer, 'editing-started', on_editing_started)
        self.connect_signal(renderer, 'edited', on_edited)

    def get_path_for_core_element(self, core_element_id):
        """Get path to the row representing core element described by handed core_element_id

        :param core_element_id: Core element identifier used in the respective list store column
        :rtype: tuple
        :return: path
        """
        raise NotImplementedError

    def tree_view_keypress_callback(self, widget, event):
        """General method to adapt widget view and controller behavior according the key press/release and
        button release events

            Here the scrollbar motion to follow key cursor motions in editable is already in.

        :param Gtk.TreeView widget: The tree view the controller use
        :param Gdk.Event event: The key press event
        :return:
        """
        current_row_path, current_focused_column = self.tree_view.get_cursor()
        # print(current_row_path, current_focused_column)
        if isinstance(widget, Gtk.TreeView) and not self.active_entry_widget:  # avoid jumps for active entry widget
            pass
            # cursor motion/selection changes (e.g. also by button release event)
            # if current_row_path is not None and len(current_row_path) == 1 and isinstance(current_row_path[0], int):
            #     self.tree_view.scroll_to_cell(current_row_path[0], current_focused_column, use_align=True)
            # # else:
            # #     self._logger.debug("A ListViewController aspects a current_row_path of dimension 1 with integer but"
            # #                        " it is {0} and column is {1}".format(current_row_path, current_focused_column))
        elif isinstance(widget, Gtk.Entry) and self.view.scrollbar_widget is not None:
            # calculate the position of the scrollbar to be always centered with the entry widget cursor
            # TODO check how to get sufficient the scroll-offset in the entry widget -> some times zero when not
            # TODO the scrollbar is one step behind cursor -> so jump from pos1 to end works not perfect
            entry_widget_scroll_offset, entry_widget_cursor_position, entry_widget_text_length = \
                widget.get_properties(*["scroll-offset", "cursor-position", "text-length"])
            cell_rect_of_entry_widget = widget.get_allocation()

            horizontal_scroll_bar = self.view.scrollbar_widget.get_hscrollbar()
            # entry_widget_text_length must be greater than zero otherwise DevisionByZero Exception
            if horizontal_scroll_bar is not None and float(entry_widget_text_length) > 0:
                adjustment = horizontal_scroll_bar.get_adjustment()
                layout_pixel_width = widget.get_layout().get_pixel_size()[0]
                # print("rel_pos pices", cell_rect_of_entry_widget.x,)
                #     int(layout_pixel_width*float(entry_widget_cursor_position)/float(entry_widget_text_length))
                rel_pos = cell_rect_of_entry_widget.x - entry_widget_scroll_offset + \
                    int(layout_pixel_width*float(entry_widget_cursor_position)/float(entry_widget_text_length))

                # optimize rel_pos for better user support
                bounds = widget.get_selection_bounds()
                if bounds and bounds[1] - bounds[0] == len(widget.get_text()):
                    # if text is fully selected stay in front as far as possible
                    rel_pos = cell_rect_of_entry_widget.x
                    if self._horizontal_scrollbar_stay_in_front_if_possible():
                        return True
                else:
                    # try to stay long at the beginning of the columns if the columns fully fit in
                    rel_space = adjustment.get_page_size() - cell_rect_of_entry_widget.x
                    if cell_rect_of_entry_widget.x + widget.get_layout().get_pixel_size()[0] < \
                            adjustment.get_page_size():
                        rel_pos = 0.
                    elif rel_space and rel_pos <= rel_space:
                        # accelerate the showing of the first columns
                        rel_pos = rel_pos + rel_pos*3.*(rel_pos - rel_space)/adjustment.get_page_size()
                        rel_pos = 0. if rel_pos <= 0 else rel_pos
                    else:
                        # and jump to the end of the scroller space if close to the upper limit
                        rel_pos = adjustment.get_upper() if rel_pos + 2*entry_widget_scroll_offset > adjustment.get_upper() else rel_pos
                self._put_horizontal_scrollbar_onto_rel_pos(rel_pos)

    def _put_horizontal_scrollbar_onto_rel_pos(self, rel_pos):
        horizontal_scroll_bar = self.view.scrollbar_widget.get_hscrollbar()
        adjustment = horizontal_scroll_bar.get_adjustment()
        value = int(float(adjustment.get_upper() - adjustment.get_page_size())*rel_pos/float(adjustment.get_upper()))
        # Gtk TODO: evtl. use idle_add if problems occur
        adjustment.set_value(value)

    def _horizontal_scrollbar_stay_in_front_if_possible(self):
        if self.active_entry_widget:
            horizontal_scroll_bar = self.view.scrollbar_widget.get_hscrollbar()
            adjustment = horizontal_scroll_bar.get_adjustment()
            cell_rect_of_entry_widget = self.active_entry_widget.get_allocation()
            rel_space = adjustment.get_page_size() - cell_rect_of_entry_widget.x
            if rel_space > 20:
                self._put_horizontal_scrollbar_onto_rel_pos(0.)
                return True

    def on_key_release_event(self, widget, event):
        self.expose_event_count_after_key_release = 0
        # self.tree_view_keypress_callback(widget, event)

    def on_entry_widget_draw_event(self, widget, event):
        # take three signals because sometimes expose events come before cursor is set
        if self.expose_event_count_after_key_release < 3:
            AbstractTreeViewController.tree_view_keypress_callback(self, widget, event)
        self.expose_event_count_after_key_release += 1


class ListViewController(AbstractTreeViewController):
    """Base class for controller having a Gtk.Tree view with a Gtk.ListStore

    The class implements methods for e.g. handling (multi-)selection and offers default callback methods for various
    signals and includes a move and edit by tab-key feature.

    :ivar Gtk.ListStore list_store: List store that set by inherit class
    :ivar Gtk.TreeView tree_view: Tree view that set by inherit class
    :ivar int ID_STORAGE_ID: Index of core element id represented by row in list store and
        used to select entries set by inherit class
    :ivar int MODEL_STORAGE_ID: Index of model represented by row in list store and
        used to update selections in state machine or tree view set by inherit class
    """
    _logger = None

    def __init__(self, model, view, tree_view, list_store, logger=None):
        assert isinstance(list_store, Gtk.ListStore)
        super(ListViewController, self).__init__(model, view, tree_view, list_store, logger)
        self.list_store = list_store

    def register_view(self, view):
        """Register callbacks for button press events and selection changed"""
        super(ListViewController, self).register_view(view)
        self.tree_view.connect('button_press_event', self.mouse_click)

    def on_remove(self, widget, data=None):
        """Removes respective selected core elements and select the next one"""

        path_list = None
        if self.view is not None:
            model, path_list = self.tree_view.get_selection().get_selected_rows()
        old_path = self.get_path()
        models = [self.list_store[path][self.MODEL_STORAGE_ID] for path in path_list] if path_list else []
        if models:
            try:
                self.remove_core_elements(models)
            except AttributeError as e:
                self._logger.warning("The respective core element of {1}.list_store couldn't be removed. -> {0}"
                                  "".format(e, self.__class__.__name__))
            if len(self.list_store) > 0:
                self.tree_view.set_cursor(min(old_path[0], len(self.list_store) - 1))
            return True
        else:
            self._logger.warning("Please select an element to be removed.")

    def get_state_machine_selection(self):
        """An abstract getter method for state machine selection

        The method maybe has to be re-implemented by inherit classes and hands generally a filtered set of
        selected elements.

        :return: selection object it self, filtered set of selected elements
        :rtype: rafcon.gui.selection.Selection, set
        """
        sm_selection = self.model.get_state_machine_m().selection if self.model.get_state_machine_m() else None
        return sm_selection, sm_selection.get_selected_elements_of_core_class(self.CORE_ELEMENT_CLASS) if sm_selection else set()

    def mouse_click(self, widget, event=None):
        """Implements shift- and control-key handling features for mouse button press events explicit

         The method is implements a fully defined mouse pattern to use shift- and control-key for multi-selection in a
         TreeView and a ListStore as model. It avoid problems caused by special renderer types like the text combo
         renderer by stopping the callback handler to continue with notifications.

        :param Gtk.Object widget: Object which is the source of the event
        :param Gtk.Event event: Event generated by mouse click
        :rtype: bool
        """

        if event.type == Gdk.EventType.BUTTON_PRESS:
            pthinfo = self.tree_view.get_path_at_pos(int(event.x), int(event.y))

            if not bool(event.get_state() & Gdk.ModifierType.CONTROL_MASK) \
                    and not bool(event.get_state() & Gdk.ModifierType.SHIFT_MASK) and \
                    event.type == Gdk.EventType.BUTTON_PRESS and event.get_button()[1] == 3:
                if pthinfo is not None:
                    model, paths = self._tree_selection.get_selected_rows()
                    # print(paths)
                    if pthinfo[0] not in paths:
                        # self._logger.info("force single selection for right click")
                        self.tree_view.set_cursor(pthinfo[0])
                        self._last_path_selection = pthinfo[0]
                    else:
                        # self._logger.info("single- or multi-selection for right click")
                        pass
                    self.on_right_click_menu()
                    return True

            if (bool(event.get_state() & Gdk.ModifierType.CONTROL_MASK) or \
                bool(event.get_state() & Gdk.ModifierType.SHIFT_MASK)) and \
                    event.type == Gdk.EventType.BUTTON_PRESS and event.get_button()[1] == 3:
                return True

            if not bool(event.get_state() & Gdk.ModifierType.SHIFT_MASK) and event.get_button()[1] == 1:
                if pthinfo is not None:
                    # self._logger.info("last select row {}".format(pthinfo[0]))
                    self._last_path_selection = pthinfo[0]
                # else:
                #     self._logger.info("deselect rows")
                #     self.tree_selection.unselect_all()

            if bool(event.get_state() & Gdk.ModifierType.SHIFT_MASK) and event.get_button()[1] == 1:
                # self._logger.info("SHIFT adjust selection range")
                model, paths = self._tree_selection.get_selected_rows()
                # print(model, paths, pthinfo[0])
                if paths and pthinfo and pthinfo[0]:
                    if self._last_path_selection[0] <= pthinfo[0][0]:
                        new_row_ids_selected = list(range(self._last_path_selection[0], pthinfo[0][0]+1))
                    else:
                        new_row_ids_selected = list(range(self._last_path_selection[0], pthinfo[0][0]-1, -1))
                    # self._logger.info("range to select {0}, {1}".format(new_row_ids_selected, model))
                    self._tree_selection.unselect_all()
                    for path in new_row_ids_selected:
                        self._tree_selection.select_path(path)
                    return True
                else:
                    # self._logger.info("nothing selected {}".format(model))
                    if pthinfo and pthinfo[0]:
                        self._last_path_selection = pthinfo[0]

            if bool(event.get_state() & Gdk.ModifierType.CONTROL_MASK) and event.get_button()[1] == 1:
                # self._logger.info("CONTROL adjust selection range")
                model, paths = self._tree_selection.get_selected_rows()
                # print(model, paths, pthinfo[0])
                if paths and pthinfo and pthinfo[0]:
                    if pthinfo[0] in paths:
                        self._tree_selection.unselect_path(pthinfo[0])
                    else:
                        self._tree_selection.select_path(pthinfo[0])
                    return True
                elif pthinfo and pthinfo[0]:
                    self._tree_selection.select_path(pthinfo[0])
                    return True

        elif event.type == Gdk.EventType._2BUTTON_PRESS:
            self._handle_double_click(event)

    def _handle_double_click(self, event):
        """ Double click with left mouse button focuses the element"""
        if event.get_button()[1] == 1:  # Left mouse button
            path_info = self.tree_view.get_path_at_pos(int(event.x), int(event.y))
            if path_info:  # Valid entry was clicked on
                path = path_info[0]
                iter = self.list_store.get_iter(path)
                model = self.list_store.get_value(iter, self.MODEL_STORAGE_ID)
                selection = self.model.get_state_machine_m().selection
                selection.focus = model

    def update_selection_sm_prior(self):
        """State machine prior update of tree selection"""
        if self._do_selection_update:
            return
        self._do_selection_update = True
        tree_selection, selected_model_list, sm_selection, sm_selected_model_list = self.get_selections()
        if tree_selection is not None:
            for path, row in enumerate(self.list_store):
                model = row[self.MODEL_STORAGE_ID]
                if model not in sm_selected_model_list and model in selected_model_list:
                    tree_selection.unselect_path(Gtk.TreePath.new_from_indices([path]))
                if model in sm_selected_model_list and model not in selected_model_list:
                    tree_selection.select_path(Gtk.TreePath.new_from_indices([path]))

        self._do_selection_update = False

    def update_selection_self_prior(self):
        """Tree view prior update of state machine selection"""
        if self._do_selection_update:
            return
        self._do_selection_update = True
        tree_selection, selected_model_list, sm_selection, sm_selected_model_list = self.get_selections()
        if isinstance(sm_selection, Selection):
            sm_selection.handle_prepared_selection_of_core_class_elements(self.CORE_ELEMENT_CLASS, selected_model_list)
        self._do_selection_update = False

    def select_entry(self, core_element_id, by_cursor=True):
        """Selects the row entry belonging to the given core_element_id by cursor or tree selection"""
        for row_num, element_row in enumerate(self.list_store):
            # Compare data port ids
            if element_row[self.ID_STORAGE_ID] == core_element_id:
                if by_cursor:
                    self.tree_view.set_cursor(row_num)
                else:
                    self.tree_view.get_selection().select_path((row_num, ))
                break

    def get_path_for_core_element(self, core_element_id):
        """Get path to the row representing core element described by handed core_element_id

        :param core_element_id: Core element identifier used in the respective list store column
        :rtype: tuple
        :return: path
        """
        for row_num, element_row in enumerate(self.list_store):
            # Compare data port ids
            if element_row[self.ID_STORAGE_ID] == core_element_id:
                return row_num,

    def get_list_store_row_from_cursor_selection(self):
        """Returns the list_store_row of the currently by cursor selected row entry

        :return: List store row, None if there is no selection
        :rtype: Gtk.TreeModelRow
        """
        path = self.get_path()
        if path is not None:
            return self.list_store[path]

    def tree_view_keypress_callback(self, widget, event):
        """Tab back and forward tab-key motion in list widget and the scrollbar motion to follow key cursor motions

         The method introduce motion and edit functionality by using "tab"- or "shift-tab"-key for a Gtk.TreeView.
         It is designed to work with a Gtk.TreeView which model is a Gtk.ListStore and only uses text cell renderer.
         Additional, the TreeView is assumed to be used as a list not as a tree.
         With the "tab"-key the cell on the right site of the actual focused cell is started to be edit. Changes in the
         Gtk.Entry-Widget are confirmed by emitting a 'edited'-signal. If the row ends the edit process continues
         with the first cell of the next row. With the "shift-tab"-key the inverse functionality of the "tab"-key is
         provided.
         The Controller over steps not editable cells.

        :param Gtk.TreeView widget: The tree view the controller use
        :param Gdk.Event event: The key press event
        :return:
        """
        # self._logger.info("key_value: " + str(event.keyval if event is not None else ''))
        if event and "GDK_KEY_PRESS" == event.type.value_name \
                and (event.keyval == Gdk.KEY_Tab or event.keyval == Gdk.KEY_ISO_Left_Tab):
            [path, focus_column] = self.tree_view.get_cursor()
            if not path:
                return False
            self.tree_view_keypress_callback.__func__.core_element_id = self.store[path][self.ID_STORAGE_ID]

            # finish active edit process
            if self.active_entry_widget is not None:
                text = self.active_entry_widget.get_buffer().get_text()
                if focus_column in self.widget_columns:
                    focus_column.get_cells()[0].emit('edited', path[0], text)

            # row could be updated by other call_backs caused by emitting 'edited' signal but selection stays an editable neighbor
            path = self.get_path_for_core_element(self.tree_view_keypress_callback.__func__.core_element_id)
            if event.keyval == Gdk.KEY_Tab:
                # logger.info("move right")
                direction = +1
            else:
                # logger.info("move left")
                direction = -1

            # get next row_id for focus
            if direction < 0 and focus_column is self.widget_columns[0] \
                    or direction > 0 and focus_column is self.widget_columns[-1]:
                if direction < 0 < path[0] or direction > 0 and not path[0] + 1 > len(self.store):
                    next_row = path[0] + direction
                else:
                    return False
            else:
                next_row = path[0]
            # get next column_id for focus
            focus_column_id = self.widget_columns.index(focus_column)
            if focus_column_id is not None:
                # search all columns for next editable cell renderer
                next_focus_column_id = 0
                for index in range(len(self.tree_view.get_model())):
                    test_id = focus_column_id + direction * index + direction
                    next_focus_column_id = test_id % len(self.widget_columns)
                    if test_id > len(self.widget_columns) - 1 or test_id < 0:
                        next_row = path[0] + direction
                        if next_row < 0 or next_row > len(self.tree_view.get_model()) - 1:
                            return False

                    if self.widget_columns[next_focus_column_id].get_cells()[0].get_property('editable'):
                        break
            else:
                return False

            del self.tree_view_keypress_callback.__func__.core_element_id
            # self._logger.info("self.tree_view.scroll_to_cell(next_row={0}, self.widget_columns[{1}] , use_align={2})"
            #              "".format(next_row, next_focus_column_id, False))
            # self.tree_view.scroll_to_cell(next_row, self.widget_columns[next_focus_column_id], use_align=False)
            self.tree_view.set_cursor_on_cell(Gtk.TreePath.new_from_indices([next_row]), self.widget_columns[
                next_focus_column_id], focus_cell=None, start_editing=True)
            return True
        else:
            super(ListViewController, self).tree_view_keypress_callback(widget, event)


class TreeViewController(AbstractTreeViewController):
    """Base class for controller having a Gtk.Tree view with a Gtk.TreeStore

    The class implements methods for e.g. handling (multi-)selection.

    :ivar Gtk.TreeStore tree_store: Tree store that set by inherit class
    :ivar Gtk.TreeView tree_view: Tree view that set by inherit class
    :ivar int ID_STORAGE_ID: Index of core element id represented by row in list store and
        used to select entries set by inherit class
    :ivar int MODEL_STORAGE_ID: Index of model represented by row in list store and
        used to update selections in state machine or tree view set by inherit class
    """
    _logger = None

    def __init__(self, model, view, tree_view, tree_store, logger=None):
        assert isinstance(tree_store, Gtk.TreeStore)
        super(TreeViewController, self).__init__(model, view, tree_view, tree_store, logger)
        self.tree_store = tree_store
        self._changed_id_to = {}

    def register_view(self, view):
        """Register callbacks for button press events and selection changed"""
        super(TreeViewController, self).register_view(view)
        # self.tree_view.connect('button_press_event', self.mouse_click)

    def get_path_for_core_element(self, core_element_id):
        """Get path to the row representing core element described by handed core_element_id

        :param core_element_id: Core element identifier used in the respective list store column
        :rtype: tuple
        :return: path
        """
        raise NotImplementedError  # TODO do it better maybe by reusing the method written for semantic data editor

    def iter_tree_with_handed_function(self, function, *function_args):
        """Iterate tree view with condition check function"""
        def iter_all_children(row_iter, function, function_args):
            if isinstance(row_iter, Gtk.TreeIter):
                function(row_iter, *function_args)
                for n in reversed(range(self.tree_store.iter_n_children(row_iter))):
                    child_iter = self.tree_store.iter_nth_child(row_iter, n)
                    iter_all_children(child_iter, function, function_args)
            else:
                self._logger.warning("Iter has to be TreeIter -> handed argument is: {0}".format(row_iter))

        # iter on root level of tree
        next_iter = self.tree_store.get_iter_first()
        while next_iter:
            iter_all_children(next_iter, function, function_args)
            next_iter = self.tree_store.iter_next(next_iter)

    def update_selection_sm_prior_condition(self, state_row_iter, selected_model_list, sm_selected_model_list):
        """State machine prior update of tree selection for one tree model row"""
        selected_path = self.tree_store.get_path(state_row_iter)
        tree_model_row = self.tree_store[selected_path]
        model = tree_model_row[self.MODEL_STORAGE_ID]

        if model not in sm_selected_model_list and model in selected_model_list:
            self._tree_selection.unselect_iter(state_row_iter)
        elif model in sm_selected_model_list and model not in selected_model_list:
            self.tree_view.expand_to_path(selected_path)
            self._tree_selection.select_iter(state_row_iter)

    def update_selection_self_prior_condition(self, state_row_iter, sm_selected_model_set, selected_model_list):
        """Tree view prior update of one model in the state machine selection"""
        selected_path = self.tree_store.get_path(state_row_iter)
        tree_model_row = self.tree_store[selected_path]
        model = tree_model_row[self.MODEL_STORAGE_ID]

        if model in sm_selected_model_set and model not in selected_model_list:
            sm_selected_model_set.remove(model)
        elif model not in sm_selected_model_set and model in selected_model_list:
            sm_selected_model_set.add(model)

    def check_selection_consistency(self, sm_check=True, tree_check=True):
        tree_selection, selected_model_list, sm_selection, sm_selected_model_list = self.get_selections()
        selected_model_list = list(reduce_to_parent_states(set(selected_model_list)))
        if not ((all([model in selected_model_list for model in sm_selected_model_list]) or not sm_check) and
                (all([model in sm_selected_model_list for model in selected_model_list]) or not tree_check)):
            self._logger.warning("Elements of sm and tree selection are not identical: \ntree: {0}\nsm:   {1}"
                                 "".format(selected_model_list, sm_selected_model_list))
            return False
        return True

    def update_selection_self_prior(self):
        """Tree view prior update of state machine selection"""
        if self._do_selection_update:
            return
        self._do_selection_update = True
        tree_selection, selected_model_list, sm_selection, sm_selected_model_set = self.get_selections()
        if isinstance(sm_selection, Selection):
            # current sm_selected_model_set will be updated and hand it back
            self.iter_tree_with_handed_function(self.update_selection_self_prior_condition,
                                                sm_selected_model_set, selected_model_list)
            sm_selection.handle_prepared_selection_of_core_class_elements(self.CORE_ELEMENT_CLASS, sm_selected_model_set)
            # TODO check if we can solve the difference that occurs e.g. while complex actions?, or same state paths!
            # -> models in selection for core element not in the tree the function iter tree + condition tolerates this
            if not set(selected_model_list) == sm_selected_model_set:
                self._logger.verbose("Difference between tree view selection: \n{0} \nand state machine selection: "
                                     "\n{1}".format(set(selected_model_list), sm_selected_model_set))

        # TODO check why sometimes not consistent with sm selection. e.g while modification history test
        if self.check_selection_consistency(sm_check=False):
            self.update_selection_sm_prior()
        self._do_selection_update = False

    def update_selection_sm_prior(self):
        """State machine prior update of tree selection"""
        if self._do_selection_update:
            return
        self._do_selection_update = True
        tree_selection, selected_model_list, sm_selection, sm_selected_model_list = self.get_selections()
        if tree_selection is not None:
            # self._logger.info("SM SELECTION IS: {2}\n{0}, \n{1}".format(selected_model_list, sm_selected_model_list,
            #                                                             tree_selection.get_mode()))
            self.iter_tree_with_handed_function(self.update_selection_sm_prior_condition,
                                                selected_model_list, sm_selected_model_list)
            self.check_selection_consistency()
        self._do_selection_update = False

    def on_add(self, widget, data=None):
        """An abstract add method for a respective new core element and final selection of those"""
        raise NotImplementedError

    def on_remove(self, widget, data=None):
        """Remove respective selected core elements and select the next one"""
        raise NotImplementedError

    def select_entry(self, core_element_id, by_cursor=True):
        """Selects the row entry belonging to the given core_element_id by cursor or tree selection"""
        path = self.get_path_for_core_element(core_element_id)
        if path:
            if by_cursor:
                self.tree_view.set_cursor(path)
            else:
                self.tree_view.get_selection().select_path(path)
        else:
            self._logger.warning("Path not valid: {0} (by_cursor {1})".format(str(core_element_id), str(by_cursor)))

    def tree_view_keypress_callback(self, widget, event):
        """Tab back and forward tab-key motion in list widget and the scrollbar motion to follow key cursor motions

         The method introduce motion and edit functionality by using "tab"- or "shift-tab"-key for a Gtk.TreeView.
         It is designed to work with a Gtk.TreeView which model is a Gtk.ListStore and only uses text cell renderer.
         Additional, the TreeView is assumed to be used as a list not as a tree.
         With the "tab"-key the cell on the right site of the actual focused cell is started to be edit. Changes in the
         Gtk.Entry-Widget are confirmed by emitting a 'edited'-signal. If the row ends the edit process continues
         with the first cell of the next row. With the "shift-tab"-key the inverse functionality of the "tab"-key is
         provided.
         The Controller over steps not editable cells.

        :param Gtk.TreeView widget: The tree view the controller use
        :param Gdk.Event event: The key press event
        :return:
        """
        # self._logger.info("key_value: " + str(event.keyval if event is not None else ''))
        # TODO works for root level or other single level of tree view but not for switching in between levels
        if event and "GDK_KEY_PRESS" == event.type.value_name \
                and (event.keyval == Gdk.KEY_Tab or event.keyval == Gdk.KEY_ISO_Left_Tab):
            [path, focus_column] = self.tree_view.get_cursor()
            # print("cursor ", path, focus_column)
            model, paths = self.tree_view.get_selection().get_selected_rows()
            if paths:
                path = paths[0]
                # print("tree selection", path, focus_column, paths)
            if not path:
                return False
            self.tree_view_keypress_callback.__func__.core_element_id = self.store[path][self.ID_STORAGE_ID]
            # print("core id", self.store[path][self.ID_STORAGE_ID], path, type(path))
            # finish active edit process
            if self.active_entry_widget is not None:
                text = self.active_entry_widget.get_buffer().get_text()
                if focus_column in self.widget_columns:
                    # print("path", ':'.join([str(elem) for elem in path]))
                    focus_column.get_cells()[0].emit('edited', ':'.join([str(elem) for elem in path]), text)

            # row could be updated by other call_backs caused by emitting 'edited' signal but selection stays an editable neighbor
            core_element_id = ':'.join([str(key) for key in self.tree_view_keypress_callback.__func__.core_element_id])
            if core_element_id in self._changed_id_to:
                self.tree_view_keypress_callback.__func__.core_element_id = self._changed_id_to[core_element_id]
            path = self.get_path_for_core_element(self.tree_view_keypress_callback.__func__.core_element_id)
            if event.keyval == Gdk.KEY_Tab:
                # logger.info("move right")
                direction = +1
            else:
                # logger.info("move left")
                direction = -1

            if len(path) > 1:
                n_elements_in_hierarchy = self.tree_store.iter_n_children(self.tree_store.get_iter(path[:-1]))
            else:
                n_elements_in_hierarchy = self.tree_store.iter_n_children(None)  # root

            if direction < 0 and focus_column is self.widget_columns[0] \
                    or direction > 0 and focus_column is self.widget_columns[-1]:
                if direction < 0 < path[-1] or direction > 0 and not path[-1] + 1 > n_elements_in_hierarchy:
                    next_row = path[-1] + direction
                else:
                    return False
            else:
                next_row = path[-1]
            new_path = tuple(list(path[:-1]) + [next_row])
            # print("new path", new_path, path[:-1])
            # get next column_id for focus
            focus_column_id = self.widget_columns.index(focus_column)
            if focus_column_id is not None:
                # search all columns for next editable cell renderer
                next_focus_column_id = 0
                for index in range(len(self.tree_view.get_model())):
                    test_id = focus_column_id + direction * index + direction
                    next_focus_column_id = test_id % len(self.widget_columns)
                    if test_id > len(self.widget_columns) - 1 or test_id < 0:
                        next_row = path[-1] + direction
                        if next_row < 0 or next_row > len(self.tree_store[path[:-1]] if len(path) > 1 else self.tree_store) - 1:
                            return False

                    if self.widget_columns[next_focus_column_id].get_cells()[0].get_property('editable'):
                        break
            else:
                return False
            new_path = tuple(list(path[:-1]) + [next_row])
            # print("nnew path", new_path)
            del self.tree_view_keypress_callback.__func__.core_element_id
            # self._logger.info("self.tree_view.scroll_to_cell(next_row={0}, self.widget_columns[{1}] , use_align={2})"
            #              "".format(next_row, next_focus_column_id, False))
            # self.tree_view.scroll_to_cell(new_path, self.widget_columns[next_focus_column_id], use_align=False)
            self.tree_view.set_cursor_on_cell(Gtk.TreePath.new_from_indices([new_path]), self.widget_columns[
                next_focus_column_id], focus_cell=None, start_editing=True)
            return True
        else:
            super(TreeViewController, self).tree_view_keypress_callback(widget, event)
