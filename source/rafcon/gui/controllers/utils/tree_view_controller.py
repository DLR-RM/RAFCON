# Copyright (C) 2016-2017 DLR
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

import glib
import gtk
from gtk.gdk import CONTROL_MASK, SHIFT_MASK
from gtk.keysyms import Tab as Key_Tab, ISO_Left_Tab

from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.helpers.label import react_to_event, is_event_of_key_string
from rafcon.gui.models.selection import Selection, reduce_to_parent_states
from rafcon.utils import log

module_logger = log.get_logger(__name__)


class ListViewController(ExtendedController):
    """Base class for controller having a gtk.Tree view with a gtk.ListStore

    The class implements methods for e.g. handling (multi-)selection and offers default callback methods for various
    signals and includes a move and edit by tab-key feature.

    :ivar gtk.ListStore list_store: List store that set by inherit class
    :ivar gtk.TreeView tree_view: Tree view that set by inherit class
    :ivar int ID_STORAGE_ID: Index of core element id represented by row in list store and
        used to select entries set by inherit class
    :ivar int MODEL_STORAGE_ID: Index of model represented by row in list store and
        used to update selections in state machine or tree view set by inherit class
    """
    ID_STORAGE_ID = None
    CORE_STORAGE_ID = None
    MODEL_STORAGE_ID = None
    _logger = None
    CORE_ELEMENT_CLASS = None

    # TODO refactor to move functionality to gui.utils.tree.TreeView and possible reuse in TreeViewController

    def __init__(self, model, view, tree_view, list_store, logger=None):
        super(ListViewController, self).__init__(model, view)
        self._logger = logger if logger is not None else module_logger
        self._do_selection_update = False
        self._last_path_selection = None
        self._setup_tree_view(tree_view, list_store)
        self.active_entry_widget = None
        self.widget_columns = self.tree_view.get_columns()

    def register_view(self, view):
        """Register callbacks for button press events and selection changed"""
        self.tree_view.connect('button_press_event', self.mouse_click)
        self.tree_view.connect('key-press-event', self.tree_view_keypress_callback)
        self._tree_selection.connect('changed', self.selection_changed)
        self._tree_selection.set_mode(gtk.SELECTION_MULTIPLE)
        self.update_selection_sm_prior()

    def _setup_tree_view(self, tree_view, list_store):
        self.tree_view = tree_view
        self.tree_view.set_model(list_store)
        self.list_store = list_store
        self._tree_selection = self.tree_view.get_selection()

    def _apply_value_on_edited_and_focus_out(self, renderer, apply_method):
        """Sets up the renderer to apply changed when loosing focus

        The default behaviour for the focus out event dismisses the changes in the renderer. Therefore we setup
        handlers for that event, applying the changes.

        :param gtk.CellRendererText renderer: The cell renderer who's changes are to be applied on focus out events
        :param apply_method: The callback method applying the newly entered value
        """
        assert isinstance(renderer, gtk.CellRenderer)

        def remove_handler(widget, data_name):
            """Remove handler from given widget

            :param gtk.Widget widget: Widget from which a handler is to be removed
            :param data_name: Name of the data of the widget in which the handler id is stored
            """
            handler_id = widget.get_data(data_name)
            if widget.handler_is_connected(handler_id):
                widget.disconnect(handler_id)

        def remove_all_handler(renderer):
            """Remove all handler for given renderer and its editable

            :param renderer: Renderer of the respective column which is edit by a entry widget, at the moment
            """
            editable = renderer.get_data("editable")
            remove_handler(editable, "focus_out_handler_id")
            remove_handler(editable, "cursor_move_handler_id")
            remove_handler(renderer, "editing_cancelled_handler_id")

        def on_editing_canceled(renderer):
            """Disconnects the focus-out-event handler of cancelled editable

            :param gtk.CellRendererText renderer: The cell renderer who's editing was cancelled
            """
            remove_all_handler(renderer)
            self.active_entry_widget = None

        def on_focus_out(entry, event):
            """Applies the changes to the entry

            :param gtk.Entry entry: The entry that was focused out
            :param gtk.Event event: Event object with information about the event
            """
            remove_all_handler(renderer)

            if self.get_path() is None:
                return
            # We have to use idle_add to prevent core dumps:
            # https://mail.gnome.org/archives/gtk-perl-list/2005-September/msg00143.html
            glib.idle_add(apply_method, self.get_path(), entry.get_text())

        def on_cursor_move_in_entry_widget(entry, step, count, extend_selection):
            """Trigger scroll bar adjustments according active entry widgets cursor change
            """
            self.tree_view_keypress_callback(entry, None)

        def on_editing_started(renderer, editable, path):
            """Connects the a handler for the focus-out-event of the current editable

            :param gtk.CellRendererText renderer: The cell renderer who's editing was started
            :param gtk.CellEditable editable: interface for editing the current TreeView cell
            :param str path: the path identifying the edited cell
            """
            # secure scrollbar adjustments on active cell
            [path, focus_column] = self.tree_view.get_cursor()
            self.tree_view.scroll_to_cell(path, self.widget_columns[self.widget_columns.index(focus_column)], use_align=False)

            editing_cancelled_handler_id = renderer.connect('editing-canceled', on_editing_canceled)
            focus_out_handler_id = editable.connect('focus-out-event', on_focus_out)
            cursor_move_handler_id = editable.connect('move-cursor', on_cursor_move_in_entry_widget)
            # Store reference to editable and signal handler ids for later access when removing the handlers
            renderer.set_data("editable", editable)
            renderer.set_data("editing_cancelled_handler_id", editing_cancelled_handler_id)
            editable.set_data("focus_out_handler_id", focus_out_handler_id)
            editable.set_data("cursor_move_handler_id", cursor_move_handler_id)
            self.active_entry_widget = editable

        def on_edited(renderer, path, new_value_str):
            """Calls the apply method with the new value

            :param gtk.CellRendererText renderer: The cell renderer that was edited
            :param str path: The path string of the renderer
            :param str new_value_str: The new value as string
            """
            remove_all_handler(renderer)
            apply_method(path, new_value_str)
            self.active_entry_widget = None

        renderer.connect('editing-started', on_editing_started)
        renderer.connect('edited', on_edited)

    def copy_action_callback(self, *event):
        """Callback method for copy action"""
        if react_to_event(self.view, self.tree_view, event) and self.active_entry_widget is None:
            sm_selection, sm_selected_model_list = self.get_state_machine_selection()
            # only list specific elements are copied by widget
            if sm_selection is not None:
                sm_selection.set(sm_selected_model_list)
                global_clipboard.copy(sm_selection)
                return True

    def cut_action_callback(self, *event):
        """Callback method for copy action"""
        if react_to_event(self.view, self.tree_view, event) and self.active_entry_widget is None:
            sm_selection, sm_selected_model_list = self.get_state_machine_selection()
            # only list specific elements are cut by widget
            if sm_selection is not None:
                sm_selection.set(sm_selected_model_list)
                global_clipboard.cut(sm_selection)
                return True

    def add_action_callback(self, *event):
        """Callback method for add action"""
        if react_to_event(self.view, self.tree_view, event) and self.active_entry_widget is None:
            self.on_add(None)
            return True

    def remove_action_callback(self, *event):
        """Callback method for remove action

        The method checks whether a shortcut ('Delete') is in the gui config model which shadow the delete functionality
        of maybe active a entry widget. If a entry widget is active the remove callback return with None.
        """
        if react_to_event(self.view, self.tree_view, event) and \
                not (self.active_entry_widget and not is_event_of_key_string(event, 'Delete')):
            self.on_remove(None)
            return True

    def on_add(self, widget, data=None):
        """An abstract add method for a respective new core element and final selection of those"""
        raise NotImplementedError

    def remove_core_element(self, model):
        """An abstract remove method that removes respective core element by handed core element id

        The method has to be implemented by inherit classes

        :param StateElementModel model: Model which core element should be removed
        :return:
        """
        raise NotImplementedError

    def on_remove(self, widget, data=None):
        """Remove respective selected core elements and select the next one"""
        import rafcon.gui.helpers.state_machine as gui_helper_state_machine
        path_list = None
        if self.view is not None:
            model, path_list = self.tree_view.get_selection().get_selected_rows()
        old_path = self.get_path()
        models = [self.list_store[path][self.MODEL_STORAGE_ID] for path in path_list] if path_list else []
        if models:
            try:
                gui_helper_state_machine.delete_core_elements_of_models(models)
            except AttributeError as e:
                self._logger.warn("The respective core element of {1}.list_store couldn't be removed. -> {0}"
                                  "".format(e, self.__class__.__name__))
            if len(self.list_store) > 0:
                self.tree_view.set_cursor(min(old_path[0], len(self.list_store) - 1))
            return True
        else:
            self._logger.warning("Please select a element to be removed.")

    def on_right_click_menu(self):
        """An abstract method called after right click events"""
        raise NotImplementedError

    def get_view_selection(self):
        """Get actual tree selection object and all respective models of selected rows"""
        if not self.MODEL_STORAGE_ID:
            return None, None
        model, paths = self._tree_selection.get_selected_rows()
        selected_model_list = []
        for path in paths:
            model = self.list_store[path][self.MODEL_STORAGE_ID]
            selected_model_list.append(model)
        return self._tree_selection, selected_model_list

    def get_state_machine_selection(self):
        """An abstract getter method for state machine selection

        The method maybe has to be re-implemented by inherit classes and hands generally a filtered set of
        selected elements.

        :return: selection object it self, filtered set of selected elements
        :rtype: rafcon.gui.selection.Selection, set
        """
        sm_selection = self.model.get_state_machine_m().selection if self.model.get_state_machine_m() else None
        return sm_selection, sm_selection.get_selected_elements_of_core_class(self.CORE_ELEMENT_CLASS) if sm_selection else set()

    def get_selections(self):
        """Get current model selection status in state machine selection (filtered according the purpose of the widget)
        and tree selection of the widget"""
        sm_selection, sm_filtered_selected_model_set = self.get_state_machine_selection()
        tree_selection, selected_model_list = self.get_view_selection()
        return tree_selection, selected_model_list, sm_selection, sm_filtered_selected_model_set

    def mouse_click(self, widget, event=None):
        """Implements shift- and control-key handling features for mouse button press events explicit

         The method is implements a fully defined mouse pattern to use shift- and control-key for multi-selection in a
         TreeView and a ListStore as model. It avoid problems caused by special renderer types like the text combo
         renderer by stopping the callback handler to continue with notifications.

        :param gtk.Object widget: Object which is the source of the event
        :param gtk.Event event: Event generated by mouse click
        :rtype: bool
        """

        if event.type == gtk.gdk.BUTTON_PRESS:
            pthinfo = self.tree_view.get_path_at_pos(int(event.x), int(event.y))

            if not bool(event.state & CONTROL_MASK) and not bool(event.state & SHIFT_MASK) and \
                    event.type == gtk.gdk.BUTTON_PRESS and event.button == 3:
                if pthinfo is not None:
                    model, paths = self._tree_selection.get_selected_rows()
                    # print paths
                    if pthinfo[0] not in paths:
                        # self._logger.info("force single selection for right click")
                        self.tree_view.set_cursor(pthinfo[0])
                        self._last_path_selection = pthinfo[0]
                    else:
                        # self._logger.info("single- or multi-selection for right click")
                        pass
                    self.on_right_click_menu()
                    return True

            if (bool(event.state & CONTROL_MASK) or bool(event.state & SHIFT_MASK)) and \
                    event.type == gtk.gdk.BUTTON_PRESS and event.button == 3:
                return True

            if not bool(event.state & SHIFT_MASK) and event.button == 1:
                if pthinfo is not None:
                    # self._logger.info("last select row {}".format(pthinfo[0]))
                    self._last_path_selection = pthinfo[0]
                # else:
                #     self._logger.info("deselect rows")
                #     self.tree_selection.unselect_all()

            if bool(event.state & SHIFT_MASK) and event.button == 1:
                # self._logger.info("SHIFT adjust selection range")
                model, paths = self._tree_selection.get_selected_rows()
                # print model, paths, pthinfo[0]
                if paths and pthinfo and pthinfo[0]:
                    if self._last_path_selection[0] <= pthinfo[0][0]:
                        new_row_ids_selected = range(self._last_path_selection[0], pthinfo[0][0]+1)
                    else:
                        new_row_ids_selected = range(self._last_path_selection[0], pthinfo[0][0]-1, -1)
                    # self._logger.info("range to select {0}, {1}".format(new_row_ids_selected, model))
                    self._tree_selection.unselect_all()
                    for path in new_row_ids_selected:
                        self._tree_selection.select_path(path)
                    return True
                else:
                    # self._logger.info("nothing selected {}".format(model))
                    if pthinfo and pthinfo[0]:
                        self._last_path_selection = pthinfo[0]

            if bool(event.state & CONTROL_MASK) and event.button == 1:
                # self._logger.info("CONTROL adjust selection range")
                model, paths = self._tree_selection.get_selected_rows()
                # print model, paths, pthinfo[0]
                if paths and pthinfo and pthinfo[0]:
                    if pthinfo[0] in paths:
                        self._tree_selection.unselect_path(pthinfo[0])
                    else:
                        self._tree_selection.select_path(pthinfo[0])
                    return True
                elif pthinfo and pthinfo[0]:
                    self._tree_selection.select_path(pthinfo[0])
                    return True

        # Double click with left mouse button focuses the element
        elif event.type == gtk.gdk._2BUTTON_PRESS and event.button == 1:
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
                    tree_selection.unselect_path(path)
                if model in sm_selected_model_list and model not in selected_model_list:
                    tree_selection.select_path(path)

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

    @ExtendedController.observe("sm_selection_changed_signal", signal=True)
    def state_machine_selection_changed(self, state_machine_m, signal_name, signal_msg):
        if self.CORE_ELEMENT_CLASS in signal_msg.arg.affected_core_element_classes:
            self.update_selection_sm_prior()

    def selection_changed(self, widget, event=None):
        """Notify tree view about state machine selection"""
        # print type(self).__name__, self._do_selection_update, "select changed", widget, event, self
        self.update_selection_self_prior()

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
        :rtype: gtk.TreeModelRow
        """
        path = self.get_path()
        if path is not None:
            return self.list_store[path]

    def get_path(self):
        """Get path to the currently selected entry row

        :return: path to the tree view cursor row, None if there is no selection
        :rtype: tuple
        """
        # the cursor is a tuple containing the current path and the focused column
        return self.tree_view.get_cursor()[0]

    def tree_view_keypress_callback(self, widget, event):
        """Tab back and forward tab-key motion in list widget

         The method introduce motion and edit functionality by using "tab"- or "shift-tab"-key for a gtk.TreeView.
         It is designed to work with a gtk.TreeView which model is a gtk.ListStore and only uses text cell renderer.
         Additional, the TreeView is assumed to be used as a list not as a tree.
         With the "tab"-key the cell on the right site of the actual focused cell is started to be edit. Changes in the
         gtk.Entry-Widget are confirmed by emitting a 'edited'-signal. If the row ends the edit process continues
         with the first cell of the next row. With the "shift-tab"-key the inverse functionality of the "tab"-key is
         provided.
         The Controller over steps not editable cells.

        :param gtk.TreeView widget: The tree view the controller use
        :param gtk.gdk.Event event: The key press event
        :return:
        """
        # self._logger.info("key_value: " + str(event.keyval))

        if event and (event.keyval == Key_Tab or event.keyval == ISO_Left_Tab):
            [path, focus_column] = self.tree_view.get_cursor()
            if not path:
                return False
            self.tree_view_keypress_callback.__func__.core_element_id = self.list_store[path][self.ID_STORAGE_ID]

            # finish active edit process
            if self.active_entry_widget is not None:
                text = self.active_entry_widget.get_buffer().get_text()
                if focus_column in self.widget_columns:
                    focus_column.get_cell_renderers()[0].emit('edited', path[0], text)

            # row could be updated by other call_backs caused by emitting 'edited' signal but selection stays an editable neighbor
            path = self.get_path_for_core_element(self.tree_view_keypress_callback.__func__.core_element_id)
            if event.keyval == Key_Tab:
                # logger.info("move right")
                direction = +1
            else:
                # logger.info("move left")
                direction = -1

            # get next row_id for focus
            if direction < 0 and focus_column is self.widget_columns[0] \
                    or direction > 0 and focus_column is self.widget_columns[-1]:
                if direction < 0 < path[0] or direction > 0 and not path[0] + 1 > len(self.widget_columns):
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

                    if self.widget_columns[next_focus_column_id].get_cell_renderers()[0].get_property('editable'):
                        break
            else:
                return False

            del self.tree_view_keypress_callback.__func__.core_element_id
            # self._logger.info("self.tree_view.scroll_to_cell(next_row={0}, self.widget_columns[{1}] , use_align={2})"
            #              "".format(next_row, next_focus_column_id, False))
            self.tree_view.scroll_to_cell(next_row, self.widget_columns[next_focus_column_id], use_align=False)
            self.tree_view.set_cursor_on_cell(next_row, self.widget_columns[next_focus_column_id], start_editing=True)
            return True
        else:
            current_row_path, current_focused_column = self.tree_view.get_cursor()
            # print current_row_path, current_focused_column
            if isinstance(widget, gtk.TreeView):
                if current_row_path is not None and len(current_row_path) == 1 and isinstance(current_row_path[0], int):
                    self.tree_view.scroll_to_cell(current_row_path[0], current_focused_column, use_align=False)
                # else:
                #     self._logger.debug("A ListViewController aspects a current_row_path of dimension 1 with integer but"
                #                        " it is {0} and column is {1}".format(current_row_path, current_focused_column))
            elif isinstance(widget, gtk.Entry) and self.view.scrollbar_widget is not None:
                # calculate the position of the scrollbar to be always centered with the entry widget cursor
                # TODO check how to get sufficient the scroll-offset in the entry widget -> some times zero when not
                # TODO the scrollbar is one step behind cursor -> so jump from pos1 to end works not perfect
                entry_widget_scroll_offset, entry_widget_cursor_position, entry_widget_text_length = \
                    widget.get_properties(*["scroll-offset", "cursor-position", "text-length"])
                cell_rect_of_entry_widget = widget.get_allocation()

                horizontal_scroll_bar = self.view.scrollbar_widget.get_hscrollbar()
                if horizontal_scroll_bar is not None:
                    adjustment = horizontal_scroll_bar.get_adjustment()
                    layout_pixel_width = widget.get_layout().get_pixel_size()[0]
                    # print "rel_pos pices", cell_rect_of_entry_widget.x,
                    #     int(layout_pixel_width*float(entry_widget_cursor_position)/float(entry_widget_text_length))
                    rel_pos = cell_rect_of_entry_widget.x - entry_widget_scroll_offset + \
                        int(layout_pixel_width*float(entry_widget_cursor_position)/float(entry_widget_text_length))
                    # print adjustment.lower, adjustment.upper, adjustment.value, adjustment.page_size, rel_pos
                    value = int(float(adjustment.upper - adjustment.page_size)*rel_pos/float(adjustment.upper))
                    adjustment.set_value(value)
                    # print "new value", adjustment.value, 'of', float(adjustment.upper - adjustment.page_size)


class TreeViewController(ExtendedController):
    """Base class for controller having a gtk.Tree view with a gtk.TreeStore

    The class implements methods for e.g. handling (multi-)selection.

    :ivar gtk.TreeStore tree_store: Tree store that set by inherit class
    :ivar gtk.TreeView tree_view: Tree view that set by inherit class
    :ivar int ID_STORAGE_ID: Index of core element id represented by row in list store and
        used to select entries set by inherit class
    :ivar int MODEL_STORAGE_ID: Index of model represented by row in list store and
        used to update selections in state machine or tree view set by inherit class
    """
    ID_STORAGE_ID = None
    MODEL_STORAGE_ID = None
    CORE_ELEMENT_CLASS = None
    _logger = None

    def __init__(self, model, view, tree_view, tree_store, logger=None):
        super(TreeViewController, self).__init__(model, view)
        self._logger = logger if logger is not None else module_logger
        self._do_selection_update = False
        self._last_path_selection = None
        self._setup_tree_view(tree_view, tree_store)

    def register_view(self, view):
        """Register callbacks for button press events and selection changed"""
        # self.tree_view.connect('button_press_event', self.mouse_click)
        self._tree_selection.connect('changed', self.selection_changed)
        self._tree_selection.set_mode(gtk.SELECTION_MULTIPLE)
        self.update_selection_sm_prior()

    def copy_action_callback(self, *event):
        """Callback method for copy action"""
        if react_to_event(self.view, self.tree_view, event):
            sm_selection, sm_selected_model_list = self.get_state_machine_selection()
            # only list specific elements are copied by widget
            if sm_selection is not None:
                sm_selection.set(sm_selected_model_list)
                global_clipboard.copy(sm_selection)
                return True

    def cut_action_callback(self, *event):
        """Callback method for copy action"""
        if react_to_event(self.view, self.tree_view, event):
            sm_selection, sm_selected_model_list = self.get_state_machine_selection()
            # only list specific elements are cut by widget
            if sm_selection is not None:
                sm_selection.set(sm_selected_model_list)
                global_clipboard.cut(sm_selection)
                return True

    def _setup_tree_view(self, tree_view, tree_store):
        self.tree_view = tree_view
        self.tree_view.set_model(tree_store)
        self.tree_store = tree_store
        self._tree_selection = self.tree_view.get_selection()

    def get_view_selection(self):
        """Get actual tree selection object and all respective models of selected rows"""
        model, paths = self._tree_selection.get_selected_rows()
        selected_model_list = []
        for path in paths:
            model = self.tree_store[path][self.MODEL_STORAGE_ID]
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

    def iter_tree_with_handed_function(self, function, *function_args):
        """Iterate tree view with condition check function"""
        def iter_all_children(state_row_iter, function, function_args):

            if isinstance(state_row_iter, gtk.TreeIter):
                function(state_row_iter, *function_args)
                for n in reversed(range(self.tree_store.iter_n_children(state_row_iter))):
                    child_iter = self.tree_store.iter_nth_child(state_row_iter, n)
                    iter_all_children(child_iter, function, function_args)
            else:
                self._logger.warning("Iter has to be TreeIter -> handed argument is: {0}".format(state_row_iter))

        if self.tree_store.get_iter_root():
            iter_all_children(self.tree_store.get_iter_root(), function, function_args)

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
            # TODO check if we can solve the difference that occurs e.g. while complex actions
            # -> models in selection for core element not in the tree the function iter tree + condition tolerates this
            if not set(selected_model_list) == sm_selected_model_set:
                self._logger.debug("Difference between tree view selection: \n{0} \nand state machine selection: \n{1}"
                                   "".format(selected_model_list, sm_selected_model_set))

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

    def selection_changed(self, widget, event=None):
        """Notify tree view about state machine selection"""
        # print type(self).__name__, "select changed", widget, event, self
        self.update_selection_self_prior()
