import gtk
from gtk.gdk import CONTROL_MASK, SHIFT_MASK
from rafcon.utils import log

module_logger = log.get_logger(__name__)


class ListSelectionFeatureController(object):
    list_store = None
    tree_view = None
    _logger = None
    ID_STORAGE_ID = None
    MODEL_STORAGE_ID = None

    def __init__(self, model, view, logger=None):
        assert isinstance(model, gtk.ListStore)
        assert isinstance(view, gtk.TreeView)
        assert self.list_store is model
        assert self.tree_view is view
        self._logger = logger if logger is not None else module_logger
        self._do_selection_update = False
        self._tree_selection = self.tree_view.get_selection()
        self._last_path_selection = None

    def register_view(self, view):
        self.tree_view.connect('button_press_event', self.mouse_click)
        self._tree_selection.connect('changed', self.selection_changed)
        self._tree_selection.set_mode(gtk.SELECTION_MULTIPLE)
        self.update_selection_sm_prior()

    def on_right_click_menu(self):
        raise NotImplementedError

    def get_view_selection(self):
        model, paths = self._tree_selection.get_selected_rows()
        selected_model_list = []
        for path in paths:
            model = self.list_store[path][self.MODEL_STORAGE_ID]
            selected_model_list.append(model)
        return self._tree_selection, selected_model_list

    def get_state_machine_selection(self):
        self._logger.info(self.__class__.__name__)
        raise NotImplementedError

    def get_selections(self):
        sm_selection, sm_selected_model_list = self.get_state_machine_selection()
        tree_selection, selected_model_list = self.get_view_selection()
        return tree_selection, selected_model_list, sm_selection, sm_selected_model_list

    def mouse_click(self, widget, event=None):

        # selection = self.tree_selection
        # print selection.get_mode(), bool(event.state & SHIFT_MASK), bool(event.state & CONTROL_MASK), type(event)

        if event.type == gtk.gdk.BUTTON_PRESS:
            pthinfo = self.tree_view.get_path_at_pos(int(event.x), int(event.y))

            if not bool(event.state & CONTROL_MASK) and not bool(event.state & SHIFT_MASK) and \
                    event.type == gtk.gdk.BUTTON_PRESS and event.button == 3:
                pthinfo = self.tree_view.get_path_at_pos(int(event.x), int(event.y))
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
                pthinfo = self.tree_view.get_path_at_pos(int(event.x), int(event.y))
                model, paths = self._tree_selection.get_selected_rows()
                # print model, paths, pthinfo[0]
                if paths and pthinfo[0]:
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
                    if pthinfo[0]:
                        self._last_path_selection = pthinfo[0]

            if bool(event.state & CONTROL_MASK) and event.button == 1:
                # self._logger.info("CONTROL adjust selection range")
                pthinfo = self.tree_view.get_path_at_pos(int(event.x), int(event.y))
                model, paths = self._tree_selection.get_selected_rows()
                # print model, paths, pthinfo[0]
                if paths and pthinfo[0]:
                    if pthinfo[0] in paths:
                        self._tree_selection.unselect_path(pthinfo[0])
                    else:
                        self._tree_selection.select_path(pthinfo[0])
                    return True
                elif pthinfo[0]:
                    self._tree_selection.select_path(pthinfo[0])
                    return True

    def update_selection_sm_prior(self):
        if self._do_selection_update:
            return
        # print "update selection sm prior", self.tree_selection.get_mode()
        self._do_selection_update = True
        tree_selection, selected_model_list, sm_selection, sm_selected_model_list = self.get_selections()
        # print sm_selected_model_list, "STATEMACHINE:\n", '\n'.join([str(model) for model in  sm_selected_model_list])
        # print selected_model_list, "WIDGET:\n", '\n'.join([str(model) for model in  selected_model_list])
        for path, row in enumerate(self.list_store):
            model = row[self.MODEL_STORAGE_ID]
            # print "condition: ", model, model in sm_selected_model_list, model in selected_model_list
            if model not in sm_selected_model_list and model in selected_model_list:
                # print type(self).__name__, "sm un-select model", model
                tree_selection.unselect_path(path)
            if model in sm_selected_model_list and model not in selected_model_list:
                # print type(self).__name__, "sm select model", model
                tree_selection.select_path(path)

        #     else:
        #         print "else"
        #         if model in sm_selected_model_list and model in selected_model_list:
        #             print type(self).__name__, "sm is selected"
        # tree_selection, selected_model_list, sm_selection, sm_selected_model_list = self.get_selections()
        # print selected_model_list, sm_selected_model_list
        self._do_selection_update = False

    def update_selection_self_prior(self):
        if self._do_selection_update:
            return
        # print "update selection self prior"
        self._do_selection_update = True
        tree_selection, selected_model_list, sm_selection, sm_selected_model_list = self.get_selections()
        # print sm_selected_model_list, "STATEMACHINE:\n", '\n'.join([str(model) for model in  sm_selected_model_list])
        # print selected_model_list, "WIDGET:\n", '\n'.join([str(model) for model in  selected_model_list])
        for row in self.list_store:
            model = row[self.MODEL_STORAGE_ID]
            # print "condition: ", model, model in sm_selected_model_list, model in selected_model_list
            if model in sm_selected_model_list and model not in selected_model_list:
                # print type(self).__name__, "unselect model", model
                sm_selection.remove(model)
            if model not in sm_selected_model_list and model in selected_model_list:
                # print type(self).__name__, "select model", model
                sm_selection.add(model)

            # else:
            #     if model in sm_selected_model_list and model in selected_model_list:
            #         print type(self).__name__, "is selected"
        # tree_selection, selected_model_list, sm_selection, sm_selected_model_list = self.get_selections()
        # print selected_model_list, sm_selected_model_list
        self._do_selection_update = False

    def selection_changed(self, widget, event=None):
        # print type(self).__name__, "select changed", widget, event, self
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

    def get_list_store_row_from_cursor_selection(self):
        """Returns the list_store_row of the currently by cursor selected row entry"""
        path = self.get_path()
        if path is not None:
            list_store_row = self.list_store[int(path)]
            return list_store_row
        return None

    def get_path(self):
        """Returns the path/index to the currently selected row entry"""
        cursor = self.tree_view.get_cursor()
        # the cursor is a tuple containing the current path and the focused column
        if cursor[0] is None:
            return None
        return cursor[0][0]


class TreeSelectionFeatureController(object):
    tree_store = None
    tree_view = None
    _logger = None
    ID_STORAGE_ID = None
    MODEL_STORAGE_ID = None

    def __init__(self, model, view, logger=None):
        assert isinstance(model, gtk.TreeStore)
        assert isinstance(view, gtk.TreeView)
        assert self.tree_store is model
        assert self.tree_view is view
        self._logger = logger if logger is not None else module_logger
        self._do_selection_update = False
        self._tree_selection = self.tree_view.get_selection()
        self._last_path_selection = None

    def register_view(self, view):
        # self.tree_view.connect('button_press_event', self.mouse_click)
        self._tree_selection.connect('changed', self.selection_changed)
        self._tree_selection.set_mode(gtk.SELECTION_MULTIPLE)
        self.update_selection_sm_prior()

    def on_right_click_menu(self):
        raise NotImplementedError

    def get_view_selection(self):
        model, paths = self._tree_selection.get_selected_rows()
        selected_model_list = []
        for path in paths:
            model = self.tree_store[path][self.MODEL_STORAGE_ID]
            selected_model_list.append(model)
        return self._tree_selection, selected_model_list

    def get_state_machine_selection(self):
        self._logger.info(self.__class__.__name__)
        raise NotImplementedError

    def get_selections(self):
        sm_selection, sm_selected_model_list = self.get_state_machine_selection()
        tree_selection, selected_model_list = self.get_view_selection()
        return tree_selection, selected_model_list, sm_selection, sm_selected_model_list

    def iter_tree_with_handed_function(self, function, *function_args):

        def iter_all_children(state_row_iter, function, function_args):
            function(state_row_iter, *function_args)

            if isinstance(state_row_iter, gtk.TreeIter):
                for n in reversed(range(self.tree_store.iter_n_children(state_row_iter))):
                    child_iter = self.tree_store.iter_nth_child(state_row_iter, n)
                    iter_all_children(child_iter, function, function_args)
            else:
                self._logger.warning("Iter has to be TreeIter")

        iter_all_children(self.tree_store.get_iter_root(), function, function_args)

    def update_selection_sm_prior_condition(self, state_row_iter, selected_model_list, sm_selected_model_list):
        selected_path = self.tree_store.get_path(state_row_iter)
        tree_model_row = self.tree_store[selected_path]
        model = tree_model_row[self.MODEL_STORAGE_ID]
        # self._logger.info("check state {1} {2} {0}".format([model],
        #                                                    model in sm_selected_model_list,
        #                                                    model in selected_model_list))

        if model not in sm_selected_model_list and model in selected_model_list:
            # print type(self).__name__, "sm un-select model", model
            self._tree_selection.unselect_iter(state_row_iter)
        elif model in sm_selected_model_list and model not in selected_model_list:
            # print type(self).__name__, "sm select model", model
            self.tree_view.expand_to_path(selected_path)
            self._tree_selection.select_iter(state_row_iter)

    def update_selection_self_prior_condition(self, state_row_iter, sm_selection, selected_model_list, sm_selected_model_list):
        selected_path = self.tree_store.get_path(state_row_iter)
        tree_model_row = self.tree_store[selected_path]
        model = tree_model_row[self.MODEL_STORAGE_ID]
        # self._logger.info("check state {1} {2} {0}".format([model],
        #                                                    model in sm_selected_model_list,
        #                                                    model in selected_model_list))

        if model in sm_selected_model_list and model not in selected_model_list:
            # print type(self).__name__, "unselect model", model
            sm_selection.remove(model)
        elif model not in sm_selected_model_list and model in selected_model_list:
            # print type(self).__name__, "select model", model
            sm_selection.add(model)

    def update_selection_self_prior(self):
        if self._do_selection_update:
            return
        tree_selection, selected_model_list, sm_selection, sm_selected_model_list = self.get_selections()
        if sm_selection is None:
            return

        # self._logger.info("SELF SELECTION IS: {2}\nSELF {0}, \nSM   {1}".format(selected_model_list, sm_selected_model_list,
        #                                                                         tree_selection.get_mode()))
        self._do_selection_update = True
        self.iter_tree_with_handed_function(self.update_selection_self_prior_condition,
                                            sm_selection, selected_model_list, sm_selected_model_list)
        # tree_selection, selected_model_list, sm_selection, sm_selected_model_list = self.get_selections()
        # print selected_model_list, sm_selected_model_list

        self._do_selection_update = False

    def update_selection_sm_prior(self):
        if self._do_selection_update:
            return
        tree_selection, selected_model_list, sm_selection, sm_selected_model_list = self.get_selections()
        if sm_selection is None:
            return

        # self._logger.info("SM SELECTION IS: {2}\n{0}, \n{1}".format(selected_model_list, sm_selected_model_list,
        #                                                             tree_selection.get_mode()))
        self._do_selection_update = True
        self.iter_tree_with_handed_function(self.update_selection_sm_prior_condition,
                                            selected_model_list, sm_selected_model_list)
        # tree_selection, selected_model_list, sm_selection, sm_selected_model_list = self.get_selections()
        # print selected_model_list, sm_selected_model_list
        self._do_selection_update = False

    def selection_changed(self, widget, event=None):
        # print type(self).__name__, "select changed", widget, event, self
        self.update_selection_self_prior()
