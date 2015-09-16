from gtk import TreeView
from gtk.keysyms import Tab as KEY_TAB, ISO_Left_Tab


class MoveAndEditWithTabKeyListFeatureController:
    """ The Controller introduce motion and edit functionality by using "tab"- or "shift-tab"-key for a gtk.TreeView.
     It is designed to work with a gtk.TreeView which model is a gtk.ListStore and only uses text cell renderer.
     Additional, the TreeView is assumed to be used as a list not as a tree.
     With the "tab"-key the cell on the right site of the actual focused cell is started to be edit. Changes in the
     gtk.Entry-Widget are confirmed by emitting a 'edited'-signal. If the row ends the edit process continues
     with the first cell of the next row. With the "shift-tab"-key the inverse functionality of the "tab"-key is provided.
     The Controller over steps not editable cells.
    """

    def __init__(self, tree_view):

        assert isinstance(tree_view, TreeView)
        self.view = tree_view

        self.last_entry_widget = None
        self.widget_columns = self.view.get_columns()

    def register_view(self):

        self.view.connect('key-press-event', self.tree_view_keypress_callback)

        for column in self.widget_columns:
            column.get_cell_renderers()[0].connect('editing-started', self.store_entry_widget)
            column.get_cell_renderers()[0].connect('editing-canceled', self.cancel_entry_widget)
            column.get_cell_renderers()[0].connect('edited', self.cancel_entry_widget)

    def store_entry_widget(self, cell_renderer, entry_widget, column_id):
        # logger.info("entry widget started")
        self.last_entry_widget = entry_widget

    def cancel_entry_widget(self, *args):
        # logger.info("entry widget canceled")
        self.last_entry_widget = None

    def tree_view_keypress_callback(self, widget, event):
        # logger.info("key_value: " + str(event.keyval))

        if event.keyval == KEY_TAB or event.keyval == ISO_Left_Tab:
            [path, focus_column] = self.view.get_cursor()
            # finish active edit process
            if self.last_entry_widget is not None and path:
                text = self.last_entry_widget.get_buffer().get_text()
                if focus_column in self.widget_columns:
                    focus_column.get_cell_renderers()[0].emit('edited', path[0], text)

            # rows could be updated by other call_backs caused by emitting 'edited' signal
            [path, focus_column] = self.view.get_cursor()
            if not path:
                return False

            if event.keyval == KEY_TAB:
                # logger.info("move right")
                direction = +1
            else:
                # logger.info("move left")
                direction = -1

            # get next row_id for focus
            if direction < 0 and focus_column is self.widget_columns[0] \
                    or direction > 0 and focus_column is self.widget_columns[-1]:
                if direction < 0 and path[0] > 0 or direction > 0 and not path[0] + 1 > len(self.view.get_model()):
                    next_row = path[0] + direction
                else:
                    return False
            else:
                next_row = path[0]

            # get next column_id for focus
            focus_column_id = self.widget_columns.index(focus_column)
            if focus_column_id is not None:
                # search all columns for next editable cell renderer
                for index in range(len(self.widget_columns)):
                    test_id = focus_column_id + direction*index + direction
                    next_focus_column_id = test_id % len(self.widget_columns)
                    if test_id > len(self.widget_columns) or test_id < 0:
                        next_row = path[0] + direction
                        if next_row < 0 or next_row > len(self.widget_columns):
                            return False

                    if self.widget_columns[next_focus_column_id].get_cell_renderers()[0].get_property('editable'):
                        break
            else:
                return False

            self.view.set_cursor(next_row, self.widget_columns[next_focus_column_id], start_editing=True)
            return True
