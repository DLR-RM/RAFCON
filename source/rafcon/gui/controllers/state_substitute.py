# Copyright

"""
.. module:: state_substitute
   :synopsis: A module that holds the controller with the interface for the user to choose a library state
   that is used to substitute another state.

"""

import gtk

from rafcon.gui.views.library_tree import LibraryTreeView
from rafcon.gui.controllers.library_tree import LibraryTreeController
from rafcon.gui.utils.dialog import RAFCONButtonDialog

from rafcon.utils import log

logger = log.get_logger(__name__)


class StateSubstituteChooseLibraryDialogTreeController(LibraryTreeController):

    def __init__(self, model=None, view=None, state_machine_manager_model=None, dialog_widget=None):
        super(StateSubstituteChooseLibraryDialogTreeController, self).__init__(model, view, state_machine_manager_model)
        self.dialog_widget = dialog_widget

    def generate_right_click_menu(self, *args, **kwargs):
        pass

    def mouse_click(self, widget, event=None):
        # Double click with left mouse button
        if event.type == gtk.gdk._2BUTTON_PRESS and event.button == 1:
            (model, row) = self.view.get_selection().get_selected()
            if isinstance(model[row][1], dict):  # double click on folder, not library
                state_row_path = self.tree_store.get_path(row)
                if state_row_path is not None:
                    if self.view.row_expanded(state_row_path):
                        self.view.collapse_row(state_row_path)
                    else:
                        self.view.expand_to_path(state_row_path)
                return False
            self.substitute_as_library_clicked(None)
            if self.dialog_widget:
                self.dialog_widget.destroy()
            return True
        if event.type == gtk.gdk._2BUTTON_PRESS and event.button == 3:
            (model, row) = self.view.get_selection().get_selected()
            if isinstance(model[row][1], dict):  # double click on folder, not library
                return False
            self.substitute_as_template_clicked(None)
            if self.dialog_widget:
                self.dialog_widget.destroy()
            return True


class StateSubstituteChooseLibraryDialog(RAFCONButtonDialog):

    def __init__(self, model, width=500, height=500, pos=None, parent=None):
        self.model = model

        super(StateSubstituteChooseLibraryDialog, self).__init__("Choose a Library to substitute the state with.",
                                                                 ['As library', 'As template', 'Cancel'],
                                                                 callback=self.check_for_library_path,
                                                                 message_type=gtk.MESSAGE_INFO,
                                                                 flags=gtk.DIALOG_MODAL, parent=parent)

        self.set_title('Library choose dialog')
        self.resize(width=width, height=height)
        if pos is not None:
            self.set_position(pos)

        self.widget_view = LibraryTreeView()
        self.widget_ctrl = StateSubstituteChooseLibraryDialogTreeController(self.model, self.widget_view,
                                                                            dialog_widget=self)

        self.vbox.pack_start(self.widget_view, True, True, 0)

        self.vbox.show_all()
        self.grab_focus()
        self.run()

    def destroy(self):
        self.widget_view.destroy()
        self.widget_ctrl.destroy()
        super(StateSubstituteChooseLibraryDialog, self).destroy()

    def check_for_library_path(self, widget, response_id):
        if response_id == 1:
            logger.debug("Library substitute state as library triggered.")
            self.widget_ctrl.substitute_as_library_clicked(None)
        elif response_id == 2:
            logger.debug("Library substitute state as template triggered.")
            self.widget_ctrl.substitute_as_template_clicked(None)
        elif response_id in [3, -4]:
            pass
        else:
            logger.warning("Response id: {} is not considered".format(response_id))
            return
        self.destroy()

