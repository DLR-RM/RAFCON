import gtk

from rafcon.mvc.views.library_tree import LibraryTreeView
from rafcon.mvc.controllers.library_tree import LibraryTreeController
from rafcon.mvc.utils.dialog import RAFCONDialog, ButtonDialog

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
                state_row_path = self.library_tree_store.get_path(row)
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


class StateSubstituteChooseLibraryDialog(RAFCONDialog):

    def __init__(self, model, width=500, height=500, pos=None, parent=None):
        self.model = model
        button_texts = ['As library', 'As template', 'Cancel']
        self.set_title('Library choose dialog')
        self.resize(width=width, height=height)
        if pos is not None:
            self.set_position(pos)

        markup_text = "Choose a Library to substitute the state with."
        super(StateSubstituteChooseLibraryDialog, self).__init__(gtk.MESSAGE_INFO, gtk.BUTTONS_NONE, gtk.DIALOG_MODAL, parent)
        self.set_markup(markup_text)
        for button_text, option in zip(button_texts, ButtonDialog):
            self.add_button(button_text, option.value)
        self.finalize(self.check_for_library_path)

        self.widget_view = LibraryTreeView()
        self.widget_ctrl = StateSubstituteChooseLibraryDialogTreeController(self.model, self.widget_view,
                                                                            dialog_widget=self)

        self.vbox.pack_start(self.widget_view, True, True, 0)
        self.widget_view.show()

        self.grab_focus()
        self.run()

    def destroy(self):
        self.widget_view.destroy()
        self.widget_ctrl.destroy()
        super(StateSubstituteChooseLibraryDialog, self).destroy()

    def check_for_library_path(self, widget, response_id):

        if response_id == ButtonDialog.OPTION_1.value:
            logger.debug("Library substitute state as library triggered.")
            self.widget_ctrl.substitute_as_library_clicked(None)
        elif response_id == ButtonDialog.OPTION_2.value:
            logger.debug("Library substitute state as template triggered.")
            self.widget_ctrl.substitute_as_template_clicked(None)
        elif response_id == ButtonDialog.OPTION_3.value:
            pass
        else:
            logger.warning("Response id: {} is not considered".format(response_id))
        if response_id in [ButtonDialog.OPTION_1.value, ButtonDialog.OPTION_2.value, ButtonDialog.OPTION_3.value]:
            self.destroy()
