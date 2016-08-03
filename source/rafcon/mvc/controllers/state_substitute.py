import gtk
from gtkmvc import View

from rafcon.utils import log

logger = log.get_logger(__name__)
from rafcon.mvc.controllers.utils.single_widget_window import SingleWidgetWindowController
from rafcon.mvc.views.utils.single_widget_window import SingleWidgetWindowView
from rafcon.mvc.views.library_tree import LibraryTreeView
from rafcon.mvc.controllers.library_tree import LibraryTreeController
# single_view = SingleWidgetWindowView(LibraryTreeView)
# LibraryTreeController(self.model, single_view.widget_view)
# single_view.top = 'library_tree_view'
# single_view['library_tree_view'] = single_view.widget_view['library_tree_view']
# SingleWidgetWindowController(self.model, single_view, LibraryTreeController)
# logger.info("state substitute finish ctrl and view generation")


import gtk

from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.mvc.shortcut_manager import ShortcutManager
from rafcon.mvc.utils.dialog import RAFCONDialog, ButtonDialog


class StateSubstituteChooseLibraryDialogController(LibraryTreeController):

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
            return True
        if event.type == gtk.gdk._2BUTTON_PRESS and event.button == 2:
            (model, row) = self.view.get_selection().get_selected()
            if isinstance(model[row][1], dict):  # double click on folder, not library
                return False
            self.substitute_as_template_clicked(None)
            return True


class StateSubstituteChooseLibraryWindow(RAFCONDialog):

    def __init__(self, model, width=500, height=500, pos=None, parent=None):
        self.model = model
        title = 'Library choose dialog'
        button_texts=['As Library', 'As Template', 'Cancel']
        type=gtk.MESSAGE_INFO
        callback_args = ()

        markup_text = "Choose a Library to substitute the state with."
        super(StateSubstituteChooseLibraryWindow, self).__init__(type, gtk.BUTTONS_NONE, gtk.DIALOG_MODAL, parent)
        self.set_markup(markup_text)
        for button_text, option in zip(button_texts, ButtonDialog):
            self.add_button(button_text, option.value)
        self.finalize(self.check_for_library_path, *callback_args)

        self.set_title(title)
        self.resize(width=width, height=height)
        if pos is not None:
            self.set_position(pos)
        self.widget_view = LibraryTreeView()
        self.widget_ctrl = StateSubstituteChooseLibraryDialogController(self.model, self.widget_view)
        label = gtk.Label("Dialogs are groovy")

        self.vbox.pack_start(self.widget_view, True, True, 0)
        self.vbox.pack_start(label, True, True, 0)
        self.widget_view.show()
        label.show()

        self.grab_focus()
        self.run()

    def check_for_library_path(self, widget, response_id):
        logger.info("response_id: {}".format(response_id))
        if response_id == ButtonDialog.OPTION_1.value:
            logger.debug("Library refresh is triggered.")
            self.widget_ctrl.substitute_as_library_clicked(None)
        elif response_id == ButtonDialog.OPTION_2.value:
            logger.debug("Refresh all is triggered.")
            self.widget_ctrl.substitute_as_template_clicked(None)
        elif response_id == ButtonDialog.OPTION_3.value:
            pass
        else:
            logger.warning("Response id: {} is not considered".format(response_id))
        if response_id in [ButtonDialog.OPTION_1.value, ButtonDialog.OPTION_2.value, ButtonDialog.OPTION_3.value]:
            widget.destroy()
