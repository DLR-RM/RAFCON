# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: state_substitute
   :synopsis: A module that holds the controller with the interface for the user to choose a library state
   that is used to substitute another state.

"""

from gi.repository import Gtk
from gi.repository import Gdk

from rafcon.gui.views.library_tree import LibraryTreeView
from rafcon.gui.controllers.library_tree import LibraryTreeController
from rafcon.gui.utils.dialog import RAFCONButtonDialog
from rafcon.gui.singleton import global_gui_config

from rafcon.utils import log

logger = log.get_logger(__name__)


class StateSubstituteChooseLibraryDialogTreeController(LibraryTreeController):

    def __init__(self, model, view, dialog_widget=None):
        super(StateSubstituteChooseLibraryDialogTreeController, self).__init__(model, view)
        self.dialog_widget = dialog_widget
        self.keep_name = global_gui_config.get_config_value('SUBSTITUTE_STATE_KEEPS_STATE_NAME')

    def generate_right_click_menu(self, *args, **kwargs):
        pass

    def mouse_click(self, widget, event=None):
        # Double click with left mouse button
        if event.type == Gdk.EventType._2BUTTON_PRESS and event.get_button()[1] == 1:
            (model, row) = self.view.get_selection().get_selected()
            if isinstance(model[row][1], dict):  # double click on folder, not library
                state_row_path = self.tree_store.get_path(row)
                if state_row_path is not None:
                    if self.view.row_expanded(state_row_path):
                        self.view.collapse_row(state_row_path)
                    else:
                        self.view.expand_to_path(state_row_path)
                return False
            self.substitute_as_library_clicked(None, self.keep_name)
            if self.dialog_widget:
                self.dialog_widget.destroy()
            return True
        if event.type == Gdk.EventType._2BUTTON_PRESS and event.get_button()[1] == 3:
            (model, row) = self.view.get_selection().get_selected()
            if isinstance(model[row][1], dict):  # double click on folder, not library
                return False
            self.substitute_as_template_clicked(None, self.keep_name)
            if self.dialog_widget:
                self.dialog_widget.destroy()
            return True


class StateSubstituteChooseLibraryDialog(RAFCONButtonDialog):

    def __init__(self, model, width=500, height=600, pos=None, parent=None):
        self.model = model

        super(StateSubstituteChooseLibraryDialog, self).__init__("Choose a Library to substitute the state with.",
                                                                 ['As library', 'As template', 'Cancel'],
                                                                 title='Library choose dialog',
                                                                 callback=self.check_for_library_path,
                                                                 message_type=Gtk.MessageType.INFO,
                                                                 flags=Gtk.DialogFlags.MODAL, parent=parent)

        self.set_title('Library choose dialog')
        self.resize(width=width, height=height)
        if pos is not None:
            self.move(*pos)

        self.set_resizable(True)

        self.scrollable = Gtk.ScrolledWindow()
        self.widget_view = LibraryTreeView()
        self.widget_ctrl = StateSubstituteChooseLibraryDialogTreeController(self.model, self.widget_view,
                                                                            dialog_widget=self)
        self.scrollable.add(self.widget_view)

        self.keep_name_check_box = Gtk.CheckButton()
        self.keep_name_check_box.set_active(self.widget_ctrl.keep_name)
        self.keep_name_check_box.set_label("Keep state name")
        self.keep_name_check_box.connect('toggled', self.on_toggle_keep_name)

        self.vbox.pack_end(self.keep_name_check_box, False, False, 0)
        self.vbox.pack_start(self.scrollable, True, True, 0)

        self.vbox.show_all()
        self.grab_focus()
        self.run()

    def on_toggle_keep_name(self, button):
        self.widget_ctrl.keep_name = button.get_active()

    def destroy(self):
        self.widget_view.destroy()
        self.widget_ctrl.destroy()
        super(StateSubstituteChooseLibraryDialog, self).destroy()

    def check_for_library_path(self, widget, response_id):
        logger.info("check box is active: {}".format(self.widget_ctrl.keep_name))
        if response_id == 1:
            logger.debug("Library substitute state as library triggered.")
            self.widget_ctrl.substitute_as_library_clicked(None, self.widget_ctrl.keep_name)
        elif response_id == 2:
            logger.debug("Library substitute state as template triggered.")
            self.widget_ctrl.substitute_as_template_clicked(None, self.widget_ctrl.keep_name)
        elif response_id in [3, -4]:
            pass
        else:
            logger.warning("Response id: {} is not considered".format(response_id))
            return
        self.destroy()

