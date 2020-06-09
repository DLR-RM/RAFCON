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

"""
.. module:: description_editor
   :synopsis: A module holds the controller to edit the state source script text.

"""

from rafcon.core.states.library_state import LibraryState

from rafcon.gui.controllers.utils.editor import EditorController

from rafcon.gui.helpers.label import react_to_event
from rafcon.utils import log

logger = log.get_logger(__name__)


class DescriptionEditorController(EditorController):
    """Controller handling the description editor of States.

    :param
    :param rafcon.gui.views.source_editor.SourceEditorView view: The GTK view showing the source editor.
    """

    def __init__(self, model, view):
        """Constructor"""
        super(DescriptionEditorController, self).__init__(model, view, observed_method="description")

    def register_view(self, view):
        super(DescriptionEditorController, self).register_view(view)

        view.textview.connect('size-allocate', self.scroll_to_bottom)

        if isinstance(self.model.state, LibraryState) or self.model.state.get_next_upper_library_root_state():
            view.textview.set_sensitive(True)
            state_copy = self.model.state.state_copy if isinstance(self.model.state, LibraryState) else self.model.state
            description = state_copy.description if state_copy.description is not None else ''
            view.textview.get_buffer().set_text(description)
            view.textview.set_editable(False)
        else:
            view.textview.connect('focus-out-event', self.on_focus_out)

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        super(DescriptionEditorController, self).register_actions(shortcut_manager)
        shortcut_manager.add_callback_for_action("abort", self._abort)

    def _abort(self, *event, **kwargs):
        if react_to_event(self.view, self.view.textview, event):
            logger.debug("Abort shortcut pressed {}".format(self.__class__.__name__))
            self.cancel_clicked(None)
            return True
        else:
            return False

    def on_focus_out(self, *args, **kwargs):
        self.apply_clicked(None)

    @property
    def source_text(self):
        return self.model.state.description if self.model.state.description is not None else ''

    @source_text.setter
    def source_text(self, text):
        if text == '':
            text = None
        self.model.state.description = text

    def scroll_to_bottom(self, widget, data=None):
        adj = self.view.scrollable.get_vadjustment()
        adj.set_value(adj.get_upper() - adj.get_page_size())
