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

"""
.. module:: editor
   :synopsis: A module holds the controller to edit the state source text.

"""

from future.utils import string_types
from rafcon.core.states.library_state import LibraryState
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.helpers.label import react_to_event
from rafcon.utils import log

logger = log.get_logger(__name__)


class EditorController(ExtendedController):
    """Controller handling the text editor for States.

    :param
    :param rafcon.gui.views.source_editor.EditorView view: The GTK view showing the editor.
    """

    def __init__(self, model, view, observed_method="script_text"):
        """Constructor"""
        assert isinstance(observed_method, string_types)
        self._observed_method = observed_method
        super(EditorController, self).__init__(model, view)

    def register_view(self, view):
        super(EditorController, self).register_view(view)
        view.get_buffer().connect('changed', self.code_changed)

        if hasattr(view.get_buffer(), 'begin_not_undoable_action'):
            view.get_buffer().begin_not_undoable_action()
        view.set_text(self.source_text)
        if hasattr(view.get_buffer(), 'end_not_undoable_action'):
            view.get_buffer().end_not_undoable_action()

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        shortcut_manager.add_callback_for_action("copy", self._copy)
        shortcut_manager.add_callback_for_action("paste", self._paste)
        shortcut_manager.add_callback_for_action("cut", self._cut)
        shortcut_manager.add_callback_for_action("undo", self._undo)
        shortcut_manager.add_callback_for_action("redo", self._redo)
        shortcut_manager.add_callback_for_action("apply", self._apply)
        shortcut_manager.add_callback_for_action("open_external_editor", self._open_external_editor)

    def _copy(self, *args, **kwargs):
        pass

    def _paste(self, *args, **kwargs):
        pass

    def _cut(self, *args, **kwargs):
        pass

    def _undo(self, *event, **kwargs):
        if not self.view:
            return
        buffer = self.view.textview.get_buffer()
        if react_to_event(self.view, self.view.textview, event) and hasattr(buffer, 'can_undo') and buffer.can_undo():
            logger.debug('Run undo on {}'.format(self.__class__.__name__))
            return buffer.undo()
        return False

    def _redo(self, *event, **kwargs):
        if not self.view:
            return
        buffer = self.view.textview.get_buffer()
        if react_to_event(self.view, self.view.textview, event) and hasattr(buffer, 'can_redo') and buffer.can_redo():
            logger.debug('Run redo on {}'.format(self.__class__.__name__))
            return buffer.redo()
        return False

    def _apply(self, *event, **kwargs):

        if react_to_event(self.view, self.view.textview, event):
            logger.debug("Apply short-cut pressed {}".format(self.__class__.__name__))
            if self.source_text == self.view.get_text():
                logger.debug("Nothing to apply {}".format(self.__class__.__name__))
            else:
                self.apply_clicked(None)
            return True
        return False

    def _open_external_editor(self, *event, **kwargs):
        if react_to_event(self.view, self.view.textview, event):
            self.view['open_external_button'].set_active(True)
            return True
        return False

    @property
    def source_text(self):
        return ''

    @source_text.setter
    def source_text(self, text):
        pass

    # ===============================================================
    def code_changed(self, source):
        """ Apply checks and adjustments of the TextBuffer and TextView after every change in buffer.

         The method re-apply the tag (style) for the buffer. It avoids changes while editable-property set to False
         which are caused by a bug in the GtkSourceView2. GtkSourceView2 is the default used TextView widget here.
         The text buffer is reset after every change to last stored source-text by a respective work around which
         suspends any generation of undo items and avoids a recursive call of the method set_enabled by observing
         its while_in_set_enabled flag.

        :param TextBuffer source:
        :return:
        """

        # work around to avoid changes at all (e.g. by enter-key) if text view property editable is False
        # TODO if SourceView3 is used in future check if this can be skipped
        if not self.view.textview.get_editable() and not self.view.while_in_set_enabled:
            if hasattr(self.view.get_buffer(), 'begin_not_undoable_action'):
                self.view.get_buffer().begin_not_undoable_action()
            self.view.set_enabled(False, self.source_text)
            if hasattr(self.view.get_buffer(), 'end_not_undoable_action'):
                self.view.get_buffer().end_not_undoable_action()

        if self.view:
            self.view.apply_tag('default')

    def apply_clicked(self, button):
        """Triggered when the Apply-Shortcut in the editor is triggered.

        """
        if isinstance(self.model.state, LibraryState):
            return

        self.set_script_text(self.view.get_text())

    def set_script_text(self, text):
        if not self.source_text == text:
            self.source_text = text
            logger.debug("The script was saved")

    def cancel_clicked(self, button):
        """Triggered when the Cancel-Shortcut in the editor is triggered

        Resets the code in the editor to the last-saved code.
        """
        self.view.set_text(self.source_text)

    @ExtendedController.observe("state", after=True)
    def after_notification_of_script_text_was_changed(self, model, prop_name, info):
        if self.view and "method_name" in info and self._observed_method == info['method_name']:
            self.view.set_text(self.source_text)
