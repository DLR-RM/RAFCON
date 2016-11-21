"""
.. module:: editor
   :platform: Unix, Windows
   :synopsis: A module holds the controller to edit the state source text.

.. moduleauthor:: Rico Belder


"""

from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.statemachine.states.library_state import LibraryState

from rafcon.mvc.gui_helper import react_to_event
from rafcon.utils import log

logger = log.get_logger(__name__)


class EditorController(ExtendedController):
    """Controller handling the text editor for States.

    :param
    :param rafcon.mvc.views.source_editor.EditorView view: The GTK view showing the editor.
    """

    def __init__(self, model, view, observed_method="script_text"):
        """Constructor"""
        assert isinstance(observed_method, str)
        self._observed_method = observed_method
        ExtendedController.__init__(self, model, view)

    def register_view(self, view):
        view.get_buffer().connect('changed', self.code_changed)

        view.get_buffer().begin_not_undoable_action()
        view.set_text(self.source_text)
        view.get_buffer().end_not_undoable_action()

    def register_adapters(self):
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        shortcut_manager.add_callback_for_action("copy", self._copy)
        shortcut_manager.add_callback_for_action("paste", self._paste)
        shortcut_manager.add_callback_for_action("cut", self._cut)
        shortcut_manager.add_callback_for_action("undo", self._undo)
        shortcut_manager.add_callback_for_action("redo", self._redo)
        shortcut_manager.add_callback_for_action("apply", self._apply)
        shortcut_manager.add_callback_for_action("open_external_editor", self._open_external_editor)

    def _copy(self, *args):
        pass

    def _paste(self, *args):
        pass

    def _cut(self, *args):
        pass

    def _undo(self, *event):
        if not self.view:
            return
        buffer = self.view.textview.get_buffer()
        if react_to_event(self.view, self.view.textview, event) and hasattr(buffer, 'can_undo') and buffer.can_undo():
            logger.debug('Run undo on {}'.format(self.__class__.__name__))
            return buffer.undo()
        return False

    def _redo(self, *event):
        if not self.view:
            return
        buffer = self.view.textview.get_buffer()
        if react_to_event(self.view, self.view.textview, event) and hasattr(buffer, 'can_redo') and buffer.can_redo():
            logger.debug('Run redo on {}'.format(self.__class__.__name__))
            return buffer.redo()
        return False

    def _apply(self, *event):

        if react_to_event(self.view, self.view.textview, event):
            logger.debug("Apply short-cut pressed {}".format(self.__class__.__name__))
            tbuffer = self.view.get_buffer()
            current_text = tbuffer.get_text(tbuffer.get_start_iter(), tbuffer.get_end_iter())
            if self.source_text == current_text:
                logger.debug("Nothing to apply {}".format(self.__class__.__name__))
            else:
                self.apply_clicked(None)
            return True
        return False

    def _open_external_editor(self, *event):
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
        if self.view:
            self.view.apply_tag('default')

    def apply_clicked(self, button):
        """Triggered when the Apply-Shortcut in the editor is triggered.

        """
        if isinstance(self.model.state, LibraryState):
            return

        tbuffer = self.view.get_buffer()
        current_text = tbuffer.get_text(tbuffer.get_start_iter(), tbuffer.get_end_iter())

        self.set_script_text(current_text)

    def set_script_text(self, text):
        if not self.source_text == text:
            self.source_text = text
            logger.debug("The source was saved {}.".format(self.__class__.__name__))
        else:
            logger.debug("Source is the same as in storage {}.".format(self.__class__.__name__))

    def cancel_clicked(self, button):
        """Triggered when the Cancel-Shortcut in the editor is triggered

        Resets the code in the editor to the last-saved code.
        """
        self.view.set_text(self.source_text)

    @ExtendedController.observe("state", after=True)
    def after_notification_of_script_text_was_changed(self, model, prop_name, info):

        if self.view and "method_name" in info and self._observed_method == info['method_name']:
            self.view.set_text(self.source_text)
