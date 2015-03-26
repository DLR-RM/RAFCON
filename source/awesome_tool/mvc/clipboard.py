from enum import Enum
from gtkmvc import Observable
from awesome_tool.mvc.selection import Selection

ClipboardType = Enum('CLIPBOARD_TYPE', 'CUT COPY')


class Clipboard(Observable):
    """
    A class to hold a selection for later usage e.g. triggered by copy, or cut. Later usage is e.g. paste.
    """
    def __init__(self):
        Observable.__init__(self)
        self._selection = None
        self.selection = Selection()

        self._state_machine_id = None
        self._clipboard_type = None

    def __str__(self):
        return "Clipboard:\nselection: %s\nstate_machine_id: %s\nclipboard_type: %s" % (str(self.selection),
                                                                                     str(self.state_machine_id),
                                                                                     str(self.clipboard_type))

    def set_selection(self, selection):
        self.selection.set(selection)

    @property
    def selection(self):
        """Property for the _selection field

        """
        return self._selection

    @selection.setter
    @Observable.observed
    def selection(self, selection):
        if not isinstance(selection, Selection):
            raise TypeError("selection must be of type Selection")
        self._selection = selection

    @property
    def state_machine_id(self):
        """Property for the _state_machine_id field

        """
        return self._state_machine_id

    @state_machine_id.setter
    @Observable.observed
    def state_machine_id(self, state_machine_id):
        if not isinstance(state_machine_id, int):
            raise TypeError("state_machine_id must be of type int")
        self._state_machine_id = state_machine_id

    @property
    def clipboard_type(self):
        """Property for the _clipboard_type field

        """
        return self._clipboard_type

    @clipboard_type.setter
    @Observable.observed
    def clipboard_type(self, clipboard_type):
        if not isinstance(clipboard_type, ClipboardType):
            raise TypeError("clipboard_type must be of type ClipBoardType")
        self._clipboard_type = clipboard_type

    def copy(self, state_machine_id, models):
        self.state_machine_id = state_machine_id
        self.selection.set(models)
        self.clipboard_type = ClipboardType.COPY

    def cut(self, state_machine_id, models):
        self.state_machine_id = state_machine_id
        self.selection.set(models)
        self.clipboard_type = ClipboardType.CUT

# To enable copy, cut and paste between state machines a global clipboard is used
global_clipboard = Clipboard()