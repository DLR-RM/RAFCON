from gi.repository import Gtk


class View:
    """
    The View class of the MVC pattern. It holds the widgets and allows the users to connect the signals of the widgets to
    the custom callbacks.
    """

    def __init__(self, builder_filename=None, parent=None):
        self._widgets = {}
        # 'Gtk.Builder' takes 'builder_filename' as the argument and creates all the widgets automatically
        self._builder = builder_filename
        self._parent = parent
        if builder_filename is not None:
            self._builder = Gtk.Builder()
            self._builder.add_from_file(builder_filename)
            for widget in self._builder.get_objects():
                if isinstance(widget, Gtk.Buildable):
                    self._widgets[Gtk.Buildable.get_name(widget)] = widget

    def __getitem__(self, key):
        if key in self._widgets:
            return self._widgets[key]
        elif self._builder is not None:
            obj = self._builder.get_object(key)
            if obj is not None:
                self._widgets[key] = obj
                return obj
        raise KeyError(key)

    def __setitem__(self, key, value):
        self._widgets[key] = value

    def get_parent_widget(self):
        """
        Gets the parent widget
        """

        return self[self._parent]

    def show(self):
        """
        Shows the parent widget
        """

        self.get_parent_widget().show_all()

    def hide(self):
        """
        Hides the parent widget
        """

        self.get_parent_widget().hide()

    def connect_signals(self, callbacks):
        """
        Connects the signals of the widgets to the custom callbacks
        """

        if self._builder is not None:
            self._builder.connect_signals(callbacks)
