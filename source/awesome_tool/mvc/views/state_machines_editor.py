import gtk
from gtkmvc import View


class StateMachinesEditorView(View):
    top = 'notebook'

    def __init__(self):
        View.__init__(self)
        self.notebook = gtk.Notebook()
        self.notebook.set_scrollable(True)
        self.notebook.show()
        self['notebook'] = self.notebook
