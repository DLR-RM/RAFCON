import gtk
from gtkmvc import View


class StatesEditorView(View):
    top = 'notebook'

    def __init__(self):
        View.__init__(self)
        self.notebook = gtk.Notebook()
        self.notebook.show()
        self['notebook'] = self.notebook

