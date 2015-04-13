import gtk
from gtkmvc import View
from awesome_tool.utils import constants


class StatesEditorView(View):
    top = 'notebook'

    def __init__(self):
        View.__init__(self)
        self.notebook = gtk.Notebook()
        self.notebook.set_scrollable(True)
        self.notebook.show()
        self.notebook.set_tab_hborder(constants.BORDER_WIDTH * 2)
        self.notebook.set_tab_vborder(constants.BORDER_WIDTH * 3)
        self['notebook'] = self.notebook

