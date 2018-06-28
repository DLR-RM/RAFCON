# Copyright

from gtkmvc import View


class TreeView(View):
    builder = None
    top = None

    def __init__(self):
        super(TreeView, self).__init__()
        self.scrollbar_widget = None
