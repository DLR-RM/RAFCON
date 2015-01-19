import gtk
from gtkmvc import View


class FullMainWindowView(View):
    builder = './glade/full_main_window.glade'
    top = 'MainWindow'

    def __init__(self):
        View.__init__()

        self.widgets["vbox"].reparent(self.container)