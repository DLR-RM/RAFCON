from gtkmvc import View
import gtk


class MenuBarView(View):
    builder = './glade/menu_bar.glade'
    top = 'menubar'

    def __init__(self):
        View.__init__(self)