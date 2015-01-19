from gtkmvc import View


class InputPortsListView(View):
    builder = './glade/InputPortsListWidget.glade'
    top = 'input_ports_tree_view'


    def __init__(self):
        View.__init__(self)

    pass