from gtkmvc import View


class InputPortsListView(View):
    builder = './glade/input_ports_list_widget.glade'
    top = 'input_ports_tree_view'

    def __init__(self):
        View.__init__(self)