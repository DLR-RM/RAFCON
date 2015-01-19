from gtkmvc import View


class OutputPortsListView(View):
    builder = './glade/OutputPortsListWidget.glade'
    top = 'output_ports_tree_view'

    def __init__(self):
        View.__init__(self)

    pass