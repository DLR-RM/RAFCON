from gtkmvc import View


class OutputPortsListView(View):
    builder = './glade/output_ports_list_widget.glade'
    top = 'output_ports_tree_view'

    def __init__(self):
        View.__init__(self)
