from gtkmvc import View


class ScopedVariablesListView(View):
    builder = './glade/scoped_variables_list_widget.glade'
    top = 'scoped_variables_tree_view'

    def __init__(self):
        View.__init__(self)
