from gtkmvc import View


class ScopedVariablesListView(View):
    builder = './glade/ScopedVariablesListWidget.glade'
    top = 'scoped_variables_tree_view'

    def __init__(self):
        View.__init__(self)