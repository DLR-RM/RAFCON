from gtkmvc import View


class GlobalVariableEditorView(View):
    builder = './glade/GlobalVariableEditorWidget.glade'
    top = 'global_variable_expander'

    def __init__(self):
        View.__init__(self)