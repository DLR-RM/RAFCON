from gtkmvc import View


class GlobalVariableEditorView(View):
    builder = './glade/global_variable_editor_widget.glade'
    top = 'global_variable_vbox'

    def __init__(self):
        View.__init__(self)