from gtkmvc import View


class ToolBarView(View):
    builder = './glade/tool_bar.glade'
    top = 'toolbar'

    def __init__(self):
        View.__init__(self)