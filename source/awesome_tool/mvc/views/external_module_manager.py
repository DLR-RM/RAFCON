from gtkmvc import View


class ExternalModuleManagerView(View):
    builder = './glade/ExternalModuleManagerWidget.glade'
    top = 'external_module_manager_widget'

    def __init__(self):
        View.__init__(self)

    pass