from gtkmvc import View


class ExternalModuleManagerView(View):
    #builder = './glade/ExternalModuleManagerWindow.glade'
    builder = './glade/ExternalModuleManagerWidget.glade'
    #top = 'external_module_manager_window'
    top = 'external_modules_expander'

    def __init__(self):
        View.__init__(self)


class ExternalModuleManagerWindowView(View):
    builder = './glade/ExternalModuleManagerWindow.glade'
    top = 'external_module_manager_window'

    def __init__(self):
        View.__init__(self)