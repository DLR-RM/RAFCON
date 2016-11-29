from gtkmvc import View
from rafcon.gui.utils import constants


class GlobalVariableEditorView(View):
    builder = constants.get_glade_path("global_variable_editor_widget.glade")
    top = 'global_variable_vbox'

    def __init__(self):
        View.__init__(self)

        self['new_global_variable_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['delete_global_variable_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
