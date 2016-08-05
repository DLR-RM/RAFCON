from gtkmvc import View
from rafcon.mvc.utils import constants
from rafcon.mvc.gui_helper import create_label_widget_with_icon


class ToolBarView(View):
    builder = constants.get_glade_path("tool_bar.glade")
    top = 'toolbar'

    def __init__(self):
        View.__init__(self)

        button_new = self['button_new']
        button_new.set_label_widget(create_label_widget_with_icon(constants.BUTTON_NEW, "New State Machine"))

        button_refresh = self['button_refresh']
        button_refresh.set_label_widget(create_label_widget_with_icon(constants.BUTTON_REFR, "Refresh"))

        button_open = self['button_open']
        button_open.set_label_widget(create_label_widget_with_icon(constants.BUTTON_OPEN, "Open State Machine"))

        button_save = self['button_save']
        button_save.set_label_widget(create_label_widget_with_icon(constants.BUTTON_SAVE, "Save State Machine"))

        button_refresh_libs = self['button_refresh_libs']
        button_refresh_libs.set_label_widget(create_label_widget_with_icon(constants.BUTTON_REFR, "Refresh Libraries"))

        # self.get_top_widget().set_border_width(constants.BORDER_WIDTH)
