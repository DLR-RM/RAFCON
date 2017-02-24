# Copyright

from gtkmvc import View

from rafcon.gui.helpers.label import create_label_widget_with_icon
from rafcon.gui.utils import constants
from rafcon.utils.i18n import _


class ToolBarView(View):
    builder = constants.get_glade_path("tool_bar.glade")
    top = 'toolbar'

    def __init__(self):
        View.__init__(self)

        button_new = self['button_new']
        button_new.set_label_widget(create_label_widget_with_icon(constants.BUTTON_NEW, _("New State Machine")))

        button_refresh = self['button_refresh']
        button_refresh.set_label_widget(create_label_widget_with_icon(constants.BUTTON_REFR, _("Refresh")))

        button_open = self['button_open']
        button_open.set_label_widget(create_label_widget_with_icon(constants.BUTTON_OPEN, _("Open State Machine")))

        button_save = self['button_save']
        button_save.set_label_widget(create_label_widget_with_icon(constants.BUTTON_SAVE, _("Save State Machine")))

        button_refresh_libs = self['button_refresh_libs']
        button_refresh_libs.set_label_widget(
            create_label_widget_with_icon(constants.BUTTON_REFR, _("Refresh Libraries")))

        # self.get_top_widget().set_border_width(constants.BORDER_WIDTH)
