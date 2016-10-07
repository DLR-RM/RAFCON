from gtkmvc import View
from rafcon.mvc.utils import constants


class SettingsWindowView(View):
    builder = constants.get_glade_path("settings_window.glade")
    top = 'properties_window'

    def __init__(self):
        View.__init__(self)

