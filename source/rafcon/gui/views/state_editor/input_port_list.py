# Copyright

from gtkmvc import View
from rafcon.gui.utils import constants


class InputPortsListView(View):
    builder = constants.get_glade_path("input_ports_list_widget.glade")
    top = 'input_ports_tree_view'

    def __init__(self):
        View.__init__(self)
