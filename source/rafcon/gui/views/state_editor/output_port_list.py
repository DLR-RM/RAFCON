from gtkmvc import View
from rafcon.gui.utils import constants


class OutputPortsListView(View):
    builder = constants.get_glade_path("output_ports_list_widget.glade")
    top = 'output_ports_tree_view'

    def __init__(self):
        View.__init__(self)
