# Copyright

from gtkmvc import View

from rafcon.gui.utils import constants


class ScopedVariablesListView(View):
    builder = constants.get_glade_path("scoped_variables_list_widget.glade")
    top = 'scoped_variables_tree_view'

    def __init__(self):
        View.__init__(self)
