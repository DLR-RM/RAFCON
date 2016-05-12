import gtk
import gobject
from gtkmvc import View

from rafcon.mvc.utils import constants
from rafcon.mvc import gui_helper


class LinkageOverviewDataView(View):
    builder = './glade/linkage_overview_data.glade'
    top = 'tree_view'

    def __init__(self):
        View.__init__(self)


class LinkageOverviewLogicView(View):
    builder = './glade/linkage_overview_logic.glade'
    top = 'tree_view'

    def __init__(self):
        View.__init__(self)

        self.treeView = self
        self.tree_view = self['tree_view']
        self.tree = self['tree_view']

        # Use same tree_store structure as used to display all elements to reduce error possibility
        self.tree_store = gtk.TreeStore(str, str, str, str, str, str,
                                        gobject.TYPE_PYOBJECT, gobject.TYPE_PYOBJECT)
        self.tree_view.set_model(self.tree_store)


class LinkageOverviewView(View):
    builder = './glade/linkage_overview.glade'
    top = 'linkage_container'

    def __init__(self):
        View.__init__(self)

        self['inputs_view'] = LinkageOverviewDataView()
        self['outputs_view'] = LinkageOverviewDataView()
        self['scope_view'] = LinkageOverviewDataView()
        self['outcomes_view'] = LinkageOverviewLogicView()

        self['inputs_scroller'].add(self['inputs_view'].get_top_widget())
        self['outputs_scroller'].add(self['outputs_view'].get_top_widget())
        self['scoped_scroller'].add(self['scope_view'].get_top_widget())
        self['outcomes_scroller'].add(self['outcomes_view'].get_top_widget())

        gui_helper.set_label_markup(self['data_linkage_label'], 'DATA LINKAGE',
                                    letter_spacing=constants.LETTER_SPACING_1PT)
        gui_helper.set_label_markup(self['logical_linkage_label'], 'LOGICAL LINKAGE',
                                    letter_spacing=constants.LETTER_SPACING_1PT)
