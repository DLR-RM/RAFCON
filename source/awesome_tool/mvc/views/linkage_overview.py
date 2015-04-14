import gtk
import gobject
from gtkmvc import View
from awesome_tool.utils import constants


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
        self.tree_store = gtk.TreeStore(str, str, str, str, str, str,
                                        gobject.TYPE_PYOBJECT, gobject.TYPE_PYOBJECT)
        self.tree_view.set_model(self.tree_store)


class LinkageOverviewView(View):
    builder = './glade/linkage_overview.glade'
    top = 'vbox1'

    def __init__(self):
        View.__init__(self)

        self['inputs_view'] = LinkageOverviewDataView()
        self['outputs_view'] = LinkageOverviewDataView()
        self['scopes_view'] = LinkageOverviewDataView()
        self['outcomes_view'] = LinkageOverviewLogicView()

        self['inputs_scroller'].add(self['inputs_view'].get_top_widget())
        self['outputs_scroller'].add(self['outputs_view'].get_top_widget())
        self['scoped_scroller'].add(self['scopes_view'].get_top_widget())
        self['outcomes_scroller'].add(self['outcomes_view'].get_top_widget())

        self['eventbox3'].set_border_width(constants.BORDER_WIDTH)
        self['eventbox4'].set_border_width(constants.BORDER_WIDTH)