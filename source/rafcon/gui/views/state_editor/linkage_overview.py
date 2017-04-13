# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gtkmvc import View

from rafcon.gui import glade
import rafcon.gui.helpers.label as gui_helper_label
from rafcon.gui.utils import constants


class LinkageOverviewDataView(View):
    builder = glade.get_glade_path("linkage_overview_data.glade")
    top = 'tree_view'

    def __init__(self):
        View.__init__(self)


class LinkageOverviewLogicView(View):
    builder = glade.get_glade_path("linkage_overview_logic.glade")
    top = 'tree_view'

    def __init__(self):
        View.__init__(self)

        self.treeView = self


class LinkageOverviewView(View):
    builder = glade.get_glade_path("linkage_overview_one.glade")
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

        gui_helper_label.set_label_markup(self['data_linkage_label'], 'DATA LINKAGE',
                                          letter_spacing=constants.LETTER_SPACING_1PT)
        gui_helper_label.set_label_markup(self['logical_linkage_label'], 'LOGICAL LINKAGE',
                                          letter_spacing=constants.LETTER_SPACING_1PT)
