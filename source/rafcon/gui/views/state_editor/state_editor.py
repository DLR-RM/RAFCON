# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gtkmvc3.view import View
from builtins import range

from rafcon.gui import glade
import rafcon.gui.helpers.label as gui_helper_label
from rafcon.gui.config import global_gui_config
from rafcon.gui.utils import constants
from rafcon.gui.views.state_editor.data_flows import StateDataFlowsEditorView
from rafcon.gui.views.state_editor.description_editor import DescriptionEditorView
from rafcon.gui.views.state_editor.input_port_list import InputPortsListView
from rafcon.gui.views.state_editor.linkage_overview import LinkageOverviewView
from rafcon.gui.views.state_editor.outcomes import StateOutcomesEditorView
from rafcon.gui.views.state_editor.output_port_list import OutputPortsListView
from rafcon.gui.views.state_editor.overview import StateOverviewView
from rafcon.gui.views.state_editor.scoped_variables_list import ScopedVariablesListView
from rafcon.gui.views.state_editor.source_editor import SourceEditorView
from rafcon.gui.views.state_editor.transitions import StateTransitionsEditorView
from rafcon.gui.views.state_editor.semantic_data_editor import SemanticDataEditorView


class StateEditorView(View):
    builder = glade.get_glade_path("state_editor_ld_widget_tab.glade")
    top = 'main_frame_vbox'

    icons = {
        _('Source'): constants.ICON_SOURCE,
        _("Data Linkage"): constants.ICON_DLINK,
        _("Logical Linkage"): constants.ICON_LLINK,
        _("Linkage Overview"): constants.ICON_OVERV,
        _("Description"): constants.ICON_DESC,
        _("Semantic Data"): constants.ICON_SEMANTICS
    }

    def __init__(self):
        View.__init__(self)

        self.page_dict = {}
        self.notebook_names = ['main_notebook_1', 'main_notebook_2']

        self.properties_view = StateOverviewView()
        self.inputs_view = InputPortsListView()
        self.outputs_view = OutputPortsListView()
        self.scopes_view = ScopedVariablesListView()
        self.outcomes_view = StateOutcomesEditorView()
        self.source_view = SourceEditorView()
        self.transitions_view = StateTransitionsEditorView()
        self.data_flows_view = StateDataFlowsEditorView()
        self.linkage_overview = LinkageOverviewView()
        self.description_view = DescriptionEditorView()
        self.semantic_data_view = SemanticDataEditorView()

        self['properties_viewport'].add(self.properties_view.get_top_widget())
        self['input_ports_scroller'].add(self.inputs_view.get_top_widget())
        self['output_ports_scroller'].add(self.outputs_view.get_top_widget())
        self['scoped_variables_scroller'].add(self.scopes_view.get_top_widget())
        self['outcomes_viewport'].add(self.outcomes_view.get_top_widget())
        self['source_viewport'].add(self.source_view.get_top_widget())
        self['transitions_viewport'].add(self.transitions_view.get_top_widget())
        self['data_flows_viewport'].add(self.data_flows_view.get_top_widget())
        self['linkage_overview_viewport'].add(self.linkage_overview.get_top_widget())
        self['description_viewport'].add(self.description_view.get_top_widget())
        self['semantic_data_viewport'].add(self.semantic_data_view.get_top_widget())

        self.inputs_view.scrollbar_widget = self['input_ports_scroller']
        self.outputs_view.scrollbar_widget = self['output_ports_scroller']
        self.scopes_view.scrollbar_widget = self['scoped_variables_scroller']

        self['description_text_view'] = self.description_view.textview
        self['description_scroller'] = self.description_view.scrollable

        # Gtk TODO
        # self['main_notebook_1'].set_tab_hborder(constants.TAB_BORDER_WIDTH * 2)
        # self['main_notebook_1'].set_tab_vborder(constants.TAB_BORDER_WIDTH * 3)
        # self['main_notebook_2'].set_tab_hborder(constants.TAB_BORDER_WIDTH * 2)
        # self['main_notebook_2'].set_tab_vborder(constants.TAB_BORDER_WIDTH * 3)
        # self['ports_notebook'].set_tab_hborder(constants.TAB_BORDER_WIDTH * 2)
        # self['ports_notebook'].set_tab_vborder(constants.TAB_BORDER_WIDTH * 3)

        self.page_dict["Source"] = self['main_notebook_1'].get_nth_page(0)
        self.page_dict["Logical Linkage"] = self['main_notebook_1'].get_nth_page(1)
        self.page_dict["Data Linkage"] = self['main_notebook_1'].get_nth_page(2)
        self.page_dict["Linkage Overview"] = self['main_notebook_2'].get_nth_page(0)
        self.page_dict["Description"] = self['main_notebook_2'].get_nth_page(1)
        self.page_dict["Semantic Data"] = self['main_notebook_2'].get_nth_page(2)
        self['main_notebook_1'].set_current_page(self['main_notebook_1'].page_num(self.page_dict["Source"]))
        self['main_notebook_2'].set_current_page(self['main_notebook_2'].page_num(self.page_dict["Description"]))

        self['add_input_port_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['remove_input_port_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['add_output_port_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['remove_output_port_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['add_scoped_variable_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['remove_scoped_variable_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)

        self._scoped_tab_page_number = None
        self._scoped_var_page = None
        self._scoped_var_page_tab_label = None
        self._source_tab_page_number = None
        self._source_page = None
        self._source_page_tab_label = None

        self.set_default_paned_positions()

    def prepare_the_labels(self):
        for notebook_name in self.notebook_names:
            notebook = self[notebook_name]
            for i in range(notebook.get_n_pages()):
                child = notebook.get_nth_page(i)
                tab_label = notebook.get_tab_label(child)
                if global_gui_config.get_config_value("USE_ICONS_AS_TAB_LABELS", True):
                    tab_label_text = tab_label.get_text()
                    notebook.set_tab_label(child, gui_helper_label.create_tab_header_label(tab_label_text, self.icons))
                else:
                    tab_label.set_angle(270)
                notebook.set_tab_reorderable(child, True)
                notebook.set_tab_detachable(child, True)

    def bring_tab_to_the_top(self, tab_label):
        """Find tab with label tab_label in list of notebook's and set it to the current page.

        :param tab_label: String containing the label of the tab to be focused
        """
        page = self.page_dict[tab_label]
        for notebook in self.notebook_names:
            page_num = self[notebook].page_num(page)
            if not page_num == -1:
                self[notebook].set_current_page(page_num)
                break

    def set_default_paned_positions(self):
        if global_gui_config.get_config_value("SEMANTIC_DATA_MODE", False):
            self['vpaned'].set_position(200)
        else:
            self['vpaned'].set_position(575)
        self['logic_vpaned'].set_position(300)
        self['data_vpaned'].set_position(300)

    def insert_source_tab(self):
        self['main_notebook_1'].insert_page(self._source_page, self._source_page_tab_label, self._source_tab_page_number)
        self['main_notebook_1'].set_tab_reorderable(self._source_page, True)
        self['main_notebook_1'].set_tab_detachable(self._source_page, True)

    def insert_scoped_variables_tab(self):
        self['ports_notebook'].insert_page(self._scoped_var_page, self._scoped_var_page_tab_label,
                                           self._scoped_tab_page_number)
        self['ports_notebook'].set_tab_reorderable(self._scoped_var_page, True)
        self['ports_notebook'].set_tab_detachable(self._scoped_var_page, True)

    def remove_source_tab(self):
        self._source_tab_page_number = self['main_notebook_1'].page_num(self['source_viewport'])
        self._source_page = self['main_notebook_1'].get_nth_page(self._source_tab_page_number)
        self._source_page_tab_label = self['main_notebook_1'].get_tab_label(self._source_page)
        self['main_notebook_1'].remove_page(self._source_tab_page_number)

    def remove_scoped_variables_tab(self):
        self._scoped_tab_page_number = self['ports_notebook'].page_num(self['scoped_variable_vbox'])
        self._scoped_var_page = self['ports_notebook'].get_nth_page(self._scoped_tab_page_number)
        self._scoped_var_page_tab_label = self['ports_notebook'].get_tab_label(self._scoped_var_page)
        self['ports_notebook'].remove_page(self._scoped_tab_page_number)

