from gtkmvc import View

from rafcon.mvc.views.state_editor.source_editor import SourceEditorView
from rafcon.mvc.views.state_editor.description_editor import DescriptionEditorView
from rafcon.mvc.views.state_editor.outcomes import StateOutcomesEditorView
from rafcon.mvc.views.state_editor.overview import StateOverviewView
from rafcon.mvc.views.state_editor.input_port_list import InputPortsListView
from rafcon.mvc.views.state_editor.output_port_list import OutputPortsListView
from rafcon.mvc.views.state_editor.scoped_variables_list import ScopedVariablesListView
from rafcon.mvc.views.state_editor.transitions import StateTransitionsEditorView
from rafcon.mvc.views.state_editor.data_flows import StateDataFlowsEditorView
from rafcon.mvc.views.state_editor.linkage_overview import LinkageOverviewView
from rafcon.mvc.utils import constants
from rafcon.mvc import gui_helper


class StateEditorView(View):
    builder = constants.get_glade_path("state_editor_ld_widget_tab.glade")
    top = 'main_frame_vbox'

    def __init__(self):
        View.__init__(self)

        self.page_dict = {}
        self.notebook_names = ['main_notebook_1', 'main_notebook_2']

        gui_helper.set_label_markup(self['data_ports_label'], 'DATA PORTS', letter_spacing=constants.LETTER_SPACING_1PT)

        self['properties_view'] = StateOverviewView()
        self['inputs_view'] = InputPortsListView()
        self['outputs_view'] = OutputPortsListView()
        self['scopes_view'] = ScopedVariablesListView()
        self['outcomes_view'] = StateOutcomesEditorView()
        self['source_view'] = SourceEditorView()
        self['transitions_view'] = StateTransitionsEditorView()
        self['data_flows_view'] = StateDataFlowsEditorView()
        self['linkage_overview'] = LinkageOverviewView()
        self['description_view'] = DescriptionEditorView()

        self['properties_viewport'].add(self['properties_view'].get_top_widget())
        self['input_ports_scroller'].add(self['inputs_view'].get_top_widget())
        self['output_ports_scroller'].add(self['outputs_view'].get_top_widget())
        self['scoped_variables_scroller'].add(self['scopes_view'].get_top_widget())
        self['outcomes_viewport'].add(self['outcomes_view'].get_top_widget())
        self['source_viewport'].add(self['source_view'].get_top_widget())
        self['transitions_viewport'].add(self['transitions_view'].get_top_widget())
        self['data_flows_viewport'].add(self['data_flows_view'].get_top_widget())
        self['linkage_overview_viewport'].add(self['linkage_overview'].get_top_widget())
        self['description_viewport'].add(self['description_view'].get_top_widget())

        self['description_text_view'] = self['description_view'].textview
        self['description_scroller'] = self['description_view'].scrollable

        self['data_vpaned'].set_position(190)
        self['logic_vpaned'].set_position(190)

        self['main_notebook_1'].set_tab_hborder(constants.BORDER_WIDTH * 2)
        self['main_notebook_1'].set_tab_vborder(constants.BORDER_WIDTH * 3)
        self['main_notebook_2'].set_tab_hborder(constants.BORDER_WIDTH * 2)
        self['main_notebook_2'].set_tab_vborder(constants.BORDER_WIDTH * 3)
        self.page_dict["Source"] = self['main_notebook_1'].get_nth_page(0)
        self.page_dict["Logical Linkage"] = self['main_notebook_1'].get_nth_page(1)
        self.page_dict["Data Linkage"] = self['main_notebook_1'].get_nth_page(2)
        self.page_dict["Linkage Overview"] = self['main_notebook_2'].get_nth_page(0)
        self.page_dict["Description"] = self['main_notebook_2'].get_nth_page(1)
        self['main_notebook_1'].set_current_page(self['main_notebook_1'].page_num(self.page_dict["Source"]))
        self['main_notebook_2'].set_current_page(self['main_notebook_2'].page_num(self.page_dict["Description"]))

        self['vpaned'].set_position(450)

        self['new_input_port_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['delete_input_port_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['new_output_port_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['delete_output_port_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['new_scoped_variable_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['delete_scoped_variable_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)

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
