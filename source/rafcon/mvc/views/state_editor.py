import gtk
from gtkmvc import View
from rafcon.mvc.views.source_editor import SourceEditorView
from rafcon.mvc.views.state_outcomes import StateOutcomesEditorView
from rafcon.mvc.views.state_overview import StateOverviewView
from rafcon.mvc.views.input_port_list import InputPortsListView
from rafcon.mvc.views.output_port_list import OutputPortsListView
from rafcon.mvc.views.scoped_variables_list import ScopedVariablesListView
from rafcon.mvc.views.state_transitions import StateTransitionsEditorView
from rafcon.mvc.views.state_data_flows import StateDataFlowsEditorView
from rafcon.mvc.views.linkage_overview import LinkageOverviewView
from rafcon.mvc.utils import constants
from rafcon.mvc import gui_helper


class StateEditorView(View):
    builder = './glade/state_editor_ld_widget_tab.glade'
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

        self['properties_viewport'].add(self['properties_view'].get_top_widget())
        self['input_ports_scroller'].add(self['inputs_view'].get_top_widget())
        self['output_ports_scroller'].add(self['outputs_view'].get_top_widget())
        self['scoped_variables_scroller'].add(self['scopes_view'].get_top_widget())
        self['outcomes_viewport'].add(self['outcomes_view'].get_top_widget())
        self['source_viewport'].add(self['source_view'].get_top_widget())
        self['transitions_viewport'].add(self['transitions_view'].get_top_widget())
        self['data_flows_viewport'].add(self['data_flows_view'].get_top_widget())
        self['linkage_overview_viewport'].add(self['linkage_overview'].get_top_widget())

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

        description_container = gtk.VBox()
        description_container.set_name('description_container')
        description_label = gui_helper.create_label_with_text_and_spacing('DESCRIPTION',
                                                                          letter_spacing=constants.LETTER_SPACING_1PT)
        description_label.set_alignment(0.0, 0.5)

        textview = gtk.TextView()
        textview.set_accepts_tab(False)
        textview.set_wrap_mode(gtk.WRAP_WORD)
        textview.set_border_width(10)
        self['description_text_view'] = textview

        description_title_box = gtk.EventBox()
        description_title_box.set_name("label_wrapper")
        description_title_box.set_border_width(constants.GRID_SIZE)
        description_title_box.add(description_label)

        title_viewport = gtk.Viewport()
        title_viewport.set_name("description_title_wrapper")
        title_viewport.add(description_title_box)
        title_viewport.show_all()

        description_scroller = gtk.ScrolledWindow()
        description_scroller.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        description_scroller.set_name('description_scroller')
        description_scroller.add(textview)
        self['description_scroller'] = description_scroller

        description_container.pack_start(title_viewport, False, True, 0)
        description_container.pack_start(description_scroller, True, True, 0)
        description_container.show()

        self['description_viewport'].add(description_container)

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
