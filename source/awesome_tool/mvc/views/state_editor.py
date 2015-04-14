"""
.. module:: state_editor (View)
   :platform: Unix, Windows
   :synopsis: A module to view the all aspects of a respective state

.. moduleauthor:: Rico Belder

"""
import pango

import gtk
from gtkmvc import View
from awesome_tool.mvc.views import SourceEditorView, StateOutcomesEditorView
from awesome_tool.mvc.views.state_overview import StateOverviewView
from awesome_tool.mvc.views.input_port_list import InputPortsListView
from awesome_tool.mvc.views.output_port_list import OutputPortsListView
from awesome_tool.mvc.views.scoped_variables_list import ScopedVariablesListView
from awesome_tool.mvc.views.state_transitions import StateTransitionsEditorView
from awesome_tool.mvc.views.state_data_flows import StateDataFlowsEditorView
from awesome_tool.mvc.views.linkage_overview import LinkageOverviewView
from awesome_tool.utils import constants


class StateEditorView(View):
    builder = './glade/state_editor_ld_widget_tab.glade'
    top = 'main_frame_vbox'

    def __init__(self):
        View.__init__(self)

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

        self['data_vpaned'].set_position(200)
        self['logic_vpaned'].set_position(200)

        self['main_notebook_1'].set_tab_hborder(constants.BORDER_WIDTH * 2)
        self['main_notebook_1'].set_tab_vborder(constants.BORDER_WIDTH * 3)
        self['main_notebook_2'].set_tab_hborder(constants.BORDER_WIDTH * 2)
        self['main_notebook_2'].set_tab_vborder(constants.BORDER_WIDTH * 3)

        self['vpaned'].set_position(325)

        self['new_input_port_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['delete_input_port_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['new_output_port_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['delete_output_port_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['new_scoped_variable_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['delete_scoped_variable_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)

        self['top_spacing_alignment'].set_size_request(0, constants.BORDER_WIDTH)
        self['bottom_spacing_alignment'].set_size_request(0, constants.BORDER_WIDTH)

    # def resize_logic_data_expander(self, widget, data=None):
    # # deactivate other expanders
    #     for expander in self.expanders:
    #         if not expander is widget:
    #             expander.set_expanded(False)
    #
    #     # set the packing expanded value correct
    #     for expander in self.expanders:
    #         expand, fill, padding, pack_type = self.top_box.query_child_packing(expander)
    #         if expander is widget:
    #             self.top_box.set_child_packing(expander, True, fill, padding, pack_type)
    #             paned = self.get_paned_child(widget)
    #             if paned is not None:
    #                 paned.set_position(200)
    #         else:
    #             self.top_box.set_child_packing(expander, False, fill, padding, pack_type)
    #
    # def get_paned_child(self, widget):
    #     for child in widget.get_children():
    #         if isinstance(child, gtk.VPaned):
    #             return child
    #         elif isinstance(child, gtk.Container):
    #             return self.get_paned_child(child)