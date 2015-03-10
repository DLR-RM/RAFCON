"""
.. module:: state_editor (View)
   :platform: Unix, Windows
   :synopsis: A module to view the all aspects of a respective state

.. moduleauthor:: Rico Belder

"""
import pango

import gtk
from gtkmvc import View, Controller
from awesome_tool.mvc.views import SourceEditorView, StateOutcomesEditorView, StateOutcomesTreeView
from awesome_tool.mvc.views.container_state import ContainerStateView
from awesome_tool.mvc.views.connections_editor import StateConnectionsEditorView
from awesome_tool.mvc.views.state_overview import StateOverviewView
from awesome_tool.mvc.views.input_port_list import InputPortsListView
from awesome_tool.mvc.views.output_port_list import OutputPortsListView
from awesome_tool.mvc.views.scoped_variables_list import ScopedVariablesListView
from awesome_tool.mvc.views.state_transitions import StateTransitionsEditorView
from awesome_tool.mvc.views.state_data_flows import StateDataFlowsEditorView


class StateEditorView(View):
    builder = './glade/state_editor_widget.glade'
    top = 'main_frame_vbox'

    def __init__(self):
        View.__init__(self)

        self['properties_view'] = StateOverviewView()  # StatePropertiesEditorView()
        self['inputs_view'] = InputPortsListView()  # StateInputsEditorView()
        self['outputs_view'] = OutputPortsListView()  # StateOutputsEditorView()
        self['scopes_view'] = ScopedVariablesListView()
        self['outcomes_view'] = StateOutcomesEditorView()
        self['source_view'] = SourceEditorView()
        self['transitions_view'] = StateTransitionsEditorView()
        self['data_flows_view'] = StateDataFlowsEditorView()

        self['properties_viewport'].add(self['properties_view'].get_top_widget())
        self['input_ports_scroller'].add(self['inputs_view'].get_top_widget())
        self['output_ports_scroller'].add(self['outputs_view'].get_top_widget())
        self['scoped_variables_scroller'].add(self['scopes_view'].get_top_widget())
        self['outcomes_viewport'].add(self['outcomes_view'].get_top_widget())
        self['source_viewport'].add(self['source_view'].get_top_widget())
        self['transitions_viewport'].add(self['transitions_view'].get_top_widget())
        self['data_flows_viewport'].add(self['data_flows_view'].get_top_widget())

        self['port_expander'].connect('size-request', self.resize_port_widget)
        self['port_expander1'].connect('size-request', self.resize_port_widget)
        self['port_expander2'].connect('size-request', self.resize_port_widget)
        self['port_expander3'].connect('size-request', self.resize_port_widget)
        self['port_expander4'].connect('size-request', self.resize_port_widget)

    def resize_port_widget(self, expander, x):

        if self['port_expander'].get_expanded():
            count = 0
            for i in range(1, 4):
                if self['port_expander'+str(i)].get_expanded():
                    count += 1
            self['port_expander'].set_size_request(width=-1, height=count*170+85)
        else:
            self['port_expander'].set_size_request(width=-1, height=-1)


class StateEditorEggView(View):
    builder = './glade/state_editor_egg_widget.glade'
    top = 'main_frame_vbox'

    def __init__(self):
        View.__init__(self)

        self['properties_view'] = StateOverviewView()  # StatePropertiesEditorView()
        self['inputs_view'] = InputPortsListView()  # StateInputsEditorView()
        self['outputs_view'] = OutputPortsListView()  # StateOutputsEditorView()
        self['scopes_view'] = ScopedVariablesListView()
        self['outcomes_view'] = StateOutcomesEditorView()
        self['source_view'] = SourceEditorView()
        self['transitions_view'] = StateTransitionsEditorView()
        self['data_flows_view'] = StateDataFlowsEditorView()

        fontdesc = pango.FontDescription("Bold")
        self['label_dataport'].modify_font(fontdesc)
        self['label_script'].modify_font(fontdesc)
        self['label_connections'].modify_font(fontdesc)
        self['label_script1'].modify_font(fontdesc)
        self['label_script2'].modify_font(fontdesc)
        self['label_script3'].modify_font(fontdesc)
        self['label_script4'].modify_font(fontdesc)
        self['label_script5'].modify_font(fontdesc)
        self['label_script6'].modify_font(fontdesc)

        self['properties_viewport'].add(self['properties_view'].get_top_widget())
        self['input_ports_scroller'].add(self['inputs_view'].get_top_widget())
        self['output_ports_scroller'].add(self['outputs_view'].get_top_widget())
        self['scoped_variables_scroller'].add(self['scopes_view'].get_top_widget())
        self['outcomes_viewport'].add(self['outcomes_view'].get_top_widget())
        self['source_viewport'].add(self['source_view'].get_top_widget())
        self['transitions_viewport'].add(self['transitions_view'].get_top_widget())
        self['data_flows_viewport'].add(self['data_flows_view'].get_top_widget())


class StateEditorLDView(View):
    builder = './glade/state_editor_ld_widget.glade'
    top = 'main_frame_vbox'

    def __init__(self):
        View.__init__(self)

        self['properties_view'] = StateOverviewView()  # StatePropertiesEditorView()
        self['inputs_view'] = InputPortsListView()  # StateInputsEditorView()
        self['outputs_view'] = OutputPortsListView()  # StateOutputsEditorView()
        self['scopes_view'] = ScopedVariablesListView()
        self['outcomes_view'] = StateOutcomesEditorView()
        self['source_view'] = SourceEditorView()
        self['transitions_view'] = StateTransitionsEditorView()
        self['data_flows_view'] = StateDataFlowsEditorView()

        self['properties_viewport'].add(self['properties_view'].get_top_widget())
        self['input_ports_scroller'].add(self['inputs_view'].get_top_widget())
        self['output_ports_scroller'].add(self['outputs_view'].get_top_widget())
        self['scoped_variables_scroller'].add(self['scopes_view'].get_top_widget())
        self['outcomes_viewport'].add(self['outcomes_view'].get_top_widget())
        self['source_viewport'].add(self['source_view'].get_top_widget())
        self['transitions_viewport'].add(self['transitions_view'].get_top_widget())
        self['data_flows_viewport'].add(self['data_flows_view'].get_top_widget())

        self.top_box = self['vbox1']
        self.expanders = [self['data_expander'], self['logic_expander']]
        self['data_expander'].connect('activate', self.resize_logic_data_expander)
        self['logic_expander'].connect('activate', self.resize_logic_data_expander)

    def resize_logic_data_expander(self, widget, data=None):
    # deactivate other expanders
        for expander in self.expanders:
            if not expander is widget:
                expander.set_expanded(False)

        # set the packing expanded value correct
        for expander in self.expanders:
            expand, fill, padding, pack_type = self.top_box.query_child_packing(expander)
            if expander is widget:
                self.top_box.set_child_packing(expander, True, fill, padding, pack_type)
            else:
                self.top_box.set_child_packing(expander, False, fill, padding, pack_type)

