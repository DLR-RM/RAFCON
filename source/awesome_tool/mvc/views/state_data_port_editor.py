from gtkmvc import View
from mvc.views.input_port_list import InputPortsListView
from mvc.views.output_port_list import OutputPortsListView
from mvc.views.scoped_variables_list import ScopedVariablesListView


class StateDataportEditorView(View):
    builder = './glade/StateDataportEditor.glade'
    top = 'state_dataport_editor'

    def __init__(self):

        View.__init__(self)
        self.input_port_list_view = InputPortsListView()
        self.output_port_list_view = OutputPortsListView()
        self.scoped_variables_list_view = ScopedVariablesListView()

        self['input_ports_scroller'].add(self.input_port_list_view.get_top_widget())
        self['output_ports_scroller'].add(self.output_port_list_view.get_top_widget())
        self['scoped_variables_scroller'].add(self.scoped_variables_list_view.get_top_widget())

        self.top_box = self['vbox1']

        self.expanders = []
        self.expanders.append(self["input_ports_expander"])
        self.expanders[0].connect("activate", self.on_expander_activate)

        self.expanders.append(self["output_ports_expander"])
        self.expanders[1].connect("activate", self.on_expander_activate)

        self.expanders.append(self["scoped_variables_expander"])
        self.expanders[2].connect("activate", self.on_expander_activate)

    #this will be called before the expander is activated
    def on_expander_activate(self, widget, data=None):

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