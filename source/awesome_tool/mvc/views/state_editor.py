import gtk
from gtkmvc import View, Controller
from mvc.views.source_editor import SourceEditorView
from mvc.views.container_state import ContainerStateView
from mvc.views.connections_editor import StateConnectionsEditorView
from mvc.views.state_overview import StateOverviewView
from mvc.views.input_port_list import InputPortsListView
from mvc.views.output_port_list import OutputPortsListView
from mvc.views.scoped_variables_list import ScopedVariablesListView


class StateEditorView(View):
    builder = './glade/state_editor_widget.glade'
    top = 'main_frame_vbox'

    def __init__(self):
        View.__init__(self)

        self['properties_view'] = StateOverviewView()  # StatePropertiesEditorView()
        self['ports_editor_view'] = ContainerStateView()  # StatePortsEditorView()
        self['inputs_view'] = InputPortsListView()  # StateInputsEditorView()
        self['outputs_view'] = OutputPortsListView()  # StateOutputsEditorView()
        self['scopes_view'] = ScopedVariablesListView()  # StateScopesEditorView()
        self['outcomes_view'] = ContainerStateView()  # StateOutcomesEditorView()
        self['source_view'] = SourceEditorView()
        self['connections_view'] = StateConnectionsEditorView()

        self['properties_viewport'].add(self['properties_view'].get_top_widget())
        self['scrolledwindow1'].add(self['inputs_view'].get_top_widget())
        self['viewport4'].add(self['outputs_view'].get_top_widget())
        self['viewport5'].add(self['scopes_view'].get_top_widget())
        self['viewport6'].add(self['outcomes_view']['vbox1'])
        self['viewport1'].add(self['source_view'].get_top_widget())
        self['alignment1'].add(self['connections_view']['connections_editor_widget'])
        #self['expander3_ScriptEditor'].add(self['source_editor_view'].get_top_widget())
        #self['expander4_ConnectionEditor'].add(self['state_connections_editor_view']['vbox1'])

        self['port_expander'].connect('size-request', self.resize_port_widget)
        self['port_expander1'].connect('size-request', self.resize_port_widget)
        self['port_expander2'].connect('size-request', self.resize_port_widget)
        self['port_expander3'].connect('size-request', self.resize_port_widget)
        self['port_expander4'].connect('size-request', self.resize_port_widget)
        self['connections_expander'].connect('size-request', self.resize_connections_widget)
        self['connections_view']['transitions_expander'].connect('size-request', self.resize_connections_widget)
        self['connections_view']['dataflows_expander'].connect('size-request', self.resize_connections_widget)

    def resize_port_widget(self, expander, x):

        if self['port_expander'].get_expanded():
            count = 0
            for i in range(1, 5):
                if self['port_expander'+str(i)].get_expanded():
                    print "%s is expanded" % ('port_expander'+str(i))
                    count += 1
            self['port_expander'].set_size_request(width=-1, height=count*150+100)
        else:
            self['port_expander'].set_size_request(width=-1, height=-1)

    def resize_connections_widget(self, expander, x):

        if self['connections_expander'].get_expanded():
            count = 0
            for expand_id in ['transitions_expander', 'dataflows_expander']:
                if self['connections_view'][expand_id].get_expanded():
                    print "%s is expanded" % expand_id
                    count += 1
            self['connections_expander'].set_size_request(width=-1, height=count*150+75)
            self['vpaned1'].set_position(1000)
        else:
            self['connections_expander'].set_size_request(width=-1, height=-1)
            self['vpaned1'].set_position(1000)
            #print "position: %s" % self.view['vpaned1'].get_position()


if __name__ == '__main__':
    from statemachine.states.execution_state import ExecutionState as State
    from statemachine.states.container_state import ContainerState

    from mvc.models import StateModel
    from mvc.views import SingleWidgetWindowView
    from mvc.controllers import StateEditorController, SingleWidgetWindowController

    state1 = State('state2')
    m = StateModel(state1)

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state, gvm_model, emm_model] = main.create_models()

    v = SingleWidgetWindowView(StateEditorView, width=550, height=600, title='State Editor')
    #c = SingleWidgetWindowController(ctr_model, v, StateEditorController)
    c = SingleWidgetWindowController(m, v, StateEditorController)

    gtk.main()