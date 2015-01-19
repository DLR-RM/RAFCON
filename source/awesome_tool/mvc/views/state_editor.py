import gtk
from gtkmvc import View, Controller
from mvc.views.source_editor import SourceEditorView
from mvc.views.container_state import ContainerStateView
from mvc.views.connections_editor import StateConnectionsEditorView
from mvc.views.state_overview import StateOverviewView


class StateEditorView(View):
    builder = './glade/state_editor_widget.glade'
    top = 'window_state_editor'

    def __init__(self):
        View.__init__(self)

        self['properties_view'] = StateOverviewView()  # StatePropertiesEditorView()
        self['ports_editor_view'] = ContainerStateView()  # StatePortsEditorView()
        self['inputs_view'] = ContainerStateView()  # StateInputsEditorView()
        self['outputs_view'] = ContainerStateView()  # StateOutputsEditorView()
        self['scopes_view'] = ContainerStateView()  # StateScopesEditorView()
        self['outcomes_view'] = ContainerStateView()  # StateOutcomesEditorView()
        self['source_view'] = SourceEditorView()
        self['connections_view'] = StateConnectionsEditorView()

        self['properties_viewport'].add(self['properties_view'].get_top_widget())
        self['viewport3'].add(self['inputs_view']['vbox1'])
        self['viewport4'].add(self['outputs_view']['vbox1'])
        self['viewport5'].add(self['scopes_view']['vbox1'])
        self['viewport6'].add(self['outcomes_view']['vbox1'])
        self['viewport1'].add(self['source_view'].get_top_widget())
        self['alignment1'].add(self['connections_view']['connections_editor_widget'])
        #self['expander3_ScriptEditor'].add(self['source_editor_view'].get_top_widget())
        #self['expander4_ConnectionEditor'].add(self['state_connections_editor_view']['vbox1'])

        self.get_top_widget().resize(width=550, height=550)
        self.get_top_widget().show_all()


if __name__ == '__main__':
    from statemachine.states.execution_state import ExecutionState as State
    from statemachine.states.container_state import ContainerState

    from mvc.models import StateModel, ContainerStateModel
    #from mvc.views.source_editor import SourceEditorView
    from mvc.controllers.state_editor import StateEditorController

    state1 = State('Rico2')
    m = StateModel(state1)

    #w = gtk.Window()
    v = StateEditorView()
    c = StateEditorController(m, v)
    #w.add(v.get_top_widget())#['main_frame'])
    #w.show_all()

    gtk.main()