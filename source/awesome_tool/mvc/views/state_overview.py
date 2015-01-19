import gtk
from gtkmvc import View, Controller


class StateOverviewView(View):
    builder = './glade/state_overview_widget.glade'
    top = 'properties_widget'

    def __init__(self):
        View.__init__(self)


if __name__ == '__main__':
    from statemachine.states.execution_state import ExecutionState as State
    from statemachine.states.container_state import ContainerState

    from mvc.models import StateModel, ContainerStateModel
    #from mvc.views.source_editor import SourceEditorView
    from mvc.controllers.state_overview import StateOverviewController

    state1 = State('state1')
    m = StateModel(state1)

    w = gtk.Window()
    v = StateOverviewView()
    c = StateOverviewController(m, v)
    w.add(v.get_top_widget())#['main_frame'])
    w.show_all()

    gtk.main()