import gtk
from gtkmvc import View


class StateOverviewView(View):
    builder = './glade/state_overview_widget.glade'
    top = 'properties_widget'

    def __init__(self):
        View.__init__(self)


if __name__ == '__main__':
    from statemachine.states.execution_state import ExecutionState as State

    from mvc.models import StateModel
    from mvc.views import SingleWidgetWindowView
    from mvc.controllers import StateOverviewController, SingleWidgetWindowController

    state1 = State('state1')
    m = StateModel(state1)

    v = SingleWidgetWindowView(StateOverviewView, width=500, height=200, title="State Overview")
    c = SingleWidgetWindowController(m, v, StateOverviewController)

    gtk.main()