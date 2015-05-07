from weakref import ref

from gaphas.connector import PointPort, Handle
from gaphas.util import path_ellipse

from awesome_tool.mvc.models.outcome import OutcomeModel
from awesome_tool.mvc.models.data_port import DataPortModel


class PortView(object):

    def __init__(self):
        self.handle = Handle(connectable=True, movable=False)
        self.port = PointPort(self.handle.pos)

    @property
    def pos(self):
        return self.handle.pos

    def draw(self, context, state):
        raise NotImplementedError


class IncomeView(PortView):

    def __init__(self):
        super(IncomeView, self).__init__()

    def draw(self, context, state):
        c = context.cairo
        min_state_side = min(state.width, state.height)
        outcome_side = min_state_side / 20.
        path_ellipse(c, self.pos.x, self.pos.y, outcome_side, outcome_side)


class OutcomeView(PortView):

    def __init__(self, outcome_m):
        super(OutcomeView, self).__init__()

        assert isinstance(outcome_m, OutcomeModel)
        self._outcome_m = ref(outcome_m)
        self.sort = outcome_m.outcome.outcome_id

    @property
    def outcome_m(self):
        return self._outcome_m()

    @property
    def outcome_id(self):
        return self.outcome_m.outcome.outcome_id

    def draw(self, context, state):
        c = context.cairo
        min_state_side = min(state.width, state.height)
        outcome_side = min_state_side / 20.
        c.set_line_width(0.25)
        # c.rectangle(self.pos.x - outcome_side / 2, self.pos.y - outcome_side / 2, outcome_side, outcome_side)
        path_ellipse(c, self.pos.x, self.pos.y, outcome_side, outcome_side)


class DataPortView(PortView):

    def __init__(self, port_m):
        super(DataPortView, self).__init__()

        assert isinstance(port_m, DataPortModel)
        self._port_m = ref(port_m)
        self.sort = port_m.data_port.data_port_id

    @property
    def port_m(self):
        return self._port_m()

    @property
    def port_id(self):
        return self.port_m.data_port.data_port_id

    def draw(self, context, state):
        c = context.cairo
        min_state_side = min(state.width, state.height)
        port_side = min_state_side / 25.
        c.set_line_width(0.15)
        c.rectangle(self.pos.x - port_side / 2, self.pos.y - port_side / 2, port_side, port_side)


class InputPortView(DataPortView):
    pass


class OutputPortView(DataPortView):
    pass