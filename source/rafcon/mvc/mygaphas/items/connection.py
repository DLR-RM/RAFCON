from weakref import ref

from rafcon.mvc.config import global_gui_config as gui_config
from rafcon.mvc.models.data_flow import DataFlowModel
from rafcon.mvc.models.transition import TransitionModel
from rafcon.mvc.mygaphas.items.line import PerpLine
from rafcon.mvc.mygaphas.utils import gap_draw_helper
from rafcon.mvc.runtime_config import global_runtime_config


class ConnectionView(PerpLine):
    def set_port_for_handle(self, port, handle):
        if handle is self.from_handle():
            self.from_port = port
        elif handle is self.to_handle():
            self.to_port = port

    def reset_port_for_handle(self, handle):
        if handle is self.from_handle():
            self.reset_from_port()
        elif handle is self.to_handle():
            self.reset_to_port()

    def remove_connection_from_port(self, port):
        if self._from_port and port is self._from_port:
            self._from_port.remove_connected_handle(self._from_handle)
        elif self._to_port and port is self._to_port:
            self._to_port.remove_connected_handle(self._to_handle)

    def remove_connection_from_ports(self):
        if self._from_port:
            self._from_port.remove_connected_handle(self._from_handle)
            self._from_port.tmp_disconnect()
        if self._to_port:
            self._to_port.remove_connected_handle(self._to_handle)
            self.to_port.tmp_disconnect()

    def prepare_destruction(self):
        super(ConnectionView, self).prepare_destruction()
        self.remove_connection_from_ports()


class ConnectionPlaceholderView(ConnectionView):
    def __init__(self, hierarchy_level, transition_placeholder):
        super(ConnectionPlaceholderView, self).__init__(hierarchy_level)

        self.transition_placeholder = transition_placeholder

        if transition_placeholder:
            self._line_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['TRANSITION_LINE'])
            self._arrow_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['LABEL'])
        else:
            self._line_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['DATA_LINE'])
            self._arrow_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['DATA_PORT'])


class TransitionView(ConnectionView):
    def __init__(self, transition_m, hierarchy_level):
        super(TransitionView, self).__init__(hierarchy_level)
        self._transition_m = None
        self.model = transition_m

    @property
    def model(self):
        return self._transition_m()

    @model.setter
    def model(self, transition_model):
        assert isinstance(transition_model, TransitionModel)
        self._transition_m = ref(transition_model)

    def draw(self, context):
        if context.selected:
            self._line_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['TRANSITION_LINE_SELECTED'],
                                                            self.parent.transparent)
        else:
            self._line_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['TRANSITION_LINE'],
                                                            self.parent.transparent)
        self._arrow_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['LABEL'], self.parent.transparent)
        super(TransitionView, self).draw(context)

    def apply_meta_data(self):
        # 1st remove all waypoints
        self.remove_all_waypoints()

        # 2nd recreate all waypoints from meta data
        for waypoint_pos in self.model.meta['gui']['editor_gaphas']['waypoints']:
            self.add_waypoint(waypoint_pos)


class DataFlowView(ConnectionView):
    def __init__(self, data_flow_m, hierarchy_level):
        super(DataFlowView, self).__init__(hierarchy_level)
        assert isinstance(data_flow_m, DataFlowModel)
        self._data_flow_m = None
        self.model = data_flow_m

        self._show = global_runtime_config.get_config_value("SHOW_DATA_FLOWS", True)

        self._line_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['DATA_LINE'])
        self._arrow_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['DATA_PORT'])

    @property
    def model(self):
        return self._data_flow_m()

    @model.setter
    def model(self, data_flow_m):
        assert isinstance(data_flow_m, DataFlowModel)
        self._data_flow_m = ref(data_flow_m)

    @property
    def show_connection(self):
        return global_runtime_config.get_config_value("SHOW_DATA_FLOWS", True) or self._show

    def show(self):
        self._show = True

    def hide(self):
        self._show = False

    def draw(self, context):
        if not self.show_connection:
            return
        if context.selected:
            self._line_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['DATA_LINE_SELECTED'])
        else:
            self._line_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['DATA_LINE'])
        super(DataFlowView, self).draw(context)
