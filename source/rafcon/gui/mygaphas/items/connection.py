# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from weakref import ref

from rafcon.gui.config import global_gui_config as gui_config
from rafcon.gui.models.data_flow import DataFlowModel
from rafcon.gui.models.transition import TransitionModel
from rafcon.gui.mygaphas.items.line import PerpLine
from rafcon.gui.mygaphas.utils import gap_draw_helper
from rafcon.gui.runtime_config import global_runtime_config


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

    def remove(self):
        self.remove_connection_from_ports()
        super(ConnectionView, self).remove()


class ConnectionPlaceholderView(ConnectionView):
    pass


class TransitionPlaceholderView(ConnectionPlaceholderView):
    def __init__(self, hierarchy_level):
        super(TransitionPlaceholderView, self).__init__(hierarchy_level)
        self._line_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['TRANSITION_LINE'])
        self._arrow_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['LABEL'])


class DataFlowPlaceholderView(ConnectionPlaceholderView):
    def __init__(self, hierarchy_level):
        super(DataFlowPlaceholderView, self).__init__(hierarchy_level)
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
        # Do not draw if the core element has already been destroyed
        if not self.model.core_element:
            return

        if context.selected:
            self._line_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['TRANSITION_LINE_SELECTED'],
                                                            self.parent.transparency)
        else:
            self._line_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['TRANSITION_LINE'],
                                                            self.parent.transparency)
        self._arrow_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['LABEL'], self.parent.transparency)
        super(TransitionView, self).draw(context)

    def apply_meta_data(self):
        # 1st remove all waypoints
        self.remove_all_waypoints()

        # 2nd recreate all waypoints from meta data
        for waypoint_pos in self.model.get_meta_data_editor()['waypoints']:
            self.add_waypoint(waypoint_pos)


class DataFlowView(ConnectionView):
    def __init__(self, data_flow_m, hierarchy_level):
        super(DataFlowView, self).__init__(hierarchy_level)
        assert isinstance(data_flow_m, DataFlowModel)
        self._data_flow_m = None
        self.model = data_flow_m

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
        return global_runtime_config.get_config_value("SHOW_DATA_FLOWS", True)

    def draw(self, context):
        # Do not draw if the core element has already been destroyed
        if not self.model.core_element:
            return

        if not self.show_connection:
            return
        if context.selected:
            self._line_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['DATA_LINE_SELECTED'])
        else:
            self._line_color = gap_draw_helper.get_col_rgba(gui_config.gtk_colors['DATA_LINE'])
        super(DataFlowView, self).draw(context)
