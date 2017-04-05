from rafcon.gui.models.signals import MetaSignalMsg
from rafcon.gui.config import global_gui_config
from rafcon.utils import log


logger = log.get_logger(__name__)


def add_pos(pos1, pos2):
    return pos1[0] + pos2[0], pos1[1] + pos2[1]


def subtract_pos(pos1, pos2):
    return pos1[0] - pos2[0], pos1[1] - pos2[1]


def insert_self_transition_meta_data(state_m, t_id, origin='graphical_editor', combined_action=False):

    try:
        gaphas_editor = global_gui_config.get_config_value('GAPHAS_EDITOR', True)
        y_axis_mirror = 1 if gaphas_editor else -1
        state_meta = state_m.get_meta_data_editor(for_gaphas=gaphas_editor)

        if 'rel_pos' not in state_meta or 'size' not in state_meta:
            return

        transition_m = state_m.parent.get_transition_m(t_id)
        margin = min(state_meta['size']) / 10.
        first_point_x = state_meta['rel_pos'][0] + state_meta['size'][0] + margin
        first_point_y = state_meta['rel_pos'][1] - y_axis_mirror * margin
        second_point_x = state_meta['rel_pos'][0] - margin
        second_point_y = state_meta['rel_pos'][1] - y_axis_mirror * margin

        waypoints = [(first_point_x, first_point_y), (second_point_x, second_point_y)]
        transition_m.set_meta_data_editor('waypoints', waypoints, from_gaphas=gaphas_editor)

        if combined_action:
            transition_m.meta_signal.emit(MetaSignalMsg(origin=origin, change='append_to_last_change'))
        else:
            transition_m.meta_signal.emit(MetaSignalMsg(origin=origin, change='viapoint_position'))
    except TypeError:
        # meta data generation currently only supported for OpenGL editor
        pass


def get_boundaries_of_elements_in_dict(models_dict, parent_size):
    """ Get boundaries of all handed models
    :param models_dict: dict of all handed models
    :param parent_size: size of the parent so maximum relative positions of elements
    :return: tuple of left, right, top and bottom value
    """

    gaphas_editor = True if global_gui_config.get_config_value('GAPHAS_EDITOR') else False
    y_axis_mirror = 1 if gaphas_editor else -1

    # Determine outer coordinates of elements that are to be grouped
    # Use borders of the parent state as initial coordinates
    left = parent_size[0]
    right = 0.0
    top = parent_size[1]
    bottom = 0.0

    # Update outer coordinates regarding all states
    for child_state_m in models_dict['states'].itervalues():
        rel_pos = child_state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']
        size = child_state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['size']
        left = min(rel_pos[0], left)
        right = max(rel_pos[0] + size[0], right)
        top = min(y_axis_mirror * rel_pos[1], top)
        bottom = max(y_axis_mirror * rel_pos[1] + size[1], bottom)

    # Update outer coordinates regarding all scoped variables
    if not gaphas_editor:
        for scoped_variable_m in models_dict['scoped_variables'].itervalues():
            rel_pos = scoped_variable_m.get_meta_data_editor(for_gaphas=gaphas_editor)['inner_rel_pos']
            left = min(rel_pos[0], left)
            right = max(rel_pos[0], right)
            top = min(y_axis_mirror * rel_pos[1], top)
            bottom = max(y_axis_mirror * rel_pos[1], bottom)

    # Update outer coordinates regarding all connections (waypoints)
    connection_models = models_dict['transitions'].values() + models_dict['data_flows'].values()
    for connection_m in connection_models:
        waypoints = connection_m.get_meta_data_editor(for_gaphas=gaphas_editor)['waypoints']
        for waypoint in waypoints:
            left = min(waypoint[0], left)
            right = max(waypoint[0], right)
            top = min(y_axis_mirror * waypoint[1], top)
            bottom = max(y_axis_mirror * waypoint[1], bottom)

    return left, right, top, bottom


def cal_frame_according_boundaries(left, right, top, bottom, parent_size, gaphas_editor):
    y_axis_mirror = 1 if gaphas_editor else -1

    margin = min(parent_size) / 20.
    # Add margin and ensure that the upper left corner is within the state
    rel_pos = max(left - margin, 0), y_axis_mirror * max(top - margin, 0)
    # Add margin and ensure that the lower right corner is within the state
    size = (min(right - left + 2 * margin, parent_size[0] - rel_pos[0]),
            min(bottom - top + 2 * margin, parent_size[1] - y_axis_mirror * rel_pos[1]))

    return margin, rel_pos, size


def offset_rel_pos_of_all_models_in_dict(models_dict, pos_offset, gaphas_editor):
    # Update relative position of states within the container in order to maintain their absolute position
    for child_state_m in models_dict['states'].itervalues():
        old_rel_pos = child_state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']
        child_state_m.set_meta_data_editor('rel_pos', add_pos(old_rel_pos, pos_offset), from_gaphas=gaphas_editor)

    # Do the same for scoped variable
    if not gaphas_editor:
        for scoped_variable_m in models_dict['scoped_variables'].itervalues():
            old_rel_pos = scoped_variable_m.get_meta_data_editor(for_gaphas=gaphas_editor)['inner_rel_pos']
            scoped_variable_m.set_meta_data_editor('inner_rel_pos', add_pos(old_rel_pos, pos_offset), gaphas_editor)

    # Do the same for all connections (transitions and data flows)
    connection_models = models_dict['transitions'].values() + models_dict['data_flows'].values()
    for connection_m in connection_models:
        old_waypoints = connection_m.get_meta_data_editor(for_gaphas=gaphas_editor)['waypoints']
        new_waypoints = []
        for waypoint in old_waypoints:
            new_waypoints.append(add_pos(waypoint, pos_offset))
        connection_m.set_meta_data_editor('waypoints', new_waypoints, from_gaphas=gaphas_editor)


def offset_rel_pos_of_models_meta_data_according_parent_state(models_dict):
    """ Offset meta data of state elements according the area used indicated by the state meta data.

    The offset_rel_pos_of_models_meta_data_according_parent_state offset the position of all handed old elements
    in the dictionary.

    :param models_dict: dict that hold lists of meta data with state attribute consistent keys
    :return:
    """
    gaphas_editor = True if global_gui_config.get_config_value('GAPHAS_EDITOR') else False
    old_parent_rel_pos = models_dict['state'].get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']
    offset = old_parent_rel_pos
    offset_rel_pos_of_all_models_in_dict(models_dict, offset, gaphas_editor)

    return True


def scale_meta_data_according_states(models_dict):
    """ Offset meta data of state elements according the area used indicated by the states and
    maybe scoped variables (in case of OpenGL editor) meta data.

    Method is used by group states to set the offset for the elements in the new container state.
    The method needs some generalisation to create methods to easily scale meta data according new parents or views
    (e.g. to show inner elements of s library state).

    :param models_dict: dictionary that hold lists of meta data with state attribute consistent keys
    :return:
    """
    gaphas_editor = True if global_gui_config.get_config_value('GAPHAS_EDITOR') else False

    parent_size = models_dict['state'].parent.get_meta_data_editor(for_gaphas=gaphas_editor)['size']

    left, right, top, bottom = get_boundaries_of_elements_in_dict(models_dict=models_dict, parent_size=parent_size)

    _, rel_pos, size = cal_frame_according_boundaries(left, right, top, bottom, parent_size, gaphas_editor)

    # Set size and position of new container state
    models_dict['state'].set_meta_data_editor('rel_pos', rel_pos, from_gaphas=gaphas_editor)
    models_dict['state'].set_meta_data_editor('size', size, from_gaphas=gaphas_editor)
    offset = (-rel_pos[0], -rel_pos[1])
    offset_rel_pos_of_all_models_in_dict(models_dict, offset, gaphas_editor)

    return True
