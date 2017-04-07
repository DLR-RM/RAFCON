from rafcon.gui.models.signals import MetaSignalMsg
from rafcon.gui.config import global_gui_config
from rafcon.gui.utils import constants
from rafcon.utils import log


logger = log.get_logger(__name__)


def add_pos(pos1, pos2):
    return pos1[0] + pos2[0], pos1[1] + pos2[1]


def subtract_pos(pos1, pos2):
    return pos1[0] - pos2[0], pos1[1] - pos2[1]


def mult_two_vectors(vec1, vec2):
    return tuple([vec1[i] * vec2[i] for i in range(len(vec2))])


def divide_two_vectors(vec1, vec2):
    return tuple([vec1[i] / vec2[i] for i in range(len(vec2))])


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


def get_boundaries_of_elements_in_dict(models_dict):
    """ Get boundaries of all handed models
    :param models_dict: dict of all handed models
    :return: tuple of left, right, top and bottom value
    """

    gaphas_editor = True if global_gui_config.get_config_value('GAPHAS_EDITOR') else False
    y_axis_mirror = 1 if gaphas_editor else -1

    # Determine outer coordinates of elements in models_dict -> states, scoped variables and transitions are relevant
    right = 0.
    bottom = 0.
    if 'states' in models_dict and models_dict['states']:
        left = models_dict['states'].items()[0][1].get_meta_data_editor(gaphas_editor)['rel_pos'][0]
    else:
        left = models_dict['scoped_variables'].items()[0][1].get_meta_data_editor(gaphas_editor)['inner_rel_pos'][0]
    if models_dict['states']:
        top = y_axis_mirror * models_dict['states'].items()[0][1].get_meta_data_editor(gaphas_editor)['rel_pos'][1]
    else:
        top = y_axis_mirror * models_dict['scoped_variables'].items()[0][1].get_meta_data_editor(gaphas_editor)['inner_rel_pos'][1]

    def cal_max(max_x, max_y, rel_pos, size):
        max_x = size[0] + rel_pos[0] if size[0] + rel_pos[0] > max_x else max_x
        max_y = y_axis_mirror * rel_pos[1] + size[1] if y_axis_mirror * rel_pos[1] + size[1] > max_y else max_y
        return max_x, max_y

    def cal_min(min_x, min_y, rel_pos, size):
        min_x = rel_pos[0] if rel_pos[0] < min_x else min_x
        min_y = y_axis_mirror * rel_pos[1] if y_axis_mirror * rel_pos[1] < min_y else min_y
        return min_x, min_y

    parts = ['states', 'transitions', 'data_flows']
    if not gaphas_editor:
        parts.append('scoped_variables')
    for key in parts:
        elems_dict = models_dict[key]
        rel_positions = []
        for model in elems_dict.itervalues():
            _size = (0., 0.)
            if key == 'states':
                rel_positions = [model.get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']]
                _size = model.get_meta_data_editor(for_gaphas=gaphas_editor)['size']
                # print "states", rel_positions, _size
            elif key == 'scoped_variables':
                rel_positions = [model.get_meta_data_editor(for_gaphas=gaphas_editor)['inner_rel_pos']]
            elif key in ['transitions', 'data_flows']:
                rel_positions = model.get_meta_data_editor(for_gaphas=gaphas_editor)['waypoints']

            for rel_position in rel_positions:
                right, bottom = cal_max(right, bottom, rel_position, _size)
                left, top = cal_min(left, top, rel_position, _size)

    return left, right, top, bottom


def cal_frame_according_boundaries(left, right, top, bottom, parent_size, gaphas_editor, group=True):
    y_axis_mirror = 1 if gaphas_editor else -1

    margin = min(parent_size[0], parent_size[1]) / constants.BORDER_WIDTH_STATE_SIZE_FACTOR
    # Add margin and ensure that the upper left corner is within the state
    if group:
        # frame of grouped state
        rel_pos = max(left - margin, 0), y_axis_mirror * max(top - margin, 0)
        # Add margin and ensure that the lower right corner is within the state
        size = (min(right - left + 2 * margin, parent_size[0] - rel_pos[0]),
                min(bottom - top + 2 * margin, parent_size[1] - y_axis_mirror * rel_pos[1]))
    else:
        # frame inside of state
        # rel_pos = max(margin, 0), y_axis_mirror * max(margin, 0)
        rel_pos = left, top
        size = right - left, bottom - top

    return margin, rel_pos, size


def offset_rel_pos_of_all_models_in_dict(models_dict, pos_offset, gaphas_editor):
    print "\n", "#"*30, "offset models", pos_offset, "#"*30
    # Update relative position of states within the container in order to maintain their absolute position
    for child_state_m in models_dict['states'].itervalues():
        old_rel_pos = child_state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']
        print "old_rel_pos", old_rel_pos, child_state_m
        child_state_m.set_meta_data_editor('rel_pos', add_pos(old_rel_pos, pos_offset), from_gaphas=gaphas_editor)
        print "new_rel_pos", child_state_m.get_meta_data_editor(for_gaphas=gaphas_editor), child_state_m

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
    print "END", "#"*30, "offset models", pos_offset, "#"*30, "\n"


def resize_state_meta(state_m, factor, gaphas_editor):
    # TODO substitute this with method in models that also include ports, outcomes, child states & inner connections
    # print "START RESIZE OF STATE", state_m.get_meta_data_editor(for_gaphas=gaphas_editor), state_m
    old_size = state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['size']
    # print old_size, type(old_size)
    state_m.set_meta_data_editor('size', mult_two_vectors(factor, old_size), from_gaphas=gaphas_editor)
    # print "new_size", state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['size']
    if gaphas_editor:
        old_rel_pos = state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['income']['rel_pos']
        state_m.set_meta_data_editor('income.rel_pos', mult_two_vectors(factor, old_rel_pos), from_gaphas=gaphas_editor)
        old_rel_pos = state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['name']['rel_pos']
        state_m.set_meta_data_editor('name.rel_pos', mult_two_vectors(factor, old_rel_pos), from_gaphas=gaphas_editor)
        old_size = state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['name']['size']
        state_m.set_meta_data_editor('name.size', mult_two_vectors(factor, old_size), from_gaphas=gaphas_editor)


def resize_of_all_models_in_dict(models_dict, factor, gaphas_editor):
    print "\n", "#"*30, "resize models", factor, "#"*30,

    # Update relative position of states within the container in order to maintain their absolute position
    for child_state_m in models_dict['states'].itervalues():
        old_rel_pos = child_state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']
        print "old_rel_pos state", old_rel_pos, child_state_m.core_element
        child_state_m.set_meta_data_editor('rel_pos', mult_two_vectors(factor, old_rel_pos), from_gaphas=gaphas_editor)
        print "new_rel_pos state", child_state_m.get_meta_data_editor(for_gaphas=gaphas_editor), child_state_m.core_element
        resize_state_meta(child_state_m, factor, gaphas_editor)
        print "re-sized state", child_state_m.get_meta_data_editor(for_gaphas=gaphas_editor), child_state_m.core_element

    # Do the same for scoped variable
    if not gaphas_editor:
        for scoped_variable_m in models_dict['scoped_variables'].itervalues():
            old_rel_pos = scoped_variable_m.get_meta_data_editor(for_gaphas=gaphas_editor)['inner_rel_pos']
            scoped_variable_m.set_meta_data_editor('inner_rel_pos', mult_two_vectors(factor, old_rel_pos), gaphas_editor)

    # Do the same for all connections (transitions and data flows)
    connection_models = models_dict['transitions'].values() + models_dict['data_flows'].values()
    for connection_m in connection_models:
        print "old_waypoints", connection_m.get_meta_data_editor(for_gaphas=gaphas_editor), connection_m.core_element
        old_waypoints = connection_m.get_meta_data_editor(for_gaphas=gaphas_editor)['waypoints']
        new_waypoints = []
        for waypoint in old_waypoints:
            new_waypoints.append(mult_two_vectors(factor, waypoint))
        connection_m.set_meta_data_editor('waypoints', new_waypoints, from_gaphas=gaphas_editor)
        print "new_waypoints", connection_m.get_meta_data_editor(for_gaphas=gaphas_editor), connection_m.core_element
    print "END", "#"*30, "resize models", factor, "#"*30, "\n"


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

    left, right, top, bottom = get_boundaries_of_elements_in_dict(models_dict=models_dict)

    parent_size = models_dict['state'].parent.get_meta_data_editor(for_gaphas=gaphas_editor)['size']
    _, rel_pos, size = cal_frame_according_boundaries(left, right, top, bottom, parent_size, gaphas_editor)

    # Set size and position of new container state
    models_dict['state'].set_meta_data_editor('rel_pos', rel_pos, from_gaphas=gaphas_editor)
    models_dict['state'].set_meta_data_editor('size', size, from_gaphas=gaphas_editor)
    offset = mult_two_vectors((-1., -1.), rel_pos)
    offset_rel_pos_of_all_models_in_dict(models_dict, offset, gaphas_editor)

    return True


def scale_meta_data_according_state(models_dict, rel_pos=None):
    # TODO Documentation needed....
    """ Define a
    """

    gaphas_editor = True if global_gui_config.get_config_value('GAPHAS_EDITOR') else False
    y_axis_mirror = 1. if gaphas_editor else -1.

    if 'states' in models_dict or 'scoped_variables' in models_dict:
        left, right, top, bottom = get_boundaries_of_elements_in_dict(models_dict=models_dict)
        parent_size = models_dict['state'].get_meta_data_editor(for_gaphas=gaphas_editor)['size']
        margin, old_rel_pos, size = cal_frame_according_boundaries(left, right, top, bottom, parent_size,
                                                                   gaphas_editor, False)
        automatic_mode = True if rel_pos is None else False
        rel_pos = (margin, margin) if rel_pos is None else rel_pos
        assert parent_size[0] > rel_pos[0]
        assert parent_size[1] > rel_pos[1]
        print "edges:", left, right, top, bottom
        print "margin:", margin, "rel_pos:", rel_pos, "old_rel_pos", old_rel_pos, "size:", size

        parent_width, parent_height = parent_size

        boundary_width, boundary_height = size
        print "parent width:   {0}, parent_height:   {1}".format(parent_width, parent_height)
        print "boundary width: {0}, boundary_height: {1}".format(boundary_width, boundary_height)

        # no site scale
        if parent_width - rel_pos[0] > boundary_width and parent_height - rel_pos[1] > boundary_height:
            if automatic_mode:
                resize_factor = 1.
                boundary_width_in_parent = boundary_width*resize_factor
                print boundary_width, resize_factor, boundary_width*resize_factor, boundary_height*resize_factor + margin*2, parent_height
                print "left over width: ", parent_width - boundary_width_in_parent - rel_pos[0] - margin
                width_pos_offset_to_middle = (parent_width - boundary_width_in_parent - rel_pos[0] - margin)/2.
                rel_pos = add_pos(rel_pos, (width_pos_offset_to_middle, 0.))
                boundary_height_in_parent = boundary_height*resize_factor
                height_pos_offset_to_middle = (parent_height - boundary_height_in_parent - rel_pos[1] - margin)/2.
                rel_pos = add_pos(rel_pos, (0., height_pos_offset_to_middle))
            offset = subtract_pos((0., 0.), subtract_pos(old_rel_pos, rel_pos))
            offset_rel_pos_of_all_models_in_dict(models_dict, offset, gaphas_editor)
        # smallest site scale
        else:
            if (parent_height - rel_pos[1] - margin)/boundary_height < \
                    (parent_width - rel_pos[0] - margin)/boundary_width:
                # print "#"*20, 1, "#"*20, rel_pos
                resize_factor = (parent_height - rel_pos[1] - margin)/boundary_height
                boundary_width_in_parent = boundary_width*resize_factor
                print boundary_width, resize_factor, boundary_width*resize_factor, boundary_height*resize_factor + margin*2, parent_height
                print "left over width: ", parent_width - boundary_width_in_parent - rel_pos[0] - margin
                width_pos_offset_to_middle = (parent_width - boundary_width_in_parent - rel_pos[0] - margin)/2.
                rel_pos = add_pos(rel_pos, (width_pos_offset_to_middle, 0.))
                print resize_factor, rel_pos
            else:
                # print "#"*20, 2, "#"*20, rel_pos
                resize_factor = (parent_width - rel_pos[0] - margin)/boundary_width
                boundary_height_in_parent = boundary_height*resize_factor
                height_pos_offset_to_middle = (parent_height - boundary_height_in_parent - rel_pos[1] - margin)/2.
                rel_pos = add_pos(rel_pos, (0., height_pos_offset_to_middle))
                # print resize_factor, rel_pos
            frame = {'rel_pos': rel_pos, 'size': mult_two_vectors((resize_factor, resize_factor), size)}
            print models_dict['state'].get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos'], \
                parent_size, models_dict['state'].parent.get_meta_data_editor(for_gaphas=gaphas_editor)['size']
            print "frame", frame, resize_factor
            # rel_pos = mult_two_vectors((1, -1), frame['rel_pos'])
            offset = subtract_pos((0., 0.), mult_two_vectors((1., y_axis_mirror), old_rel_pos))
            offset_rel_pos_of_all_models_in_dict(models_dict, offset, gaphas_editor)

            resize_of_all_models_in_dict(models_dict, (resize_factor, resize_factor), gaphas_editor)

            offset_rel_pos_of_all_models_in_dict(models_dict, mult_two_vectors((1., y_axis_mirror), frame['rel_pos']), gaphas_editor)
            # scale_meta_data_according_frame(models_dict, frame)


def scale_meta_data_according_frame(models_dict, frame):
    # TODO Documentation needed....
    """

    :param models_dict: dictionary that hold lists of meta data with state attribute consistent keys
    :return:
    """
    # TODO check if this is working
    gaphas_editor = True if global_gui_config.get_config_value('GAPHAS_EDITOR') else False
    y_axis_mirror = 1. if gaphas_editor else -1.

    left, right, top, bottom = get_boundaries_of_elements_in_dict(models_dict=models_dict)
    margin, old_rel_pos, size = cal_frame_according_boundaries(left, right, top, bottom, (0., 0.), gaphas_editor, False)
    old_frame = {'rel_pos': old_rel_pos, 'size': size}

    # cal offset and resize factor in between
    offset = subtract_pos((0., 0.), old_frame['rel_pos'])
    resize_factor = divide_two_vectors(frame['size'], old_frame['size'])

    offset_rel_pos_of_all_models_in_dict(models_dict, offset, gaphas_editor)
    resize_of_all_models_in_dict(models_dict, (resize_factor, resize_factor), gaphas_editor)
    offset_rel_pos_of_all_models_in_dict(models_dict, mult_two_vectors((1., y_axis_mirror), frame['rel_pos']), gaphas_editor)
    return True

# Something to remember maybe
#
# The function is used at the moment by the ungroup method of the ContainerStateModel, only.
# It is supposed to be use for a meta data set of a state not the StateModel it self to scale (reduce) the size of
# scoped_variables, states and transition and data_flows. New- and old-state models, or new- and old-state element
# models can be different. So the models the data is taken from can be different then the models the data is written
# on it depends how the dictionary was filled before.
#
# The method needs some generalisation to create methods to easily scale meta data according new parents or views
# (e.g. to show inner elements of a library state).
# It also maybe should scale it if the size has changed according to the minimal extension (x or y) of the handed
# size. Also it is of interest to have extension flag if the scaling should scale according the proportion
# of a parent state (the handed state) or should keep the previous element x-y-ratios.
# To keep the previous meta data proportion would be different then in the graphical editors.
