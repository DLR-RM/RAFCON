# Copyright (C) 2017-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>

from copy import deepcopy

import json
from jsonconversion.decoder import JSONObjectDecoder
from jsonconversion.encoder import JSONObjectEncoder

from rafcon.gui.models.transition import mirror_waypoints
from rafcon.gui.models.signals import MetaSignalMsg
from rafcon.gui.models import LibraryStateModel, ContainerStateModel
from rafcon.gui.config import global_gui_config
from rafcon.gui.utils import constants
from rafcon.utils import log


logger = log.get_logger(__name__)

ROOT_STATE_DEFAULT_REL_POS = (0, 0)
STATE_DEFAULT_REL_POS = (10, 10)


def add_pos(pos1, pos2):
    return pos1[0] + pos2[0], pos1[1] + pos2[1]


def subtract_pos(pos1, pos2):
    return pos1[0] - pos2[0], pos1[1] - pos2[1]


def mult_two_vectors(vec1, vec2):
    return tuple([vec1[i] * vec2[i] for i in range(len(vec2))])


def divide_two_vectors(vec1, vec2):
    return tuple([vec1[i] / vec2[i] for i in range(len(vec2))])


def cal_margin(parent_size):
    return min(parent_size[0], parent_size[1]) / constants.BORDER_WIDTH_STATE_SIZE_FACTOR


def contains_geometric_info(var):
    """ Check whether the passed variable is a tuple with two floats or integers """
    return isinstance(var, tuple) and len(var) == 2 and all(isinstance(val, (int, float)) for val in var)


def dict_has_empty_elements(d, ignored_keys=None, ignored_partial_keys=None):
    ignored_keys = ["show_content", "waypoints"] if ignored_keys is None else ignored_keys
    ignored_partial_keys = ['input_data_port', 'output_data_port'] if ignored_partial_keys is None else ignored_partial_keys
    empty = False
    if not d:
        # print "dict check -> result empty", d
        return True
    else:
        for k, v in d.iteritems():
            # print "check", k, " -> ", v
            if isinstance(v, dict):
                if dict_has_empty_elements(v):
                    if k not in ignored_keys and not any([key in k for key in ignored_partial_keys]):
                        empty = True
                        break
                    else:
                        # print "ignore empty dict: ", k
                        pass
            else:
                if isinstance(v, bool):
                    pass
                elif not len(v) > 0:
                    # print k, v
                    if k not in ignored_keys and not any([key in k for key in ignored_partial_keys]):
                        empty = True
                        break
                    else:
                        # print "ignore empty list: ", k
                        pass

    return empty


def model_has_empty_meta(m, ignored_keys=None, ignored_partial_keys=None):
    # print m, m.meta
    if dict_has_empty_elements(m.meta, ignored_keys, ignored_partial_keys):
        # print "XXX", m, m.meta
        return True
    if isinstance(m, ContainerStateModel):
        for state_m in m.states.itervalues():
            if dict_has_empty_elements(state_m.meta, ignored_keys, ignored_partial_keys):
                # print "LXXX", state_m, state_m.meta
                return True
    return False


def get_y_axis_and_gaphas_editor_flag():
    gaphas_editor = True if global_gui_config.get_config_value('GAPHAS_EDITOR') else False
    y_axis_mirror = 1. if gaphas_editor else -1.
    return gaphas_editor, y_axis_mirror


def generate_default_state_meta_data(parent_state_m, canvas=None, num_child_state=None, gaphas_editor=True):
    """Generate default meta data for a child state according its parent state

    The function could work with the handed num_child_state if all child state are not drawn, till now.
    The method checks if the parents meta data is consistent in canvas state view and model if a canvas instance is 
    handed. 

    :param rafcon.gui.models.container_state.ContainerStateModel parent_state_m: Model of the state were the child 
                                                                                 should be drawn into
    :param rafcon.gui.mygaphas.canvas.MyCanvas canvas: canvas instance the state will be drawn into
    :param int num_child_state: Number of child states already drawn in canvas parent state view
    :return child relative pos (tuple) in parent and it size (tuple)
    """
    parent_size = parent_state_m.get_meta_data_editor(gaphas_editor)['size']
    if not contains_geometric_info(parent_size):
        raise ValueError("Invalid state size: {}".format(parent_size))

    # use handed number of child states and otherwise take number of child states from parent state model
    num_child_state = len(parent_state_m.states) if num_child_state is None else num_child_state

    # check if respective canvas is handed if meta data of parent canvas view is consistent with model
    if canvas is not None:
        parent_state_v = canvas.get_view_for_model(parent_state_m)
        if not (parent_state_v.width, parent_state_v.height) == parent_size:
            logger.warning("Size meta data of model {0} is different to gaphas {1}"
                           "".format((parent_state_v.width, parent_state_v.height), parent_size))

    # Calculate default positions for the child states
    # Make the inset from the top left corner
    parent_state_width, parent_state_height = parent_size
    new_state_side_size = min(parent_state_width * 0.2, parent_state_height * 0.2)

    child_width = new_state_side_size
    child_height = new_state_side_size
    child_size = (child_width, child_height)
    child_spacing = max(child_size) * 1.2

    parent_margin = cal_margin(parent_size)
    # print "parent size", parent_size, parent_margin
    max_cols = (parent_state_width - 2*parent_margin) // child_spacing
    (row, col) = divmod(num_child_state, max_cols)

    max_rows = (parent_state_height - 2*parent_margin - 0.5*child_spacing) // (1.5*child_spacing)
    (overlapping, row) = divmod(row, max_rows)

    overlapping_step = 0.5*parent_margin
    max_overlaps_x = (parent_state_width - 2*parent_margin - child_width -
                      (parent_margin + (max_cols - 1) * child_spacing + child_spacing - child_width)) // overlapping_step
    max_overlaps_y = (parent_state_height - 2*parent_margin - child_height -
                      child_spacing * (1.5 * (max_rows - 1) + 1)) // overlapping_step
    # handle case of less space TODO check again not perfect, maybe that can be done more simple
    max_overlaps_x = 0 if max_overlaps_x < 0 else max_overlaps_x
    max_overlaps_y = 0 if max_overlaps_y < 0 else max_overlaps_y
    max_overlaps = min(max_overlaps_x, max_overlaps_y) + 1
    overlapping = divmod(overlapping, max_overlaps)[1]

    child_rel_pos_x = parent_margin + col * child_spacing + child_spacing - child_width + overlapping*overlapping_step
    child_rel_pos_y = child_spacing * (1.5 * row + 1.) + overlapping*overlapping_step
    return (child_rel_pos_x, child_rel_pos_y), (new_state_side_size, new_state_side_size)


def put_default_meta_on_state_m(state_m, parent_state_m, num_child_state=None):
    # print state_m, parent_state_m, num_child_state
    gaphas_editor, y_axis_mirror = get_y_axis_and_gaphas_editor_flag()
    rel_pos, size = generate_default_state_meta_data(parent_state_m,
                                                     num_child_state=num_child_state,
                                                     gaphas_editor=gaphas_editor,)
    state_m.set_meta_data_editor('rel_pos', mult_two_vectors((1., y_axis_mirror), rel_pos), gaphas_editor)
    state_m.set_meta_data_editor('size', size, gaphas_editor)


def put_default_meta_data_on_state_m_recursively(state_m, parent_state_m, num_child_state=None, only_child_states=False):

    if not only_child_states:
        put_default_meta_on_state_m(state_m, parent_state_m, num_child_state)

    if isinstance(state_m, ContainerStateModel):
        for child_state_number, child_state_m in enumerate(state_m.states.itervalues()):
            put_default_meta_data_on_state_m_recursively(child_state_m, parent_state_m if only_child_states else state_m,
                                                         num_child_state=child_state_number)
    return True


def insert_self_transition_meta_data(state_m, t_id, origin='graphical_editor', combined_action=False):

    try:
        gaphas_editor = global_gui_config.get_config_value('GAPHAS_EDITOR', True)
        y_axis_mirror = 1 if gaphas_editor else -1
        state_meta = state_m.get_meta_data_editor(for_gaphas=gaphas_editor)

        if 'rel_pos' not in state_meta or 'size' not in state_meta:
            return

        transition_m = state_m.parent.get_transition_m(t_id)
        margin = min(state_meta['size']) / 10.
        if gaphas_editor:
            origin_port_m = state_m.get_outcome_m(transition_m.transition.from_outcome)
            origin_port_x, origin_port_y = origin_port_m.get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']
            target_port_x, target_port_y = state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['income']['rel_pos']
            x_osign = 1 if origin_port_x/state_meta['size'][0] > 0.5 else -1
            x_tsign = -1 if target_port_x/state_meta['size'][0] < 0.5 else 1
            y_osign = 1 if origin_port_y/state_meta['size'][1] > 0.5 else -1
            y_tsign = -1 if target_port_y/state_meta['size'][1] < 0.5 else 1
            first_point_x = state_meta['rel_pos'][0] + x_osign*margin + (state_meta['size'][0] if x_osign == 1 else 0.)
            first_point_y = state_meta['rel_pos'][1] + y_osign*margin + (state_meta['size'][1] if y_osign == 1 else 0.)
            second_point_x = state_meta['rel_pos'][0] + x_tsign*margin + (state_meta['size'][0] if x_tsign == 1 else 0.)
            second_point_y = state_meta['rel_pos'][1] + y_tsign*margin + (state_meta['size'][1] if y_tsign == 1 else 0.)
        else:
            first_point_x = state_meta['rel_pos'][0] + state_meta['size'][0] + margin
            first_point_y = state_meta['rel_pos'][1] - y_axis_mirror * margin
            second_point_x = state_meta['rel_pos'][0] - margin
            second_point_y = state_meta['rel_pos'][1] - y_axis_mirror * margin

        if gaphas_editor and (first_point_x, first_point_y) == (second_point_x, second_point_y):
            waypoints = [(first_point_x, first_point_y)]
        else:
            waypoints = [(first_point_x, first_point_y), (second_point_x, second_point_y)]
        transition_m.set_meta_data_editor('waypoints', waypoints, from_gaphas=gaphas_editor)

        if combined_action:
            transition_m.meta_signal.emit(MetaSignalMsg(origin=origin, change='append_to_last_change'))
        else:
            transition_m.meta_signal.emit(MetaSignalMsg(origin=origin, change='viapoint_position'))
    except TypeError:
        # meta data generation currently only supported for OpenGL editor
        pass


def add_boundary_clearance(left, right, top, bottom, frame, clearance=0.1):
    """Increase boundary size

    The function increase the space between top and bottom and between left and right parameters.
    The increase performed on the biggest size/frame so max(size boundary, size frame)

    :param float left: lower x-axis value
    :param float right: upper x-axis value
    :param float top: lower y-axis value
    :param float bottom: upper y-axis value
    :param dict frame: Dictionary with size and rel_pos tuples
    :param float clearance: Percentage 0.01 = 1% of clearance
    :return:
    """

    # print "old boundary", left, right, top, bottom
    width = right - left
    width = frame['size'][0] if width < frame['size'][0] else width
    left -= 0.5 * clearance * width
    left = 0 if left < 0 else left
    right += 0.5 * clearance * width

    height = bottom - top
    height = frame['size'][1] if height < frame['size'][1] else height
    top -= 0.5 * clearance * height
    top = 0 if top < 0 else top
    bottom += 0.5 * clearance * height
    # print "new boundary", left, right, top, bottom
    return left, right, top, bottom


def get_boundaries_of_elements_in_dict(models_dict, clearance=0.):
    """ Get boundaries of all handed models

    The function checks all model meta data positions to increase boundary starting with a state or scoped variables.
    It is finally iterated over all states, data and logical port models and linkage if sufficient for respective
    graphical editor. At the end a clearance is added to the boundary if needed e.g. to secure size for opengl.

    :param models_dict: dict of all handed models
    :return: tuple of left, right, top and bottom value
    """

    gaphas_editor, y_axis_mirror = get_y_axis_and_gaphas_editor_flag()

    # Determine initial outer coordinates
    right = 0.
    bottom = 0.
    if 'states' in models_dict and models_dict['states']:
        left = models_dict['states'].items()[0][1].get_meta_data_editor(gaphas_editor)['rel_pos'][0]
        top = y_axis_mirror * models_dict['states'].items()[0][1].get_meta_data_editor(gaphas_editor)['rel_pos'][1]
    elif 'scoped_variables' in models_dict and models_dict['scoped_variables']:
        left = models_dict['scoped_variables'].items()[0][1].get_meta_data_editor(gaphas_editor)['inner_rel_pos'][0]
        top = y_axis_mirror * models_dict['scoped_variables'].items()[0][1].get_meta_data_editor(gaphas_editor)['inner_rel_pos'][1]
    else:
        all_ports = models_dict['input_data_ports'].values() + models_dict['output_data_ports'].values() + \
                    models_dict['scoped_variables'].values() + models_dict['outcomes'].values()
        if len(set([port_m.core_element.parent for port_m in all_ports])) == 1:
            logger.info("Only one parent {0} {1}".format(all_ports[0].core_element.parent, all_ports[0].parent.get_meta_data_editor(gaphas_editor)))
        if all_ports:
            left = all_ports[0].parent.get_meta_data_editor(gaphas_editor)['rel_pos'][0]
            top = y_axis_mirror * all_ports[0].parent.get_meta_data_editor(gaphas_editor)['rel_pos'][1]
        else:
            raise ValueError("Get boundary method does not aspects all list elements empty in dictionary. {0}"
                             "".format(models_dict))

    def cal_max(max_x, max_y, rel_pos, size):
        max_x = size[0] + rel_pos[0] if size[0] + rel_pos[0] > max_x else max_x
        max_y = y_axis_mirror * rel_pos[1] + size[1] if y_axis_mirror * rel_pos[1] + size[1] > max_y else max_y
        return max_x, max_y

    def cal_min(min_x, min_y, rel_pos, size):
        min_x = rel_pos[0] if rel_pos[0] < min_x else min_x
        min_y = y_axis_mirror * rel_pos[1] if y_axis_mirror * rel_pos[1] < min_y else min_y
        return min_x, min_y

    # Finally take into account all relevant elements in models_dict
    # -> states, scoped variables (maybe input- and output- data ports) and transitions and data flows are relevant
    parts = ['states', 'transitions', 'data_flows']
    if not gaphas_editor:
        parts.append('scoped_variables')
        parts.append('input_data_ports')
        parts.append('output_data_ports')
    for key in parts:
        elems_dict = models_dict[key]
        rel_positions = []
        for model in elems_dict.itervalues():
            _size = (0., 0.)
            if key == 'states':
                rel_positions = [model.get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']]
                _size = model.get_meta_data_editor(for_gaphas=gaphas_editor)['size']
                # print key, rel_positions, _size
            elif key in ['scoped_variables', 'input_data_ports', 'output_data_ports']:
                rel_positions = [model.get_meta_data_editor(for_gaphas=gaphas_editor)['inner_rel_pos']]
                # TODO check to take the ports size into account
                # print key, rel_positions, _size
            elif key in ['transitions', 'data_flows']:
                if gaphas_editor and key is "data_flows":
                    # take into account the meta data positions of opengl if there is some (always in opengl format)
                    rel_positions = mirror_waypoints(deepcopy(model.get_meta_data_editor(for_gaphas=gaphas_editor)))['waypoints']
                else:
                    rel_positions = model.get_meta_data_editor(for_gaphas=gaphas_editor)['waypoints']
                # print key, rel_positions, _size, model.meta

            for rel_position in rel_positions:
                # check for empty fields and ignore them at this point
                if not contains_geometric_info(rel_position):
                    continue
                right, bottom = cal_max(right, bottom, rel_position, _size)
                left, top = cal_min(left, top, rel_position, _size)
                # print "new edges:", left, right, top, bottom, key

    # increase of boundary results into bigger estimated size and finally stronger reduction of original element sizes
    left, right, top, bottom = add_boundary_clearance(left, right, top, bottom, {'size': (0., 0.)}, clearance)
    return left, right, top, bottom


def cal_frame_according_boundaries(left, right, top, bottom, parent_size, gaphas_editor, group=True):
    """ Generate margin and relative position and size handed boundary parameter and parent size """
    y_axis_mirror = 1 if gaphas_editor else -1
    # print "parent_size ->", parent_size
    margin = cal_margin(parent_size)
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
    """ Add position offset to all handed models in dict"""
    # print "\n", "#"*30, "offset models", pos_offset, "#"*30
    # Update relative position of states within the container in order to maintain their absolute position
    for child_state_m in models_dict['states'].itervalues():
        old_rel_pos = child_state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']
        # print "old_rel_pos", old_rel_pos, child_state_m
        child_state_m.set_meta_data_editor('rel_pos', add_pos(old_rel_pos, pos_offset), from_gaphas=gaphas_editor)
        # print "new_rel_pos", child_state_m.get_meta_data_editor(for_gaphas=gaphas_editor), child_state_m

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
            from rafcon.gui.models.data_flow import DataFlowModel
            if isinstance(connection_m, DataFlowModel) and gaphas_editor:
                new_waypoints.append(add_pos(waypoint, (pos_offset[0], -pos_offset[1])))
            else:
                new_waypoints.append(add_pos(waypoint, pos_offset))
        connection_m.set_meta_data_editor('waypoints', new_waypoints, from_gaphas=gaphas_editor)
    # print "END", "#"*30, "offset models", pos_offset, "#"*30, "\n"


def scale_library_ports_meta_data(state_m, gaphas_editor=True):
    """Scale the ports of library model accordingly relative to state_copy meta size.
    
    The function assumes that the meta data of ports of the state_copy of the library was copied to 
    respective elements in the library and that those was not adjusted before.
    """
    if state_m.meta_data_was_scaled:
        return
    state_m.set_meta_data_editor('income.rel_pos',
                                 state_m.state_copy.get_meta_data_editor()['income']['rel_pos'])
    # print "scale_library_ports_meta_data ", state_m.get_meta_data_editor()['size'], \
    #     state_m.state_copy.get_meta_data_editor()['size']
    factor = divide_two_vectors(state_m.get_meta_data_editor()['size'],
                                state_m.state_copy.get_meta_data_editor()['size'])

    # print "scale_library_ports_meta_data -> resize_state_port_meta", factor
    if contains_geometric_info(factor):
        resize_state_port_meta(state_m, factor, True)
        state_m.meta_data_was_scaled = True
    else:
        logger.info("Skip resize of library ports meta data {0}".format(state_m))


def scale_library_content(library_state_m, gaphas_editor=True):
    """Scales the meta data of the content of a LibraryState
    
    The contents of the `LibraryStateModel` `library_state_m` (i.e., the `state_copy` and all it children/state 
    elements) to fit the current size of the `LibraryStateModel`.
    
    :param LibraryStateModel library_state_m: The library who's content is to be resized 
    :param bool gaphas_editor: Whether to use the meta data for the GraphicalEditor using gaphas (default: True) 
    """
    assert isinstance(library_state_m, LibraryStateModel)
    # For library states with an ExecutionState as state_copy, scaling does not make sense
    if not isinstance(library_state_m.state_copy, ContainerStateModel):
        return

    # use same size for state copy and put rel_pos to zero
    library_meta = library_state_m.get_meta_data_editor(gaphas_editor)
    state_copy_meta = library_state_m.state_copy.set_meta_data_editor('size', library_meta['size'], gaphas_editor)
    library_state_m.state_copy.set_meta_data_editor('rel_pos', (0., 0.), from_gaphas=gaphas_editor)

    # work around that gaphas draws in state_copy coordinates (which is not shown) -> reduce state copy size
    if gaphas_editor:
        library_state_margin = cal_margin(state_copy_meta['size'])
        state_copy_size = subtract_pos(state_copy_meta['size'], (2*library_state_margin, 2*library_state_margin))
        library_state_m.state_copy.set_meta_data_editor('size', state_copy_size, gaphas_editor)

    # if meta data has empty fields put default data on state meta data
    if model_has_empty_meta(library_state_m.state_copy) and \
            not put_default_meta_data_on_state_m_recursively(library_state_m.state_copy, library_state_m,
                                                             only_child_states=True):
        return

    # prepare resize by collecting all state elements in the models_dict
    # do resize in respect to state copy
    # (opengl same size as library state and in case of gaphas reduced by library state margin)
    models_dict = {'state': library_state_m.state_copy}
    for state_element_key in library_state_m.state_copy.state.state_element_attrs:
        state_element_list = getattr(library_state_m.state_copy, state_element_key)
        # Some models are hold in a gtkmvc.support.wrappers.ObsListWrapper, not a list
        if hasattr(state_element_list, 'keys'):
            state_element_list = state_element_list.values()
        models_dict[state_element_key] = {elem.core_element.core_element_id: elem for elem in state_element_list}

    # perform final resize
    resize_factor = (1., 1.)
    try:
        if not models_dict['states'] and (not models_dict['scoped_variables'] or gaphas_editor):
            logger.info("Skip scaling for empty root state {0}.".format(library_state_m.state))
        else:
            resize_factor = scale_meta_data_according_state(models_dict, fill_up=True)
    except:
        logger.exception("Scale library content of {0} cause a problem.".format(library_state_m.state))
    finally:
        resize_income_of_state_m(library_state_m.state_copy, resize_factor, gaphas_editor)


def _resize_port_models_list(port_models, rel_pos_key, factor, gaphas_editor):
    """ Resize relative positions a list of (data or logical) port models """
    for port_m in port_models:
        old_rel_pos = port_m.get_meta_data_editor(for_gaphas=gaphas_editor)[rel_pos_key]
        port_m.set_meta_data_editor(rel_pos_key, mult_two_vectors(factor, old_rel_pos), from_gaphas=gaphas_editor)


def _resize_connection_models_list(connection_models, factor, gaphas_editor):
    """ Resize relative positions of way points of a list of connection/linkage models """
    for connection_m in connection_models:
        # print "old_waypoints", connection_m.get_meta_data_editor(for_gaphas=gaphas_editor), connection_m.core_element
        old_waypoints = connection_m.get_meta_data_editor(for_gaphas=gaphas_editor)['waypoints']
        new_waypoints = []
        for waypoint in old_waypoints:
            new_waypoints.append(mult_two_vectors(factor, waypoint))
        connection_m.set_meta_data_editor('waypoints', new_waypoints, from_gaphas=gaphas_editor)
    #     print "new_waypoints", connection_m.get_meta_data_editor(for_gaphas=gaphas_editor), connection_m.core_element


def resize_income_of_state_m(state_m, factor, gaphas_editor):
    if gaphas_editor:
        old_rel_pos = state_m.get_meta_data_editor(for_gaphas=True)['income']['rel_pos']
        state_m.set_meta_data_editor('income.rel_pos', mult_two_vectors(factor, old_rel_pos), from_gaphas=True)
        # print "income", old_rel_pos, state_m.get_meta_data_editor(for_gaphas=True)['income']


def resize_state_port_meta(state_m, factor, gaphas_editor):
    """ Resize data and logical ports relative positions """
    # print "scale ports", factor, state_m, gaphas_editor
    if not gaphas_editor and isinstance(state_m, ContainerStateModel):
        port_models = state_m.input_data_ports[:] + state_m.output_data_ports[:] + state_m.scoped_variables[:]
    else:
        port_models = state_m.input_data_ports[:] + state_m.output_data_ports[:] + state_m.outcomes[:]
        port_models += state_m.scoped_variables[:] if isinstance(state_m, ContainerStateModel) else []
    _resize_port_models_list(port_models, 'rel_pos' if gaphas_editor else 'inner_rel_pos', factor, gaphas_editor)

    resize_income_of_state_m(state_m, factor, gaphas_editor)


def resize_state_meta(state_m, factor, gaphas_editor):
    """ Resize state meta data recursive what includes also LibraryStateModels meta data and its internal state_copy
    """
    # print "START RESIZE OF STATE", state_m.get_meta_data_editor(for_gaphas=gaphas_editor), state_m
    old_rel_pos = state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']
    # print "old_rel_pos state", old_rel_pos, state_m.core_element
    state_m.set_meta_data_editor('rel_pos', mult_two_vectors(factor, old_rel_pos), from_gaphas=gaphas_editor)
    # print "new_rel_pos state", state_m.get_meta_data_editor(for_gaphas=gaphas_editor), state_m.core_element

    # print "resize factor", factor,  state_m, state_m.meta
    old_size = state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['size']
    # print "old_size", old_size, type(old_size)
    state_m.set_meta_data_editor('size', mult_two_vectors(factor, old_size), from_gaphas=gaphas_editor)
    # print "new_size", state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['size']
    if gaphas_editor:
        old_rel_pos = state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['name']['rel_pos']
        state_m.set_meta_data_editor('name.rel_pos', mult_two_vectors(factor, old_rel_pos), from_gaphas=gaphas_editor)
        old_size = state_m.get_meta_data_editor(for_gaphas=gaphas_editor)['name']['size']
        state_m.set_meta_data_editor('name.size', mult_two_vectors(factor, old_size), from_gaphas=gaphas_editor)

    if isinstance(state_m, LibraryStateModel):
        # print "LIBRARY", state_m
        if gaphas_editor and state_m.state_copy_initialized:
            if state_m.meta_data_was_scaled:
                resize_state_port_meta(state_m, factor, gaphas_editor)
            else:
                scale_library_ports_meta_data(state_m, gaphas_editor)

        if state_m.state_copy_initialized:
            resize_state_meta(state_m.state_copy, factor, gaphas_editor)
        # print "END LIBRARY RESIZE"
    else:
        # print "resize_state_meta -> resize_state_port_meta"
        resize_state_port_meta(state_m, factor, gaphas_editor)
        if isinstance(state_m, ContainerStateModel):
            _resize_connection_models_list(state_m.transitions[:] + state_m.data_flows[:], factor, gaphas_editor)
            for child_state_m in state_m.states.itervalues():
                resize_state_meta(child_state_m, factor, gaphas_editor)
    # print "re-sized state", state_m.get_meta_data_editor(for_gaphas=gaphas_editor), state_m.core_element


def resize_of_all_models_in_dict(models_dict, factor, gaphas_editor):
    # print "\n", "#"*30, "resize models", factor, "#"*30,

    # Update relative position of states within the container in order to maintain their absolute position
    for child_state_m in models_dict['states'].itervalues():
        resize_state_meta(child_state_m, factor, gaphas_editor)

    # Do the same for data and logic ports
    port_models = models_dict.get('scoped_variables', {}).values() + models_dict['input_data_ports'].values() + \
                  models_dict['output_data_ports'].values()
    port_models += models_dict['outcomes'].values() if gaphas_editor else []
    _resize_port_models_list(port_models, 'rel_pos' if gaphas_editor else 'inner_rel_pos', factor, gaphas_editor)

    # Do the same for all connections (transitions and data flows)
    connection_models = models_dict['transitions'].values() + models_dict['data_flows'].values()
    _resize_connection_models_list(connection_models, factor, gaphas_editor)
    # print "END", "#"*30, "resize models", factor, "#"*30, "\n"


def offset_rel_pos_of_models_meta_data_according_parent_state(models_dict):
    """ Offset meta data of state elements according the area used indicated by the state meta data.

    The offset_rel_pos_of_models_meta_data_according_parent_state offset the position of all handed old elements
    in the dictionary.

    :param models_dict: dict that hold lists of meta data with state attribute consistent keys
    :return:
    """
    gaphas_editor = True if global_gui_config.get_config_value('GAPHAS_EDITOR') else False
    old_parent_rel_pos = models_dict['state'].get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos']
    offset_rel_pos_of_all_models_in_dict(models_dict, pos_offset=old_parent_rel_pos, gaphas_editor=gaphas_editor)

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


def scale_meta_data_according_state(models_dict, rel_pos=None, as_template=False, fill_up=False):
    # TODO Documentation needed....
    """ Scale elements in dict to fit into dict state key element.

    If elements are already small enough no resize is performed.
    """
    gaphas_editor, y_axis_mirror = get_y_axis_and_gaphas_editor_flag()
    # TODO check about positions of input-data- and output-data- or scoped variable-ports is needed
    # TODO adjustments of data ports positions are not sufficient -> origin state size is maybe needed for that
    # TODO consistency check on boundary and scale parameter for every if else case

    if 'states' in models_dict or 'scoped_variables' in models_dict:
        left, right, top, bottom = get_boundaries_of_elements_in_dict(models_dict=models_dict)
        parent_size = models_dict['state'].get_meta_data_editor(for_gaphas=gaphas_editor)['size']
        margin, old_rel_pos, size = cal_frame_according_boundaries(left, right, top, bottom, parent_size,
                                                                   gaphas_editor, group=False)
        automatic_mode = True if rel_pos is None else False
        clearance = 0.2 if rel_pos is None else 0.
        rel_pos = (margin, margin) if rel_pos is None else rel_pos
        # rel_pos = (0., 0.) if fill_up else rel_pos
        clearance_scale = 1 + clearance

        assert parent_size[0] > rel_pos[0]
        assert parent_size[1] > rel_pos[1]
        # print "edges:", left, right, top, bottom
        # print "margin:", margin, "rel_pos:", rel_pos, "old_rel_pos", old_rel_pos, "size:", size

        parent_width, parent_height = parent_size

        boundary_width, boundary_height = size
        # print "parent width:   {0}, parent_height:   {1}".format(parent_width, parent_height)
        # print "boundary width: {0}, boundary_height: {1}".format(boundary_width, boundary_height)

        # no site scale
        if parent_width - rel_pos[0] > boundary_width * clearance_scale and \
                parent_height - rel_pos[1] > boundary_height * clearance_scale and not fill_up:
            if automatic_mode:
                resize_factor = 1.
                boundary_width_in_parent = boundary_width*resize_factor
                # print boundary_width, resize_factor, boundary_width*resize_factor, boundary_height*resize_factor + margin*2, parent_height
                # print "left over width: ", parent_width - boundary_width_in_parent - rel_pos[0] - margin
                width_pos_offset_to_middle = (parent_width - boundary_width_in_parent - rel_pos[0] - margin)/2.
                rel_pos = add_pos(rel_pos, (width_pos_offset_to_middle, 0.))
                boundary_height_in_parent = boundary_height*resize_factor
                # print "left over height: ", parent_height - boundary_height_in_parent - rel_pos[0] - margin
                height_pos_offset_to_middle = (parent_height - boundary_height_in_parent - rel_pos[1] - margin)/2.
                rel_pos = add_pos(rel_pos, (0., height_pos_offset_to_middle))
            offset = subtract_pos((0., 0.), subtract_pos(old_rel_pos, rel_pos))
            offset_rel_pos_of_all_models_in_dict(models_dict, mult_two_vectors((1., y_axis_mirror), offset), gaphas_editor)
            return 1, 1  # (1, 1), offset, rel_pos
        # smallest site scale
        else:
            # increase of boundary or clearance results into bigger estimated size and finally stronger
            # reduction of original element sizes
            left, right, top, bottom = get_boundaries_of_elements_in_dict(models_dict=models_dict, clearance=clearance)
            parent_size = models_dict['state'].get_meta_data_editor(for_gaphas=gaphas_editor)['size']
            margin, old_rel_pos, size = cal_frame_according_boundaries(left, right, top, bottom, parent_size,
                                                                       gaphas_editor, False)

            rel_pos = (margin, margin) if rel_pos is None else rel_pos
            assert parent_size[0] > rel_pos[0]
            assert parent_size[1] > rel_pos[1]
            # print "edges:", left, right, top, bottom
            # print "margin:", margin, "rel_pos:", rel_pos, "old_rel_pos", old_rel_pos, "size:", size

            parent_width, parent_height = parent_size

            boundary_width, boundary_height = size
            if (parent_height - rel_pos[1] - margin)/boundary_height < \
                    (parent_width - rel_pos[0] - margin)/boundary_width:
                # print "#2"*20, 1, "#"*20, rel_pos
                resize_factor = (parent_height - rel_pos[1] - margin)/boundary_height
                boundary_width_in_parent = boundary_width*resize_factor
                # print boundary_width, resize_factor, boundary_width*resize_factor
                # print "left over width: ", parent_width - boundary_width_in_parent - rel_pos[0] - margin
                width_pos_offset_to_middle = (parent_width - boundary_width_in_parent - rel_pos[0] - margin)/2.
                rel_pos = add_pos(rel_pos, (width_pos_offset_to_middle, 0.))
                # print resize_factor, rel_pos
            else:
                # print "#2"*20, 2, "#"*20, rel_pos
                resize_factor = (parent_width - rel_pos[0] - margin)/boundary_width
                boundary_height_in_parent = boundary_height*resize_factor
                # print boundary_width, resize_factor, boundary_width*resize_factor
                # print "left over height: ", parent_height - boundary_height_in_parent - rel_pos[0] - margin
                height_pos_offset_to_middle = (parent_height - boundary_height_in_parent - rel_pos[1] - margin)/2.
                rel_pos = add_pos(rel_pos, (0., height_pos_offset_to_middle))
                # print resize_factor, rel_pos
            if not global_gui_config.get_config_value('GAPHAS_ENABLED'):
                resize_factor *= 0.9  # TODO check it again (needed to hold child elements in bounds of parent state)
            frame = {'rel_pos': rel_pos, 'size': mult_two_vectors((resize_factor, resize_factor), size)}
            # print models_dict['state'].get_meta_data_editor(for_gaphas=gaphas_editor)['rel_pos'], \
            #     parent_size, models_dict['state'].parent.get_meta_data_editor(for_gaphas=gaphas_editor)['size']
            # print "frame", frame, resize_factor
            # # rel_pos = mult_two_vectors((1, -1), frame['rel_pos'])
            offset = subtract_pos((0., 0.), mult_two_vectors((1., y_axis_mirror), old_rel_pos))
            offset_rel_pos_of_all_models_in_dict(models_dict, offset, gaphas_editor)

            resize_of_all_models_in_dict(models_dict, (resize_factor, resize_factor), gaphas_editor)

            offset_rel_pos_of_all_models_in_dict(models_dict, mult_two_vectors((1., y_axis_mirror), frame['rel_pos']), gaphas_editor)
            # scale_meta_data_according_frame(models_dict, frame)
            return resize_factor, resize_factor  # (resize_factor, resize_factor) , offset, frame['rel_pos']
    else:
        return 1, 1  # (1, 1) , (0., 0.), (0., 0.)


def scale_meta_data_according_frame(models_dict, frame):
    # TODO Documentation needed....
    """

    :param models_dict: dictionary that hold lists of meta data with state attribute consistent keys
    :return:
    """
    # TODO check if this is working and wherefore we wanted to use it -> scaling of states into empty space
    gaphas_editor, y_axis_mirror = get_y_axis_and_gaphas_editor_flag()

    left, right, top, bottom = get_boundaries_of_elements_in_dict(models_dict=models_dict)
    # increase of boundary results into bigger estimated size and finally stronger reduction of original element sizes
    left, right, top, bottom = add_boundary_clearance(left, right, top, bottom, frame)

    margin, old_rel_pos, size = cal_frame_according_boundaries(left, right, top, bottom, (0., 0.), gaphas_editor, False)
    old_frame = {'rel_pos': old_rel_pos, 'size': size}

    # cal offset and resize factor in between
    offset = subtract_pos((0., 0.), old_frame['rel_pos'])
    resize_factor = divide_two_vectors(frame['size'], old_frame['size'])

    offset_rel_pos_of_all_models_in_dict(models_dict, offset, gaphas_editor)
    resize_of_all_models_in_dict(models_dict, (resize_factor, resize_factor), gaphas_editor)
    offset_rel_pos_of_all_models_in_dict(models_dict, mult_two_vectors((1., y_axis_mirror), frame['rel_pos']), gaphas_editor)
    return True


def meta_data_reference_check(meta):

    def reference_free_check(v1, v2, prepend=[]):
        """Returns elements of a dict that have the same memory addresses except strings."""
        d = {'value': {}, 'same_ref': [], 'same_ref_value': [], 'missing_keys1': [], 'missing_keys2': []}
        v1_keys = v1.keys()
        v2_keys = v2.keys()
        not_to_check = set(v1_keys).symmetric_difference(v2_keys)
        d['missing_keys1'] = filter(lambda k: k in not_to_check, v1_keys)
        d['missing_keys2'] = filter(lambda k: k in not_to_check, v2_keys)
        for key in set(v1_keys + v2_keys):
            if key not in not_to_check:
                if not hasattr(v1[key], 'keys'):
                    if isinstance(v1[key], str):
                        d['value'].update({key: v1[key]})
                    else:
                        if id(v1[key]) == id(v2[key]):
                            if not isinstance(v1[key], tuple):
                                d['same_ref'].append(prepend + [key])
                                d['same_ref_value'].append(str(v1[key]) + " == " + str(v2[key]) + ', ' + str(type(v1[key])) + " == " + str(type(v2[key])))
                        else:
                            d['value'].update({key: v1[key]})
                else:
                    if id(v1[key]) == id(v2[key]):
                        d['same_ref'].append(prepend + [key])
                        d['same_ref_value'].append(str(v1[key]) + " == " + str(v2[key]) + ', ' + str(type(v1[key])) + " == " + str(type(v2[key])))
                    else:
                        d['value'].update({key: reference_free_check(v1[key], v2[key], prepend=prepend + [key])})

        return d

    meta_source = meta
    meta_str = json.dumps(meta, cls=JSONObjectEncoder,
                          indent=4, check_circular=False, sort_keys=True)
    meta_dump_copy = json.loads(meta_str, cls=JSONObjectDecoder, substitute_modules=substitute_modules)
    meta_deepcopy = deepcopy(meta)

    meta_source_str = json.dumps(meta, cls=JSONObjectEncoder,
                                 indent=4, check_circular=False, sort_keys=True)
    meta_dump_copy_str = json.dumps(meta_dump_copy, cls=JSONObjectEncoder,
                                    indent=4, check_circular=False, sort_keys=True)
    meta_deepcopy_str = json.dumps(meta_deepcopy, cls=JSONObjectEncoder,
                                   indent=4, check_circular=False, sort_keys=True)
    assert meta_dump_copy_str == meta_source_str
    assert meta_dump_copy_str == meta_deepcopy_str

    def diff_print(diff):
        if diff['same_ref']:
            logger.verbose("same_ref: {0} {1}".format(diff['same_ref'], diff['same_ref_value']))
            assert False
        for value in diff['value'].itervalues():
            if isinstance(value, dict):
                diff_print(value)

    source_dump_diff = reference_free_check(meta_source, meta_dump_copy)
    source_deep_diff = reference_free_check(meta_source, meta_deepcopy)
    logger.verbose("source_dump_diff")
    diff_print(source_dump_diff)
    logger.verbose("source_deep_diff")
    diff_print(source_deep_diff)

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
