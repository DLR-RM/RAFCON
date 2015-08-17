from rafcon.mvc.views.gap.state import StateView, NameView
from rafcon.mvc.views.gap.ports import InputPortView, OutputPortView, IncomeView, OutcomeView,\
    ScopedVariablePortView
from rafcon.mvc.views.gap.connection import TransitionView

from rafcon.statemachine.states.container_state import ContainerState

from gaphas.item import NW

from rafcon.utils import log
logger = log.get_logger(__name__)


def calc_rel_pos_to_parent(canvas, item, handle):
    """
    This method calculates the relative position of the given item's handle to its parent
    :param canvas: Canvas to find relative position in
    :param item: Item to find relative position to parent
    :param handle: Handle of item to find relative position to
    :return: Relative position (x, y)
    """
    parent = canvas.get_parent(item)
    if parent:
        c_pos = canvas.project(item, handle.pos)
        p_pos = canvas.project(parent, parent.handles()[NW].pos)
        rel_x = c_pos[0].value - p_pos[0].value
        rel_y = c_pos[1].value - p_pos[1].value
    else:
        pos = canvas.project(item, item.handles()[NW].pos)
        rel_x = pos[0].value
        rel_y = pos[1].value
    return rel_x, rel_y


def assert_exactly_one_true(bool_list):
    """
    This method asserts that only one value of the provided list is True.
    :param bool_list: List of booleans to check
    :return: True if only one value is True, False otherwise
    """
    assert isinstance(bool_list, list)
    counter = 0
    for item in bool_list:
        if item:
            counter += 1
    return counter == 1


def get_state_id_for_port(port):
    """
    This method returns the state ID of the state containing the given port
    :param port: Port to check for containing state ID
    :return: State ID of state containing port
    """
    parent = port.parent
    if isinstance(parent, StateView):
        return parent.model.state.state_id


def get_port_for_handle(handle, state):
    """
    Looks for and returns the PortView to the given handle in the provided state
    :param handle: Handle to look for port
    :param state: State containing handle and port
    :returns: PortView for handle
    """
    if isinstance(state, StateView):
        if state.income.handle == handle:
            return state.income
        else:
            for outcome in state.outcomes:
                if outcome.handle == handle:
                    return outcome
            for input in state.inputs:
                if input.handle == handle:
                    return input
            for output in state.outputs:
                if output.handle == handle:
                    return output
            for scoped in state.scoped_variables:
                if scoped.handle == handle:
                    return scoped


def create_new_connection(start_state, from_port, to_port, drop_state=None):
    """
    Creates a new connection and adds this connection to the state machine model.
    If the new connection is transition:
    Only transitions from outcomes are added (transitions from incomes are added via "start_state" checkbox
    """
    def is_income_or_outcome(port):
        return isinstance(port, IncomeView) or isinstance(port, OutcomeView)

    def is_input_or_output(port):
        return isinstance(port, InputPortView) or isinstance(port, OutputPortView)

    # Ensure from_port is an outcome and
    # from_port as well as to_port are connected to transition and
    # ports are not the same port
    if from_port and to_port and from_port is not to_port:
        # Make sure the ports are not connected to an outcome of the same state
        if (from_port.parent is to_port.parent and isinstance(from_port, OutcomeView) and
                isinstance(to_port, OutcomeView)):
            logger.warn("Cannot connect outcome to outcome of the same state")
            return
        if (from_port.parent is to_port.parent and isinstance(from_port, IncomeView) and
                isinstance(to_port, OutcomeView)):
            logger.warn("Cannot connect income to outcome of the same state")
            return
        if isinstance(from_port, InputPortView) and isinstance(to_port, OutputPortView):
            logger.warn("Cannot connect input to output")
            return

        # It is possible to create new transitions from:
        # - income to income (parent state to child state)
        # - outcome to income
        # - outcome to outcome (child state to parent state)
        if is_income_or_outcome(from_port) and is_income_or_outcome(to_port):
            add_transition_to_state(start_state, to_port, from_port)
        # It is possible to create new data flows from:
        # - input to input (parent state to child state)
        # - output to input
        # - output to output (child state to parent state)
        elif is_input_or_output(from_port) and is_input_or_output(to_port):
            add_data_flow_to_port_parent(to_port, from_port)
        elif isinstance(from_port, ScopedVariablePortView) and is_input_or_output(to_port):
            add_from_scoped_variable_data_flow_to_port_parent(to_port, from_port)
        elif is_input_or_output(from_port) and isinstance(to_port, ScopedVariablePortView):
            add_to_scoped_variable_data_flow_to_port_parent(to_port, from_port)
    elif from_port and drop_state:
        if is_income_or_outcome(from_port):
            add_transition_to_state(start_state, drop_state.income, from_port)


def add_from_scoped_variable_data_flow_to_port_parent(to_port, from_port):
    """
    This method adds a new data flow from a scoped variable to a data port.
    """
    from_state_v = from_port.parent
    to_state_v = to_port.parent

    from_state_m = from_state_v.model
    to_state_m = to_state_v.model

    if isinstance(to_port, InputPortView) and from_state_v is to_state_v:
        return
    if isinstance(to_port, InputPortView) and to_port.has_incoming_connection():
        return

    responsible_parent_m = from_state_m

    from_state_id = from_state_m.state.state_id
    from_data_port_id = from_port.port_id
    to_state_id = to_state_m.state.state_id
    to_data_port_id = to_port.port_id

    if isinstance(responsible_parent_m.state, ContainerState):
        try:
            responsible_parent_m.state.add_data_flow(from_state_id, from_data_port_id, to_state_id, to_data_port_id)
        except AttributeError as e:
            logger.warn(e)


def add_to_scoped_variable_data_flow_to_port_parent(to_port, from_port):
    """
    This method adds a new data flow to a scoped variable from a data port.
    """
    from_state_v = from_port.parent
    to_state_v = to_port.parent

    from_state_m = from_state_v.model
    to_state_m = to_state_v.model

    if isinstance(from_port, OutputPortView) and to_state_v in from_state_v.parent.child_state_vs:
        return

    responsible_parent_m = to_state_m

    from_state_id = from_state_m.state.state_id
    from_data_port_id = from_port.port_id
    to_state_id = to_state_m.state.state_id
    to_data_port_id = to_port.port_id

    if isinstance(responsible_parent_m.state, ContainerState):
        try:
            responsible_parent_m.state.add_data_flow(from_state_id, from_data_port_id, to_state_id, to_data_port_id)
        except AttributeError as e:
            logger.warn(e)


def add_data_flow_to_port_parent(to_port, from_port):
    """
    This method adds a new data flow.
    """
    from_state_v = from_port.parent
    to_state_v = to_port.parent

    from_state_m = from_state_v.model
    to_state_m = to_state_v.model

    if (isinstance(from_port, InputPortView) and
            isinstance(to_port, InputPortView) and
            from_state_m is to_state_m.parent):
        responsible_parent_m = from_state_m
        if to_port.has_incoming_connection():
            return
    elif (isinstance(from_port, OutputPortView) and
            isinstance(to_port, OutputPortView) and
            to_state_m is from_state_m.parent):
        responsible_parent_m = to_state_m
        if to_port.has_incoming_connection():
            return
    elif (isinstance(from_port, OutputPortView) and
            isinstance(to_port, InputPortView) and
            to_state_m.parent is from_state_m.parent):
        responsible_parent_m = to_state_m.parent
        if to_port.has_incoming_connection():
            return
    else:
        return

    from_state_id = from_state_m.state.state_id
    from_data_port_id = from_port.port_id
    to_state_id = to_state_m.state.state_id
    to_data_port_id = to_port.port_id

    if isinstance(responsible_parent_m.state, ContainerState):
        try:
            responsible_parent_m.state.add_data_flow(from_state_id, from_data_port_id, to_state_id, to_data_port_id)
        except AttributeError as e:
            logger.warn(e)


def add_transition_to_state(start_state, to_port, from_port):
    """
    This method adds a new transition.
    """
    from_state_v = from_port.parent
    to_state_v = to_port.parent

    # Gather necessary information to create transition
    from_state_id = None
    from_outcome_id = None
    to_state_id = None
    to_outcome_id = None

    from_state_m = from_state_v.model
    to_state_m = to_state_v.model
    responsible_parent_m = None

    if isinstance(from_port, OutcomeView):
        from_state_id = start_state.model.state.state_id
        from_outcome_id = from_port.outcome_id

    if isinstance(to_port, IncomeView) and isinstance(from_port, IncomeView):
        to_state_id = to_state_m.state.state_id
        responsible_parent_m = from_state_m
    elif isinstance(to_port, IncomeView) and isinstance(from_port, OutcomeView):
        to_state_id = to_state_m.state.state_id
        responsible_parent_m = from_state_m.parent
    elif isinstance(to_port, OutcomeView) and isinstance(from_port, OutcomeView):
        to_state_id = to_state_m.state.state_id
        to_outcome_id = to_port.outcome_id
        responsible_parent_m = to_state_m

    if responsible_parent_m:
        try:
            responsible_parent_m.state.add_transition(from_state_id,
                                                      from_outcome_id,
                                                      to_state_id,
                                                      to_outcome_id)
        except AttributeError as e:
            logger.warn("Transition couldn't be added: {0}".format(e))
        except Exception as e:
            logger.error("Unexpected exception while creating transition: {0}".format(e))


def convert_handles_pos_list_to_rel_pos_list(canvas, transition):
    """
    This method takes the waypoints of a connection and returns all relative positions of these waypoints.
    :param canvas: Canvas to check relative position in
    :param transition: Transition to extract all relative waypoint positions
    :return: List with all relative positions of the given transition
    """
    handles_list = transition.handles()
    rel_pos_list = []
    for handle in handles_list:
        if handle in transition.end_handles_perp():
            continue
        rel_pos_list.append(calc_rel_pos_to_parent(canvas, transition, handle))
    return rel_pos_list


def update_transition_waypoints(graphical_editor_view, transition_v, last_waypoint_list):
    """
    This method updates the relative position meta data of the transitions waypoints if they changed
    :param graphical_editor_view: Graphical Editor the change occurred in
    :param transition_v: Transition that changed
    :param last_waypoint_list: List of waypoints before change
    """
    assert isinstance(transition_v, TransitionView)
    transition_m = transition_v.model
    transition_meta_gaphas = transition_m.meta['gui']['editor_gaphas']
    waypoint_list = convert_handles_pos_list_to_rel_pos_list(graphical_editor_view.editor.canvas, transition_v)
    if waypoint_list != last_waypoint_list:
        transition_meta_gaphas['waypoints'] = waypoint_list
        graphical_editor_view.emit('meta_data_changed', transition_m, "Move waypoint", True)


def update_port_position_meta_data(graphical_editor_view, item, handle):
    """
    This method updates the meta data of the states ports if they changed.
    :param graphical_editor_view: Graphical Editor the change occurred in
    :param item: State the port was moved in
    :param handle: Handle of moved port
    """
    rel_pos = (handle.pos.x.value, handle.pos.y.value)
    port_meta = None
    for port in item.get_all_ports():
        if handle is port.handle:
            if isinstance(port, IncomeView):
                port_meta = item.model.meta['income']['gui']['editor_gaphas']
            elif isinstance(port, OutcomeView):
                port_meta = item.model.meta['outcome%d' % port.outcome_id]['gui']['editor_gaphas']
            elif isinstance(port, InputPortView):
                port_meta = item.model.meta['input%d' % port.port_id]['gui']['editor_gaphas']
            elif isinstance(port, OutputPortView):
                port_meta = item.model.meta['output%d' % port.port_id]['gui']['editor_gaphas']
            elif isinstance(port, ScopedVariablePortView):
                port_meta = item.model.meta['scoped%d' % port.port_id]['gui']['editor_gaphas']
            break
    if rel_pos != port_meta['rel_pos']:
        port_meta['rel_pos'] = rel_pos
        graphical_editor_view.emit('meta_data_changed', item.model, "Move port", True)


def update_meta_data_for_item(graphical_editor_view, grabbed_handle, item, child_resize=False):
    """
    This method updates the meta data of state and name views.
    :param graphical_editor_view: Graphical Editor the change occurred in
    :param grabbed_handle: The handle that has been moved
    :param item: The item which has been changed/moved
    :param child_resize: Whether the children of the item have been resized or not
    """
    size_msg = "Change size"
    move_msg = "Move"
    affect_children = False

    meta_gaphas = None
    meta_opengl = None

    if isinstance(item, StateView):
        # If handle for state resize was pulled
        if grabbed_handle in item.corner_handles or child_resize:
            # Update all port meta data to match with new position and size of parent
            for port in item.get_all_ports():
                update_port_position_meta_data(graphical_editor_view, item, port.handle)
            meta_gaphas = item.model.meta['gui']['editor_gaphas']
            meta_opengl = item.model.meta['gui']['editor_opengl']
            move_msg = "Move state"
            affect_children = True
        # If pulled handle is port update port meta data and return
        else:
            update_port_position_meta_data(graphical_editor_view, item, grabbed_handle)
            return
    elif isinstance(item, NameView):
        parent = graphical_editor_view.editor.canvas.get_parent(item)
        item = parent
        assert isinstance(parent, StateView)

        meta_gaphas = parent.model.meta['name']['gui']['editor_gaphas']
        size_msg = "Change name size"
        move_msg = "Move name"

    rel_pos = calc_rel_pos_to_parent(graphical_editor_view.editor.canvas, item, item.handles()[NW])

    if meta_gaphas:
        meta_gaphas['size'] = (item.width, item.height)
        graphical_editor_view.emit('meta_data_changed', item.model, size_msg, affect_children)
        meta_gaphas['rel_pos'] = rel_pos
        graphical_editor_view.emit('meta_data_changed', item.model, move_msg, affect_children)
    if meta_opengl:
        meta_opengl['size'] = (item.width, item.height)
        meta_opengl['rel_pos'] = (rel_pos[0], -rel_pos[1])
