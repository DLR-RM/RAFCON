import rafcon.gui.singleton

from rafcon.core.state_elements.transition import Transition
from rafcon.core.states.state import State
from rafcon.utils import log

import rafcon.gui.helpers.meta_data as gui_helper_meta_data

logger = log.get_logger(__name__)


def is_outcome_connect_to_state(outcome, state_id):
    state = outcome.parent
    transition = state.parent.get_transition_for_outcome(state, outcome)
    return isinstance(transition, Transition) and transition.to_state == state_id


def all_outcomes_have_transitions_with_the_same_target(outcomes):
    target_state_id = None
    target_outcome_id = None
    for oc in outcomes:
        transition = oc.parent.parent.get_transition_for_outcome(oc.parent, oc)
        if target_state_id is None and target_outcome_id is None \
                or target_state_id == transition.to_state and target_outcome_id == transition.to_outcome:
            target_state_id = transition.to_state
            target_outcome_id = transition.to_outcome
        else:
            return

    return target_state_id, target_outcome_id


def remove_transitions_if_target_is_the_same(from_outcomes):
    target = all_outcomes_have_transitions_with_the_same_target(from_outcomes)
    if target:
        for from_outcome in from_outcomes:
            transition = from_outcome.parent.parent.get_transition_for_outcome(from_outcome.parent, from_outcome)
            from_outcome.parent.parent.remove(transition)
        return target


def get_all_outcomes_except_of_abort_and_preempt(state):
    return [outcome for outcome in state.outcomes.itervalues() if outcome.outcome_id >= 0]


def add_transitions_from_selected_state_to_parent():
    """ Generates the default success transition of a state to its parent success port

    :return:
    """
    selected_sm_id = rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id
    if not selected_sm_id:
        logger.warning("Can not create transition to parent state: No state machine selected!")
        return
    else:
        logger.debug("Check transition to parent state ...")

    selected_sm_m = rafcon.gui.singleton.state_machine_manager_model.state_machines[selected_sm_id]
    if len(selected_sm_m.selection.states) == 1:
        state = selected_sm_m.selection.get_selected_state().state
        parent_state = state.parent
        if not isinstance(parent_state, State):
            logger.warning("Can not create transition to parent state: Selected state has not parent state!")
            return

        # find all possible from outcomes
        from_outcomes = get_all_outcomes_except_of_abort_and_preempt(state)

        # find lowest valid outcome id
        possible_oc_ids = [oc_id for oc_id in state.parent.outcomes.keys() if oc_id >= 0]
        possible_oc_ids.sort()
        to_outcome = state.parent.outcomes[possible_oc_ids[0]]

        oc_connected_to_parent = [oc for oc in from_outcomes if is_outcome_connect_to_state(oc, parent_state.state_id)]
        oc_not_connected = [oc for oc in from_outcomes if not state.parent.get_transition_for_outcome(state, oc)]
        if all(oc in oc_connected_to_parent for oc in from_outcomes):
            logger.info("Remove transition to parent state because all outcomes are connected to it.")
            for from_outcome in oc_connected_to_parent:
                transition = parent_state.get_transition_for_outcome(state, from_outcome)
                parent_state.remove(transition)
        elif oc_not_connected:
            logger.debug("Create transition to parent state ...")
            for from_outcome in from_outcomes:
                parent_state.add_transition(state.state_id, from_outcome.outcome_id,
                                            parent_state.state_id, to_outcome.outcome_id)
        else:
            if remove_transitions_if_target_is_the_same(from_outcomes):
                logger.info("Removed transitions origin from outcomes of selected state "
                            "because all point to the same target.")
                return add_transitions_from_selected_state_to_parent()
            logger.info("Will not create transition to parent state: Not clear situation of connected transitions.\n"
                        "There will be no transitions to other states be touched.")

    else:
        logger.warning("Can not create transition to parent state: Please select one state!")

    return True


def add_transitions_to_closest_sibling_state_from_selected_state():
    """ Generates the outcome transitions from outcomes with positive outcome_id to the closest next state

    :return:
    """
    task_string = "to closest sibling state"
    selected_sm_id = rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id
    if not selected_sm_id:
        logger.warning("Can not create transition {0}: No state machine selected!".format(task_string))
        return
    else:
        logger.debug("Check transition {0} ...".format(task_string))

    selected_sm_m = rafcon.gui.singleton.state_machine_manager_model.state_machines[selected_sm_id]
    if len(selected_sm_m.selection.states) == 1:
        selected_state_m = selected_sm_m.selection.get_selected_state()
        state = selected_state_m.state
        parent_state = state.parent
        if not isinstance(parent_state, State):
            logger.warning("Can not create transition {0}: Selected state has not parent state!".format(task_string))
            return

        # find closest other state to connect to -> to_state
        closest_sibling_state = gui_helper_meta_data.get_closest_sibling_state(selected_state_m)
        if closest_sibling_state is None:
            logger.info("Can not create transition {0}: There is no other sibling state.".format(task_string))
        to_state = closest_sibling_state[1].state

        # find all possible from outcomes
        from_outcomes = get_all_outcomes_except_of_abort_and_preempt(state)

        from_oc_not_connected = [oc for oc in from_outcomes if not state.parent.get_transition_for_outcome(state, oc)]
        # all ports not connected connect to next state income
        if from_oc_not_connected:
            logger.debug("Create transition {0} ...".format(task_string))
            for from_outcome in from_oc_not_connected:
                parent_state.add_transition(state.state_id, from_outcome.outcome_id, to_state.state_id, None)
        # no transitions are removed if not all connected to the same other state
        else:
            target = remove_transitions_if_target_is_the_same(from_outcomes)
            if target:
                target_state_id, _ = target
                if not target_state_id == to_state.state_id:
                    logger.info("Removed transitions from outcomes {0} "
                                "because all point to the same target.".format(task_string.replace('closest ', '')))
                    add_transitions_to_closest_sibling_state_from_selected_state()
                else:
                    logger.info("Removed transitions from outcomes {0} "
                                "because all point to the same target.".format(task_string))
                return True
            logger.info("Will not create transition {0}: Not clear situation of connected transitions."
                        "There will be no transitions to other states be touched.".format(task_string))

    else:
        logger.warning("Can not create transition {0}: Please select one state!".format(task_string))
    return True


def add_transitions_from_closest_sibling_state_to_selected_state():
    """ Generates the outcome transitions from the outcomes of the closest state to selected state income

    :return:
    """
    task_string = "from closest sibling state"
    selected_sm_id = rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id
    if not selected_sm_id:
        logger.warning("Can not create transitions {0}: No state machine selected!".format(task_string))
        return
    else:
        logger.debug("Check transitions {0} ...".format(task_string))

    selected_sm_m = rafcon.gui.singleton.state_machine_manager_model.state_machines[selected_sm_id]
    if len(selected_sm_m.selection.states) == 1:
        selected_state_m = selected_sm_m.selection.get_selected_state()
        state = selected_state_m.state
        parent_state = state.parent
        if not isinstance(parent_state, State):
            logger.warning("Can not create transition {0}: Selected state has no parent state!".format(task_string))
            return
        # find closest other state to connect to -> from_state
        closest_sibling_state = gui_helper_meta_data.get_closest_sibling_state(selected_state_m)
        if closest_sibling_state is None:
            logger.info("Can not create transition {0}: There is no other sibling state.".format(task_string))
        from_state = closest_sibling_state[1].state

        # find all possible from outcomes
        from_outcomes = get_all_outcomes_except_of_abort_and_preempt(from_state)

        from_oc_not_connected = [oc for oc in from_outcomes if not state.parent.get_transition_for_outcome(from_state, oc)]

        # all ports not connected connect to state income
        if from_oc_not_connected:
            logger.debug("Create transitions {0} ...".format(task_string))
            for from_outcome in from_oc_not_connected:
                parent_state.add_transition(from_state.state_id, from_outcome.outcome_id, state.state_id, None)
        # no transitions are removed if not all connected to the same other target
        else:
            target = remove_transitions_if_target_is_the_same(from_outcomes)
            if target:
                target_state_id, _ = target
                if not target_state_id == state.state_id:
                    logger.info("Removed transitions origin from outcomes of {0} "
                                "because all point to the same target.".format(task_string.replace("from ", "")))
                    add_transitions_from_closest_sibling_state_to_selected_state()
                else:
                    logger.info("Removed transitions origin from outcomes of {0} to selected state {0} "
                                "because all point to the same target.".format(task_string.replace("from ", "")))
                return True
            logger.info("Will not create transition {0}: Not clear situation of connected transitions."
                        "There will be no transitions to other states be touched.".format(task_string))

    else:
        logger.warning("Can not create transition {0}: Please select one state!".format(task_string))
    return True
