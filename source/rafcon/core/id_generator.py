# Copyright (C) 2014-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>
# Sebastian Riedel <sebastian.riedel@dlr.de>

"""
.. module:: id_generator
   :synopsis: A module to generate different kinds of state machine ids

"""

from builtins import str
from builtins import range
import string
import random
import uuid

STATE_ID_LENGTH = 6
RUN_ID_LENGTH = 10
experiment_id = ''+str(uuid.uuid1())

state_name_counter = 0
state_machine_id_counter = 0
transition_id_counter = 0
data_flow_id_counter = 0
script_id_counter = 0
run_id_counter = 0
history_item_id_counter = 0
semantic_data_id_counter = 0

used_run_ids = []
used_global_variable_ids = []


def generate_state_name_id():
    """Generates an id for a state

    It simply uses a global counter that is increased each time. It is intended for the name of a new state.

    :return: a new state machine id
    """
    global state_name_counter
    state_name_counter += 1
    return state_name_counter


def generate_state_machine_id():
    """
    Generates an id for a state machine. It simply uses a global counter that is increased each time.
    :return: a new state machine id
    """
    global state_machine_id_counter
    state_machine_id_counter += 1
    return state_machine_id_counter


# As the id generation for the next functions is identical to the one above, the comments are omitted.
def generate_transition_id():
    global transition_id_counter
    transition_id_counter += 1
    return transition_id_counter


def generate_data_flow_id():
    global data_flow_id_counter
    data_flow_id_counter += 1
    return data_flow_id_counter


# outcome id will start with value 0
def generate_outcome_id(used_outcome_ids):
    outcome_id_counter = -1
    while True:
        outcome_id_counter += 1
        if outcome_id_counter not in used_outcome_ids:
            break
    return outcome_id_counter


def generate_data_port_id(used_data_port_ids):
    """ Create a new and unique data port id

    :param list used_data_port_ids: Handed list of ids already in use
    :rtype: int
    :return: data_port_id
    """
    data_port_id_counter = -1
    while True:
        data_port_id_counter += 1
        if data_port_id_counter not in used_data_port_ids:
            break
    return data_port_id_counter


def generate_semantic_data_key(used_semantic_keys):
    """ Create a new and unique semantic data key

    :param list used_semantic_keys: Handed list of keys already in use
    :rtype: str
    :return: semantic_data_id
    """
    semantic_data_id_counter = -1
    while True:
        semantic_data_id_counter += 1
        if "semantic data key " + str(semantic_data_id_counter) not in used_semantic_keys:
            break
    return "semantic data key " + str(semantic_data_id_counter)


def generate_script_id():
    global script_id_counter
    script_id_counter += 1
    return script_id_counter


def run_id_generator():
    global run_id_counter
    run_id_counter += 1
    final_run_id = experiment_id + ".run_id." + '%020d' % run_id_counter
    return final_run_id


def history_item_id_generator():
    global history_item_id_counter
    history_item_id_counter += 1
    final_id = experiment_id + ".history_item_id." + '%020d' % history_item_id_counter
    return final_id


def state_id_generator(size=STATE_ID_LENGTH, chars=string.ascii_uppercase, used_state_ids=None):
    """ Create a new and unique state id

    Generates an id for a state. It randomly samples from random ascii uppercase letters size times
    and concatenates them. If the id already exists it draws a new one.

    :param size: the length of the generated keys
    :param chars: the set of characters a sample draws from
    :param list used_state_ids: Handed list of ids already in use
    :rtype: str
    :return: new_state_id
    """
    new_state_id = ''.join(random.choice(chars) for x in range(size))
    while used_state_ids is not None and new_state_id in used_state_ids:
        new_state_id = ''.join(random.choice(chars) for x in range(size))
    return new_state_id


def global_variable_id_generator(size=10, chars=string.ascii_uppercase):
    """ Create a new and unique global variable id

    Generates an id for a global variable. It randomly samples from random ascii uppercase letters size times
    and concatenates them. If the id already exists it draws a new one.

    :param size: the length of the generated keys
    :param chars: the set of characters a sample draws from
    """
    new_global_variable_id = ''.join(random.choice(chars) for x in range(size))
    while new_global_variable_id in used_global_variable_ids:
        new_global_variable_id = ''.join(random.choice(chars) for x in range(size))
    used_global_variable_ids.append(new_global_variable_id)
    return new_global_variable_id
