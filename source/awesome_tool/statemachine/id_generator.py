"""
.. module:: id_generator
   :platform: Unix, Windows
   :synopsis: A module to generate different kinds of state machine ids

.. moduleauthor:: TODO


"""

import string
import random


state_machine_id_counter = 0
transition_id_counter = 0
data_flow_id_counter = 0
# outcome 0 is success, outcome -1 is aborted and outcome -2 is preempted
outcome_id_counter = 0
data_port_id_counter = 0
script_id_counter = 0
external_module_id_counter = 0

used_state_ids = []
used_global_variable_ids = []


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

# outcome id will start with value 1
def generate_outcome_id():
    global outcome_id_counter
    outcome_id_counter += 1
    return outcome_id_counter


def generate_data_port_id():
    global data_port_id_counter
    data_port_id_counter += 1
    return data_port_id_counter


def generate_script_id():
    global script_id_counter
    script_id_counter += 1
    return script_id_counter


def generate_external_module_id():
    global external_module_id_counter
    external_module_id_counter += 1
    return external_module_id_counter


def state_id_generator(size=6, chars=string.ascii_uppercase):
    """
    Generates an id for a state. It randomly samples from random ascii uppercase letters size times
    and concatenates them. If the id already exists it draws a new one.
    :param size: the length of the generated keys
    :param chars: the set of characters a sample draws from
    """
    new_state_id = ''.join(random.choice(chars) for x in range(size))
    while new_state_id in used_state_ids:
        new_state_id = ''.join(random.choice(chars) for x in range(size))
    used_state_ids.append(new_state_id)
    return new_state_id


def global_variable_id_generator(size=10, chars=string.ascii_uppercase):
    """
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