"""
.. module:: id_generator
   :platform: Unix, Windows
   :synopsis: A module to generate different kinds of state machine ids

.. moduleauthor:: TODO


"""

import string
import random


transition_id_counter = 0
data_flow_id_counter = 0
# outcome 0 is success, outcome 1 is aborted and outcome 2 is preempted
outcome_id_counter = 2
script_id_counter = 0
external_module_id_counter = 0

used_state_ids = []


def generate_transition_id():
    global transition_id_counter
    transition_id_counter += 1
    return transition_id_counter


def generate_data_flow_id():
    global data_flow_id_counter
    data_flow_id_counter += 1
    return data_flow_id_counter


#outcome id will start with value 2
def generate_outcome_id():
    global outcome_id_counter
    outcome_id_counter += 1
    return outcome_id_counter


def generate_script_id():
    global script_id_counter
    script_id_counter += 1
    return script_id_counter


def generate_external_module_id():
    global external_module_id_counter
    external_module_id_counter += 1
    return external_module_id_counter


def state_id_generator(size=6, chars=string.ascii_uppercase + string.digits):
    new_state_id = ''.join(random.choice(chars) for x in range(size))
    while new_state_id in used_state_ids:
        new_state_id = ''.join(random.choice(chars) for x in range(size))
    used_state_ids.append(new_state_id)
    return new_state_id

def global_variable_id_generator(size=10, chars=string.ascii_uppercase + string.digits):
    return ''.join(random.choice(chars) for x in range(size))