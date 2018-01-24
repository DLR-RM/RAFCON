import hashlib

from rafcon.core.states.execution_state import ExecutionState
from rafcon.utils.hashable import Hashable


def test_update_hash_from_dict():
    hash1 = hashlib.sha256()
    hash2 = hashlib.sha256()

    assert hash1.hexdigest() == hash2.hexdigest()

    d1 = {
        'a': {1, 2, 3},
        'b': [1, 2, 3],
        'c': ("test", )
    }

    d2 = {
        'b': [1, 2, 3],
        'a': {3, 2, 1},
        'c': ("test", )
    }

    Hashable.update_hash_from_dict(hash1, d1)
    Hashable.update_hash_from_dict(hash2, d2)

    assert hash1.hexdigest() == hash2.hexdigest()

    s1 = {4, 5, 6}
    s2 = {6, 4, 5}

    Hashable.update_hash_from_dict(hash1, s1)
    Hashable.update_hash_from_dict(hash2, s2)

    assert hash1.hexdigest() == hash2.hexdigest()

    l1 = [1, 2]
    l2 = [2, 1]

    Hashable.update_hash_from_dict(hash1, l1)
    Hashable.update_hash_from_dict(hash2, l2)

    assert hash1.hexdigest() != hash2.hexdigest()


def test_state_hash():
    state1 = ExecutionState('ES', state_id="12345")
    state2 = ExecutionState('ES', state_id="12345")

    state1.add_output_data_port("out1", "int", data_port_id=1)
    state1.add_output_data_port("out2", "int", data_port_id=2)
    state1.add_input_data_port("in1", "int", data_port_id=3)
    state1.add_input_data_port("in2", "int", data_port_id=4)
    state1.add_outcome("o1", outcome_id=1)
    state1.add_outcome("o2", outcome_id=2)

    state2.add_outcome("o2", outcome_id=2)
    state2.add_outcome("o1", outcome_id=1)
    state2.add_input_data_port("in2", "int", data_port_id=4)
    state2.add_input_data_port("in1", "int", data_port_id=3)
    state2.add_output_data_port("out2", "int", data_port_id=2)
    state2.add_output_data_port("out1", "int", data_port_id=1)

    hash1 = hashlib.sha256()
    hash2 = hashlib.sha256()

    assert hash1.hexdigest() == hash2.hexdigest()

    Hashable.update_hash_from_dict(hash1, state1)
    Hashable.update_hash_from_dict(hash2, state2)

    assert hash1.hexdigest() == hash2.hexdigest()
