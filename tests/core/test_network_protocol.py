import json

import pytest

from tests import utils as testing_utils

from rafcon.network import protocol


def test_message_round_trip():
    frame = protocol.make_message(protocol.EXECUTION_COMMAND, {"command": "start", "state_machine_id": 1}, seq=5)
    message = protocol.parse_message(frame)
    assert message["type"] == protocol.EXECUTION_COMMAND
    assert message["seq"] == 5
    assert message["payload"] == {"command": "start", "state_machine_id": 1}


def test_message_defaults_and_invalid_envelope():
    message = protocol.parse_message(json.dumps({"type": protocol.HELLO}))
    assert message["seq"] == 0
    assert message["payload"] == {}

    with pytest.raises(ValueError):
        protocol.parse_message(json.dumps(["no", "envelope"]))


def test_state_machine_pack_unpack(caplog):
    testing_utils.initialize_environment_core()
    try:
        from rafcon.core.storage import storage
        from rafcon.network import mirror

        sm_path = testing_utils.get_test_sm_path(
            testing_utils.os.path.join("unit_test_state_machines", "simple_states_without_data_port"))
        state_machine = storage.load_state_machine_from_path(sm_path)

        sm_zip_b64 = mirror.pack_state_machine(state_machine)
        unpacked_sm = mirror.unpack_state_machine(sm_zip_b64, state_machine_id=42)

        assert unpacked_sm.state_machine_id == 42
        assert unpacked_sm.root_state.name == state_machine.root_state.name
        assert set(unpacked_sm.root_state.states.keys()) == set(state_machine.root_state.states.keys())
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


if __name__ == '__main__':
    pytest.main([__file__, '-xvs'])
