# singleton elements
import rafcon.core.singleton
from rafcon.core.storage import storage as global_storage
import rafcon.utils.execution_log as log_helper

# test environment elements
import pytest
import testing_utils
import os


def test_execution_log(caplog):
    try:
        testing_utils.initialize_environment_core(
            core_config={'EXECUTION_LOG_ENABLE': True,
                         'EXECUTION_LOG_PATH': testing_utils.get_unique_temp_path()+'/test_execution_log'})

        state_machine = global_storage.load_state_machine_from_path(
            testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines",
                                                        "execution_file_log_test")))

        rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
        rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
        rafcon.core.singleton.state_machine_execution_engine.start()
        rafcon.core.singleton.state_machine_execution_engine.join()

        import shelve
        import json
        ss = shelve.open(state_machine.get_last_execution_log_filename())

        assert len(ss) == 36

        start, next, concurrent, hierarchy, collapsed_items = log_helper.log_to_collapsed_structure(ss)

        prod1_id = [k for k, v in collapsed_items.items() if v['state_name'] == 'MakeProd1' ][0]
        prod1 = collapsed_items[prod1_id]
        assert prod1['scoped_data_ins']['product'] == 2
        assert prod1['scoped_data_outs']['product'] == 2
        assert prod1['outcome_name'] == 'success'

        prod2_id = [k for k, v in collapsed_items.items() if v['state_name'] == 'MakeProd2' ][0]
        prod2 = collapsed_items[prod2_id]
        assert prod2['data_ins']['input_1'] == 0
        assert prod2['data_outs']['output_1'] == 3
        assert prod2['scoped_data_ins']['product'] == 1
        assert prod2['scoped_data_outs']['product'] == 1

        start_states = [k for k, v in collapsed_items.items() if v['state_name'] == 'Start' and v['state_type'] == 'ExecutionState']
        assert len(start_states) == 3 # Start state is executed three times until state machine is done
        start_id = start_states[0]
        start_item = collapsed_items[start_id]
        assert 'Starts the factory' in start_item['description']

        df = log_helper.log_to_DataFrame(ss)
        all_starts = df.groupby('state_name').get_group('Start')
        assert len(all_starts) == 3
        assert list(all_starts['outcome_name']) == ['success', 'success', 'done']

        rafcon.core.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog, expected_warnings=0, expected_errors=0)

if __name__ == '__main__':
    pytest.main([__file__])
