import yaml
import os

from awesome_tool.statemachine.storage.storage import StateMachineStorage

class StateHelper(object):

    def __init__(self):
        pass

    @staticmethod
    def get_state_copy(source_state):
        state_copy = None
        local_storage = StateMachineStorage("/tmp/DFC/state_copy_tmp_folder")

        if not os.path.exists(local_storage.base_path):
            os.makedirs(local_storage.base_path)
        local_storage.save_state_recursively(source_state, "")
        state_copy = local_storage.load_state_from_yaml(os.path.join(local_storage.base_path, source_state.state_id))
        print state_copy.get_path()
        state_copy.script.reset_script(state_copy.get_path())

        return state_copy
