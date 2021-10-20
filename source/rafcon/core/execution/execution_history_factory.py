
from rafcon.core.config import global_config

from rafcon.core.execution.in_memory_execution_history import InMemoryExecutionHistory
from rafcon.core.execution.base_execution_history import BaseExecutionHistory

from rafcon.utils import log
logger = log.get_logger(__name__)


class ExecutionHistoryFactory(object):

    def __init__(self):
        pass

    @staticmethod
    def get_execution_history(initial_prev=None, root_state_name="", consumer_manager=None):
        if global_config.get_config_value("EXECUTION_HISTORY_ENABLE", True):
            return InMemoryExecutionHistory(initial_prev=initial_prev,
                                            root_state_name=root_state_name,
                                            consumer_manager=consumer_manager)
        else:
            return BaseExecutionHistory(initial_prev=initial_prev,
                                        root_state_name=root_state_name,
                                        consumer_manager=consumer_manager)
