
from rafcon.core.config import global_config

from rafcon.core.execution.in_memory_execution_history import InMemoryExecutionHistory
from rafcon.core.execution.base_execution_history import BaseExecutionHistory

from rafcon.utils import log
logger = log.get_logger(__name__)


class ExecutionHistoryFactory(object):
    """ A factory class for creating an instance of BaseExecutionHistory or InMemoryExecutionHistory
    """

    def __init__(self):
        pass

    @staticmethod
    def get_execution_history(initial_prev=None, root_state_name="", consumer_manager=None):
        """ Create an instance of a InMemoryExecutionHistory or BaseExecutionHistory

        :param initial_prev: the initial previous history item
        :param root_state_name: the root state name
        :param consumer_manager: the consumer manager

        :return: an instance of BaseExecutionHistory or InMemoryExecutionHistory
        """
        if global_config.get_config_value("IN_MEMORY_EXECUTION_HISTORY_ENABLE", True):
            return InMemoryExecutionHistory(initial_prev=initial_prev,
                                            root_state_name=root_state_name,
                                            consumer_manager=consumer_manager)
        else:
            return BaseExecutionHistory(initial_prev=initial_prev,
                                        root_state_name=root_state_name,
                                        consumer_manager=consumer_manager)
