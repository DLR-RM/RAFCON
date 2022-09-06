import threading

from queue import Queue

from rafcon.core.config import global_config
from rafcon.core.execution.consumers.file_system_consumer import FileSystemConsumer

from rafcon.utils import plugins
from rafcon.utils import log
logger = log.get_logger(__name__)


class ExecutionHistoryConsumerManager(object):
    """ A class for managing all consumers including the consumer plugins
    """

    FILE_SYSTEM_CONSUMER_NAME = "file_system_consumer"

    def __init__(self, root_state_name):
        self.consumers = dict()
        # Queue with infinite space
        self.execution_history_item_queue = Queue()
        self.condition = threading.Condition()
        self.interrupt = False
        self._consumers_exist = False
        self._file_system_consumer_exists = False
        if global_config.get_config_value("FILE_SYSTEM_EXECUTION_HISTORY_ENABLE", False):
            self.register_consumer(self.FILE_SYSTEM_CONSUMER_NAME, FileSystemConsumer(root_state_name))
            self._file_system_consumer_exists = True
        plugins.run_hook("register_execution_history_consumer", self)
        # Only have one thread here that will call the notify function of each consumer
        # The advantage it that the consumer authors don't have to care about threading
        # and don't have to care about when an item is popped from the queue
        self.worker_thread = threading.Thread(target=self._feed_consumers)
        self.worker_thread.start()

    @property
    def consumers_exist(self):
        return self._consumers_exist

    @consumers_exist.setter
    def consumers_exist(self, value):
        self._consumers_exist = value

    @property
    def file_system_consumer_exists(self):
        """ Check if the file system consumer is activated
        """
        return self._file_system_consumer_exists

    def get_file_system_consumer_file_name(self):
        """ Get the filename of the shelve
        """
        if self.FILE_SYSTEM_CONSUMER_NAME in self.consumers.keys():
            return self.consumers[self.FILE_SYSTEM_CONSUMER_NAME].filename

    def stop_consumers(self):
        """ Stop the working thread and unregister all consumers
        """
        self.stop_worker_thread()
        for consumer in self.consumers.values():
            self.unregister_consumer(consumer)

    def stop_worker_thread(self):
        """ Stop the working thread by setting interrupt to true
        """
        self.interrupt = True
        with self.condition:
            self.condition.notify()
        self.worker_thread.join()

    def register_consumer(self, consumer_name, consumer):
        """ Register a specific consumer

        :param consumer_name: the consumer name
        :param consumer: an instance of the consumer
        """
        self.consumers_exist = True
        self.consumers[consumer_name] = consumer
        consumer.register()

    def add_history_item_to_queue(self, execution_history_item):
        """ Add execution history item to the dedicated queue of all consumers
        and notify their condition variables

        :param execution_history_item: the execution history item
        """
        with self.condition:
            self.execution_history_item_queue.put(execution_history_item)
            self.condition.notify()

    def _feed_consumers(self):
        """ Distribute the available execution history items to the consumers
        """
        while not self.interrupt:
            with self.condition:
                self.condition.wait_for(lambda: not self.execution_history_item_queue.empty() or self.interrupt)
                while not self.execution_history_item_queue.empty():
                    next_execution_history_event = self.execution_history_item_queue.get(block=False)
                    self._notifyConsumers(next_execution_history_event)

    def _notifyConsumers(self, execution_history_event):
        """ Add execution history item to the dedicated queue of all consumers
        """
        for client in self.consumers.values():
            client.enqueue(execution_history_event)

    def unregister_consumer(self, consumer):
        """ Unegister a specific consumer

        :param consumer: an instance of the consumer
        """
        consumer.stop()
        # Unregister the consumer after stopping the thread to avoid e.g., writing on the closed resources
        consumer.unregister()
