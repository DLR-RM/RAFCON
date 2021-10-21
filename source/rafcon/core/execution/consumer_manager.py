import threading

from queue import Queue

from rafcon.core.config import global_config
from rafcon.core.execution.file_system_consumer import FileSystemConsumer

from rafcon.utils import log
logger = log.get_logger(__name__)

FILE_SYSTEM_CONSUMER_NAME = "file_system_consumer"


class ExecutionHistoryConsumerManager(object):

    def __init__(self, root_state_name):
        self.consumers = dict()
        # Queue with infinite space
        self.execution_history_item_queue = Queue()
        self.condition = threading.Condition()
        self.interrupt = False
        self._file_system_consumer_exists = False
        if global_config.get_config_value("EXECUTION_LOG_ENABLE", False):
            self.register_consumer(FILE_SYSTEM_CONSUMER_NAME, FileSystemConsumer(root_state_name))
            self._file_system_consumer_exists = True
        # Only have one thread here that will call the notify function of each consumer
        # The advantage it that the consumer authors don't have to care about threading
        # and don't have to care about when an item is popped from the queue
        self.worker_thread = threading.Thread(target=self._feed_consumers)
        self.worker_thread.start()

    @property
    def file_system_consumer_exists(self):
        return self._file_system_consumer_exists

    def get_file_system_consumer_file_name(self):
        if FILE_SYSTEM_CONSUMER_NAME in self.consumers.keys():
            return self.consumers[FILE_SYSTEM_CONSUMER_NAME].filename

    def stop_consumers(self):
        self.stop_worker_thread()
        for consumer in self.consumers.values():
            self.unregister_consumer(consumer)

    def stop_worker_thread(self):
        self.condition.acquire()
        self.interrupt = True
        self.condition.notify()
        self.condition.release()
        self.worker_thread.join()

    def register_consumer(self, consumer_name, consumer):
        self.consumers[consumer_name] = consumer
        consumer.register()

    def add_history_item_to_queue(self, execution_history_item):
        self.condition.acquire()
        self.execution_history_item_queue.put(execution_history_item)
        self.condition.notify()
        self.condition.release()

    def _feed_consumers(self):
        while not self.interrupt:
            self.condition.acquire()
            while self.execution_history_item_queue.empty() or not self.interrupt:
                self.condition.wait()
            if not self.execution_history_item_queue.empty():
                next_execution_history_event = self.execution_history_item_queue.get()
                self._notifyConsumers(next_execution_history_event)
            self.condition.release()

    def _notifyConsumers(self, execution_history_event):
        for client in self.consumers.values():
            client.consume(execution_history_event)

    def unregister_consumer(self, consumer):
        consumer.unregister()
