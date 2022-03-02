from queue import Queue
from threading import Thread, Condition


class AbstractExecutionHistoryConsumer(object):
    """A class that should be the base for every defined consumer
    """
    def __init__(self):
        self._queue = Queue()
        self._thread = Thread(target=self.worker)
        self._condition = Condition()
        self._stop = False
        self._thread.start()

    def register(self):
        """ Override the register for the consumer to run the required procedures when a consumer starts
        """
        raise NotImplementedError("The register function has to be implemented")

    def consume(self, execution_history_item):
        """ Override the register for the consumer to run the required procedures when a consumer wants to
        consume an execution history item
        """
        raise NotImplementedError("The consume function has to be implemented")

    def unregister(self):
        """ Override the register for the consumer to run the required procedures when a consumer stops
        """
        raise NotImplementedError("The unregister function has to be implemented")

    def enqueue(self, execution_history_item):
        """ Add the execution history item to the local consumer queue

        :param execution_history_item: the execution history item
        """
        with self._condition:
            self._queue.put(execution_history_item, block=False)
            self._condition.notify()

    def worker(self):
        """ Consume the available execution history item until the thread stops
        """
        while not self._stop:
            with self._condition:
                self._condition.wait_for(lambda: not self._queue.empty() or self._stop)
                while not self._queue.empty():
                    item = self._queue.get(block=False)
                    self.consume(item)

    def stop(self):
        """ Stop the consumer thread
        """
        self._stop = True
        with self._condition:
            self._condition.notify()
        self._thread.join()
