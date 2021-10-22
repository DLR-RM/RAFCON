from queue import Queue
from threading import Thread, Condition


class AbstractExecutionHistoryConsumer(object):
    def __init__(self):
        self._queue = Queue()
        self._thread = Thread(target=self.worker)
        self._condition = Condition()
        self._stop = False
        self._thread.start()

    def register(self):
        raise NotImplementedError("The register function has to be implemented")

    def consume(self, execution_history_item):
        raise NotImplementedError("The consume function has to be implemented")

    def unregister(self):
        raise NotImplementedError("The unregister function has to be implemented")

    def enqueue(self, execution_history_item):
        with self._condition:
            self._queue.put(execution_history_item, block=False)
            self._condition.notify()

    def worker(self):
        while not self._stop:
            with self._condition:
                self._condition.wait_for(lambda: not self._queue.empty() or self._stop)
                while not self._queue.empty():
                    item = self._queue.get(block=False)
                    self.consume(item)

    def stop(self):
        self._stop = True
        with self._condition:
            self._condition.notify()
        self._thread.join()
