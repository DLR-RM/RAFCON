from threading import Condition


class ReaderWriterLock:

    class ReaderLock:
        def __init__(self, condition):
            self._readers = 0
            self._condition = condition

        def __enter__(self):
            with self._condition:
                self._readers += 1

        def __exit__(self, exc_type, exc_value, traceback):
            with self._condition:
                self._readers -= 1
                if self._readers == 0:
                    self._condition.notifyAll()

        @property
        def readers(self):
            return self._readers

    class WriterLock:
        def __init__(self, condition, reader_lock):
            self._reader_lock = reader_lock
            self._condition = condition

        def __enter__(self):
            self._condition.acquire()
            self._condition.wait_for(lambda: self._reader_lock.readers == 0)

        def __exit__(self, exc_type, exc_value, traceback):
            self._condition.release()

    def __init__(self):
        self._condition = Condition()
        self._readers = 0
        self._reader_lock = self.ReaderLock(self._condition)
        self._writer_lock = self.WriterLock(self._condition, self._reader_lock)

    @property
    def reader_lock(self):
        return self._reader_lock

    @property
    def writer_lock(self):
        return self._writer_lock
