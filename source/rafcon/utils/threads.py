# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Sebastian Brunner <sebastian.brunner@dlr.de>

import sys

if sys.version_info[0] == 2:
    from threading import _Condition as Condition
else:
    from threading import Condition


class ReaderWriterLock:
    """
    This class is an implementation of the readers-writer solution. The resource can only be read by several threads or
    written by a single thread at the time. The writers have priority over the readers to avoid writing starvation.
    """

    class Counter:
        readers = 0
        pending_writers = 0
        is_writing = False

    class ReaderLock:
        def __init__(self, condition, counter, resource):
            self._condition = condition
            self._counter = counter
            self._resource = resource

        def __enter__(self):
            with self._condition:
                while self._counter.is_writing or self._counter.pending_writers > 0:
                    self._condition.wait()
                self._counter.readers += 1
            return self._resource

        def __exit__(self, exc_type, exc_value, traceback):
            with self._condition:
                self._counter.readers -= 1
                if self._counter.readers == 0:
                    self._condition.notifyAll()

    class WriterLock:
        def __init__(self, condition, counter, resource):
            self._condition = condition
            self._counter = counter
            self._resource = resource

        def __enter__(self):
            self._condition.acquire()
            self._counter.pending_writers += 1
            while self._counter.readers > 0 or self._counter.is_writing:
                self._condition.wait()
            self._counter.pending_writers -= 1
            self._counter.is_writing = True
            return self._resource

        def __exit__(self, exc_type, exc_value, traceback):
            self._counter.is_writing = False
            self._condition.notifyAll()
            self._condition.release()

    def __init__(self, resource):
        self._condition = Condition()
        self._counter = self.Counter()
        self._reader_lock = self.ReaderLock(self._condition, self._counter, resource)
        self._writer_lock = self.WriterLock(self._condition, self._counter, resource)

    @property
    def reader_lock(self):
        return self._reader_lock

    @property
    def writer_lock(self):
        return self._writer_lock
