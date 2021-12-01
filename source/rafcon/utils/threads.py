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

from threading import RLock

if sys.version_info[0] == 2:
    from threading import _Condition as Condition
else:
    from threading import Condition


class ReaderWriterLock:
    """
    This class is an implementation of the readers-writer lock solution. The resource can be read by several threads or
    written by a single thread at the time. The writers have priority over the readers to avoid writing starvation.
    """

    class State:
        def __init__(self):
            self.lock = RLock()
            self.reader_condition = Condition(self.lock)
            self.writer_conditions = []
            self.readers = 0
            self.pending_writers = 0
            self.is_writing = False

    class ReaderLock:
        def __init__(self, state, resource):
            self._state = state
            self._resource = resource

        def __enter__(self):
            with self._state.lock:
                while self._state.is_writing or self._state.pending_writers > 0:
                    self._state.reader_condition.wait()
                self._state.readers += 1
            return self._resource

        def __exit__(self, exc_type, exc_value, traceback):
            with self._state.lock:
                self._state.readers -= 1
                ReaderWriterLock.notify(self._state)

    class WriterLock:
        def __init__(self, state, resource):
            self._state = state
            self._resource = resource

        def __enter__(self):
            with self._state.lock:
                condition = Condition(self._state.lock)
                self._state.writer_conditions.append(condition)
                while self._state.readers > 0 or self._state.is_writing:
                    self._state.pending_writers += 1
                    condition.wait()
                    self._state.pending_writers -= 1
                self._state.is_writing = True
            return self._resource

        def __exit__(self, exc_type, exc_value, traceback):
            with self._state.lock:
                self._state.is_writing = False
                ReaderWriterLock.notify(self._state)

    def __init__(self, resource):
        self._state = self.State()
        self._reader_lock = self.ReaderLock(self._state, resource)
        self._writer_lock = self.WriterLock(self._state, resource)

    @property
    def reader_lock(self):
        return self._reader_lock

    @property
    def writer_lock(self):
        return self._writer_lock

    @staticmethod
    def notify(state):
        if state.pending_writers > 0 and state.readers == 0:
            condition = state.writer_conditions.pop(0)
            condition.notify()
        elif state.pending_writers == 0:
            state.reader_condition.notify_all()
