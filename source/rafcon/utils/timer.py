from builtins import object
from builtins import str
import time
from timeit import default_timer as timer

from rafcon.utils import log
logger = log.get_logger(__name__)


def measure_time(func):
    def func_wrapper(*args, **kwargs):
        start = timer()
        return_value = func(*args, **kwargs)
        end = timer()
        logger.verbose("Profiler: {0} (args: {1}; kwargs: {2}); duration: {3:.3}s".format(func.__name__, str(args), str(kwargs), end - start))
        return return_value
    return func_wrapper


class Timer(object):
    # TODO make it look beautiful and cut it maybe down -> therefore description of intended behavior would be great :)
    def __init__(self, logger, name='', continues=True):
        self._logger = logger
        self.__name = name
        self.__init_time = time.time()
        self.__duration = 0.
        self.__status = 'stopped'
        self._last_start_time = self.__init_time
        self.continues = continues
        self.count = 0

    def start(self):
        # print("start", self.count + 1)
        if self.__status == 'stopped':
            pass
        else:
            # self._logger.verbose("Do not start was not stopped")
            return

        self.__status = 'running'
        self._last_start_time = time.time()
        self.count += 1
        return self.count

    def stop(self, key):
        # print("stop", key, self.count, self.count == key)
        if self.__status == 'running':
            if self.count == key:
                pass
            else:
                # self._logger.verbose("Do not stop was not started with key {0}".format(key))
                return
        else:
            self._logger.warning("Do not stop was not started")
            return
        self.__status = 'stopped'

        if self.continues:
            self.__duration += time.time() - self._last_start_time
        else:
            self.__duration = time.time() - self._last_start_time
        self._logger.verbose("{0} new duration {1} x{2}".format(self.__name, self.__duration, self.count))
        return self.__duration

    def reset(self):
        self.__duration = 0.

    @property
    def duration(self):
        return self.__duration
