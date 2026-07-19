"""
.. module:: web_streams
   :synopsis: Log record and execution history streaming for web clients

Both streams batch their events over a short window (like the state status batching in
:mod:`rafcon.network.core_observer`) and broadcast them with the ``flavor`` gate set to ``"web"``,
so GTK remote GUIs never receive them.
"""

import logging
import threading
from collections import deque

from rafcon.core.execution.consumers.abstract_execution_history_consumer import AbstractExecutionHistoryConsumer
from rafcon.network import protocol

BATCH_INTERVAL = 0.05
LOG_BACKLOG_SIZE = 500

#: name under which the history consumer shim is registered in rafcon.utils.plugins.plugin_dict
PLUGIN_NAME = "rafcon_network_web"
CONSUMER_NAME = "websocket_consumer"


class _Batcher(object):
    """Collects items and flushes them to a callback after a short delay, from a timer thread"""

    def __init__(self, flush_callback, interval=BATCH_INTERVAL):
        self._flush_callback = flush_callback
        self._interval = interval
        self._lock = threading.Lock()
        self._items = []
        self._timer = None

    def add(self, item):
        with self._lock:
            self._items.append(item)
            if self._timer is None:
                self._timer = threading.Timer(self._interval, self._flush)
                self._timer.daemon = True
                self._timer.start()

    def _flush(self):
        with self._lock:
            items = self._items
            self._items = []
            self._timer = None
        if items:
            self._flush_callback(items)

    def stop(self):
        """Cancel the pending timer and flush whatever is queued (nothing may be lost
        when a consumer is unregistered right after a short run)"""
        with self._lock:
            if self._timer:
                self._timer.cancel()
                self._timer = None
        self._flush()


class WebSocketLogHandler(logging.Handler):
    """Forwards log records to web clients and keeps a backlog for the initial sync

    :param broadcast: callable taking a message dict, safe to call from any thread
    """

    def __init__(self, broadcast):
        super(WebSocketLogHandler, self).__init__()
        self._broadcast = broadcast
        self._backlog = deque(maxlen=LOG_BACKLOG_SIZE)
        self._batcher = _Batcher(self._flush_records)
        self._emitting = threading.local()

    def emit(self, record):
        # guard against recursion: a log emitted while broadcasting must not re-enter
        if getattr(self._emitting, "active", False):
            return
        self._emitting.active = True
        try:
            entry = {
                "ts": record.created,
                "level": record.levelname,
                "logger": record.name,
                "message": record.getMessage(),
            }
            self._backlog.append(entry)
            self._batcher.add(entry)
        except (KeyboardInterrupt, SystemExit):
            raise
        except Exception:
            self.handleError(record)
        finally:
            self._emitting.active = False

    def _flush_records(self, records):
        self._broadcast({"type": protocol.LOG_RECORD,
                         "payload": {"records": records},
                         "flavor": protocol.FLAVOR_WEB})

    def backlog(self):
        return list(self._backlog)

    def close(self):
        self._batcher.stop()
        super(WebSocketLogHandler, self).close()


#: loggers whose records are streamed to web clients (same set the GTK log console shows)
STREAMED_LOGGERS = ("rafcon", "py")


def attach_log_handler(broadcast):
    handler = WebSocketLogHandler(broadcast)
    for logger_name in STREAMED_LOGGERS:
        logging.getLogger(logger_name).addHandler(handler)
    return handler


def detach_log_handler(handler):
    for logger_name in STREAMED_LOGGERS:
        logging.getLogger(logger_name).removeHandler(handler)
    handler.close()


class WebSocketHistoryConsumer(AbstractExecutionHistoryConsumer):
    """Streams execution history items to web clients"""

    def __init__(self, broadcast):
        self._broadcast = broadcast
        self._batcher = _Batcher(self._flush_items)
        super(WebSocketHistoryConsumer, self).__init__()

    def register(self):
        pass

    def unregister(self):
        self._batcher.stop()

    def consume(self, execution_history_item):
        try:
            record = execution_history_item.to_dict(pickled=False)
        except Exception:
            return
        record["state_machine_id"] = self._resolve_state_machine_id(execution_history_item)
        self._batcher.add(record)

    @staticmethod
    def _resolve_state_machine_id(item):
        try:
            state_machine = item.state_reference.get_state_machine()
            if state_machine is not None:
                return state_machine.state_machine_id
        except Exception:
            pass
        return None

    def _flush_items(self, items):
        self._broadcast({"type": protocol.EXECUTION_HISTORY_EVENT,
                         "payload": {"items": items},
                         "flavor": protocol.FLAVOR_WEB})


class _HistoryConsumerHooks(object):
    """Plugin hook shim; every new run's consumer manager registers a fresh consumer

    Note: registering a consumer sets ``consumers_exist`` on the manager, which enables
    execution history item generation while the network server runs — the same overhead
    the GTK GUI already incurs.
    """

    def __init__(self, broadcast):
        self._broadcast = broadcast

    def register_execution_history_consumer(self, consumer_manager):
        consumer_manager.register_consumer(CONSUMER_NAME, WebSocketHistoryConsumer(self._broadcast))


class _HistoryConsumerShim(object):
    """Minimal object satisfying the plugin module interface (a ``hooks`` attribute)"""

    def __init__(self, broadcast):
        self.hooks = _HistoryConsumerHooks(broadcast)


def register_history_consumer_plugin(broadcast):
    from rafcon.utils import plugins
    plugins.plugin_dict[PLUGIN_NAME] = _HistoryConsumerShim(broadcast)


def unregister_history_consumer_plugin():
    from rafcon.utils import plugins
    plugins.plugin_dict.pop(PLUGIN_NAME, None)
