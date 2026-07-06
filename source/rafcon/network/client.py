"""
.. module:: client
   :synopsis: Websocket client connecting a (GUI) process to a remote RAFCON core

The client is GTK-free: incoming server events are passed to an ``event_dispatcher`` callable from
the network thread. GUI integrations must marshal onto the GTK main loop themselves (see
:mod:`rafcon.network.gui_integration`). One :class:`RemoteClient` instance represents one
connection to one core, so multiple instances can be used to attach to several cores later.
"""

import asyncio
import threading

from rafcon.network import protocol

from rafcon.utils import log
logger = log.get_logger(__name__)

RECONNECT_INTERVAL = 2.0


class RemoteClient:
    """Client connection to a remote RAFCON core

    :param str url: websocket url of the core server, e.g. ``ws://localhost:9999``
    :param event_dispatcher: callable taking a message dict, called from the network thread
    :param str client_name: name announced to the server in the handshake
    """

    def __init__(self, url, event_dispatcher, client_name="rafcon-gui"):
        self.url = url
        self.client_name = client_name
        self._event_dispatcher = event_dispatcher
        self._loop = None
        self._thread = None
        self._websocket = None
        self._stopped = False
        self.connected = threading.Event()

    def start(self):
        """Start the client thread; it keeps reconnecting until :meth:`stop` is called"""
        self._thread = threading.Thread(target=self._run, name="RAFCONNetworkClient", daemon=True)
        self._thread.start()

    def stop(self):
        self._stopped = True
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(3)
            self._thread = None

    def send_execution_command(self, command, state_machine_id=None, state_path=None):
        self._send(protocol.EXECUTION_COMMAND, {"command": command,
                                                "state_machine_id": state_machine_id,
                                                "state_path": state_path})

    def open_state_machine(self, path):
        """Ask the core to open a state machine from a path on the core host"""
        self._send(protocol.OPEN_STATE_MACHINE, {"path": path})

    def _send(self, message_type, payload):
        if not self.connected.is_set():
            logger.warning("Not connected to remote core - dropping '{0}' message".format(message_type))
            return
        frame = protocol.make_message(message_type, payload)
        asyncio.run_coroutine_threadsafe(self._websocket.send(frame), self._loop)

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.create_task(self._connect_loop())
        try:
            self._loop.run_forever()
        finally:
            pending = asyncio.all_tasks(self._loop)
            for task in pending:
                task.cancel()
            if pending:
                self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            self._loop.close()

    async def _connect_loop(self):
        import websockets
        while not self._stopped:
            try:
                async with websockets.connect(self.url, max_size=None) as websocket:
                    self._websocket = websocket
                    await websocket.send(protocol.make_message(
                        protocol.HELLO, {"client_name": self.client_name,
                                         "protocol_version": protocol.PROTOCOL_VERSION}))
                    welcome = protocol.parse_message(await websocket.recv())
                    if welcome["type"] != protocol.WELCOME:
                        logger.error("Remote core rejected connection: {0}".format(welcome["payload"]))
                        break
                    logger.info("Connected to remote core at {0}".format(self.url))
                    self.connected.set()
                    async for frame in websocket:
                        try:
                            self._event_dispatcher(protocol.parse_message(frame))
                        except Exception:
                            logger.exception("Error dispatching event from remote core")
            except Exception as e:
                logger.warning("Connection to remote core failed ({0}), retrying in {1}s"
                               "".format(e, RECONNECT_INTERVAL))
            finally:
                self.connected.clear()
                self._websocket = None
            if not self._stopped:
                await asyncio.sleep(RECONNECT_INTERVAL)
        self._loop.stop()
