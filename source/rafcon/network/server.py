"""
.. module:: server
   :synopsis: Websocket server that exposes a running RAFCON core to remote GUI clients

The server runs an asyncio event loop in a daemon thread next to the core. It broadcasts core
change events (fed by :class:`rafcon.network.core_observer.CoreObserver`) to all connected clients
and dispatches incoming execution commands onto the real core singletons.
"""

import asyncio
import threading

from rafcon.network import protocol
from rafcon.network.core_observer import CoreObserver

from rafcon.utils import log
logger = log.get_logger(__name__)


class RemoteServer:
    """Websocket server for remote RAFCON GUIs

    :param str host: interface to bind to
    :param int port: port to listen on
    :param int max_clients: maximum number of concurrently connected GUI clients
    """

    def __init__(self, host="0.0.0.0", port=protocol.DEFAULT_PORT, max_clients=3):
        self.host = host
        self.port = port
        self.max_clients = max_clients
        self._clients = set()
        self._loop = None
        self._thread = None
        self._started = threading.Event()
        self._observer = None
        self._websocket_server = None

    def start(self):
        """Start the server thread and wait until it accepts connections"""
        self._observer = CoreObserver(self.broadcast)
        self._thread = threading.Thread(target=self._run, name="RAFCONNetworkServer", daemon=True)
        self._thread.start()
        if not self._started.wait(5):
            raise RuntimeError("RAFCON network server did not start within 5 seconds")
        logger.info("RAFCON network server listening on {0}:{1}".format(self.host, self.port))

    def stop(self):
        if self._observer:
            self._observer.shutdown()
            self._observer = None
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(3)
            self._thread = None

    def broadcast(self, message):
        """Send a message dict to all connected clients; safe to call from any thread"""
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._async_broadcast(message), self._loop)

    def _run(self):
        import websockets
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        async def serve():
            self._websocket_server = await websockets.serve(self._handle_client, self.host, self.port)
            self._started.set()

        self._loop.run_until_complete(serve())
        try:
            self._loop.run_forever()
        finally:
            self._websocket_server.close()
            self._loop.run_until_complete(self._websocket_server.wait_closed())
            self._loop.close()

    async def _async_broadcast(self, message):
        frame = protocol.make_message(message["type"], message.get("payload"))
        for client in list(self._clients):
            try:
                await client.send(frame)
            except Exception:
                self._clients.discard(client)

    async def _handle_client(self, websocket):
        if len(self._clients) >= self.max_clients:
            await websocket.send(protocol.make_message(
                protocol.ERROR, {"code": "too_many_clients",
                                 "message": "Maximum number of {0} clients reached".format(self.max_clients)}))
            await websocket.close()
            return

        try:
            hello = protocol.parse_message(await websocket.recv())
            if hello["type"] != protocol.HELLO or \
                    hello["payload"].get("protocol_version") != protocol.PROTOCOL_VERSION:
                await websocket.send(protocol.make_message(
                    protocol.ERROR, {"code": "bad_handshake",
                                     "message": "Expected hello with protocol version {0}"
                                                "".format(protocol.PROTOCOL_VERSION)}))
                await websocket.close()
                return

            self._clients.add(websocket)
            client_name = hello["payload"].get("client_name", "unknown")
            logger.info("Remote GUI '{0}' connected ({1} client(s))".format(client_name, len(self._clients)))

            await websocket.send(protocol.make_message(
                protocol.WELCOME, {"protocol_version": protocol.PROTOCOL_VERSION}))
            await websocket.send(protocol.make_message(protocol.SYNC, await self._build_sync_payload()))

            async for frame in websocket:
                try:
                    message = protocol.parse_message(frame)
                    await self._dispatch_command(message, websocket)
                except Exception as e:
                    logger.exception("Error handling message from remote GUI")
                    await websocket.send(protocol.make_message(
                        protocol.ERROR, {"code": "command_failed", "message": str(e)}))
        except Exception:
            logger.debug("Remote GUI connection terminated")
        finally:
            self._clients.discard(websocket)
            logger.info("Remote GUI disconnected ({0} client(s) left)".format(len(self._clients)))

    async def _build_sync_payload(self):
        from rafcon.network import mirror
        import rafcon.core.singleton as core_singletons

        def build():
            manager = core_singletons.state_machine_manager
            state_machines = []
            for sm_id, state_machine in manager.state_machines.items():
                state_machines.append({
                    "state_machine_id": sm_id,
                    "path": state_machine.file_system_path,
                    "sm_zip_b64": mirror.pack_state_machine(state_machine),
                    "state_statuses": self._observer.collect_active_state_statuses(state_machine),
                })
            return {"state_machines": state_machines,
                    "execution_status": self._observer.current_execution_status_name(),
                    "active_state_machine_id": manager.active_state_machine_id}

        return await self._loop.run_in_executor(None, build)

    async def _dispatch_command(self, message, websocket):
        payload = message["payload"]
        if message["type"] == protocol.EXECUTION_COMMAND:
            command = payload.get("command")
            if command not in protocol.EXECUTION_COMMANDS:
                raise ValueError("Unknown execution command '{0}'".format(command))
            await self._loop.run_in_executor(None, self._run_execution_command, command, payload)
        elif message["type"] == protocol.OPEN_STATE_MACHINE:
            await self._loop.run_in_executor(None, self._open_state_machine, payload["path"])
        else:
            logger.warning("Ignoring unknown message type '{0}' from remote GUI".format(message["type"]))

    @staticmethod
    def _run_execution_command(command, payload):
        import rafcon.core.singleton as core_singletons
        engine = core_singletons.state_machine_execution_engine
        state_machine_id = payload.get("state_machine_id")
        state_path = payload.get("state_path")
        if command == "start":
            engine.start(state_machine_id=state_machine_id, start_state_path=state_path)
        elif command == "step_mode":
            engine.step_mode(state_machine_id=state_machine_id)
        elif command == "run_to_selected_state":
            engine.run_to_selected_state(state_path, state_machine_id=state_machine_id)
        elif command in ("run_selected_state", "run_only_selected_state"):
            getattr(engine, command)(start_state_path=state_path, state_machine_id=state_machine_id)
        else:  # pause, stop, step_into, step_over, step_out, backward_step
            getattr(engine, command)()

    @staticmethod
    def _open_state_machine(path):
        import rafcon.core.singleton as core_singletons
        from rafcon.core.storage import storage
        state_machine = storage.load_state_machine_from_path(path)
        core_singletons.state_machine_manager.add_state_machine(state_machine)
