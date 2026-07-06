import asyncio
import os
import socket

import pytest

from tests import utils as testing_utils

from rafcon.network import protocol


def get_free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


async def connect_and_handshake(port, client_name="test-client"):
    import websockets
    websocket = await websockets.connect("ws://127.0.0.1:{0}".format(port), max_size=None)
    await websocket.send(protocol.make_message(
        protocol.HELLO, {"client_name": client_name, "protocol_version": protocol.PROTOCOL_VERSION}))
    welcome = protocol.parse_message(await asyncio.wait_for(websocket.recv(), 5))
    assert welcome["type"] == protocol.WELCOME
    sync = protocol.parse_message(await asyncio.wait_for(websocket.recv(), 5))
    assert sync["type"] == protocol.SYNC
    return websocket, sync


async def wait_for_message_of_type(websocket, message_type, timeout=15):
    """Read frames until one of the requested type arrives; returns its payload"""
    async def read():
        while True:
            message = protocol.parse_message(await websocket.recv())
            if message["type"] == message_type:
                return message["payload"]
    return await asyncio.wait_for(read(), timeout)


def test_remote_server_sync_and_execution_control(caplog):
    testing_utils.initialize_environment_core()
    server = None
    try:
        import rafcon.core.singleton as core_singletons
        from rafcon.core.storage import storage
        from rafcon.network.server import RemoteServer

        sm_path = testing_utils.get_test_sm_path(
            os.path.join("unit_test_state_machines", "test_custom_entry_point"))
        state_machine = storage.load_state_machine_from_path(sm_path)
        core_singletons.state_machine_manager.add_state_machine(state_machine)

        port = get_free_port()
        server = RemoteServer(host="127.0.0.1", port=port, max_clients=2)
        server.start()

        async def scenario():
            # two clients can connect and both receive the open state machine in their sync
            websocket_1, sync_1 = await connect_and_handshake(port, "client-1")
            websocket_2, sync_2 = await connect_and_handshake(port, "client-2")
            for sync in (sync_1, sync_2):
                sm_infos = sync["payload"]["state_machines"]
                assert len(sm_infos) == 1
                assert sm_infos[0]["state_machine_id"] == state_machine.state_machine_id
                assert sm_infos[0]["sm_zip_b64"]
                assert sync["payload"]["execution_status"] == "STOPPED"

            # a third client is rejected (max_clients=2)
            import websockets
            websocket_3 = await websockets.connect("ws://127.0.0.1:{0}".format(port))
            error = protocol.parse_message(await asyncio.wait_for(websocket_3.recv(), 5))
            assert error["type"] == protocol.ERROR
            assert error["payload"]["code"] == "too_many_clients"

            # client 1 starts the state machine; both clients must see the execution status changes
            # (STARTED, later FINISHED) and per-state execution status broadcasts, in any order
            await websocket_1.send(protocol.make_message(
                protocol.EXECUTION_COMMAND,
                {"command": "start", "state_machine_id": state_machine.state_machine_id}))

            async def collect_until_finished(websocket):
                execution_statuses = []
                state_status_payloads = []
                while "FINISHED" not in execution_statuses or not state_status_payloads:
                    message = protocol.parse_message(await websocket.recv())
                    if message["type"] == protocol.EXECUTION_STATUS_CHANGED:
                        execution_statuses.append(message["payload"]["status"])
                    elif message["type"] == protocol.STATE_EXECUTION_STATUS_CHANGED:
                        state_status_payloads.append(message["payload"])
                return execution_statuses, state_status_payloads

            for websocket in (websocket_1, websocket_2):
                execution_statuses, state_status_payloads = await asyncio.wait_for(
                    collect_until_finished(websocket), 15)
                assert "STARTED" in execution_statuses
                assert all(payload["state_machine_id"] == state_machine.state_machine_id and payload["state_path"]
                           for payload in state_status_payloads)

            await websocket_1.close()
            await websocket_2.close()

        asyncio.run(scenario())

        core_singletons.state_machine_execution_engine.join(3)
    finally:
        if server:
            server.stop()
        testing_utils.shutdown_environment_only_core(caplog=caplog)


if __name__ == '__main__':
    pytest.main([__file__, '-xvs'])
