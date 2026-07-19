import asyncio
import json
import os
import socket
import urllib.request

import pytest

from tests import utils as testing_utils

from rafcon.network import protocol


def get_free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


async def connect_and_handshake(port, client_name="test-client", flavor=None):
    import websockets
    websocket = await websockets.connect("ws://127.0.0.1:{0}".format(port), max_size=None)
    hello = {"client_name": client_name, "protocol_version": protocol.PROTOCOL_VERSION}
    if flavor:
        hello["flavor"] = flavor
    await websocket.send(protocol.make_message(protocol.HELLO, hello))
    welcome = protocol.parse_message(await asyncio.wait_for(websocket.recv(), 5))
    assert welcome["type"] == protocol.WELCOME
    sync = protocol.parse_message(await asyncio.wait_for(websocket.recv(), 5))
    assert sync["type"] == protocol.SYNC
    return websocket, sync


def test_web_flavor_sync_and_streams(caplog):
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
        server = RemoteServer(host="127.0.0.1", port=port, max_clients=3)
        server.start()

        async def scenario():
            web_ws, web_sync = await connect_and_handshake(port, "web-client", flavor="web")
            gtk_ws, gtk_sync = await connect_and_handshake(port, "gtk-client")

            # web sync: nested JSON instead of the zip, plus globals and log backlog
            web_sm = web_sync["payload"]["state_machines"][0]
            assert "sm_json" in web_sm and "sm_zip_b64" not in web_sm
            assert web_sm["sm_json"]["root_state"]["path"] == state_machine.root_state.get_path()
            assert "global_variables" in web_sync["payload"]
            assert "log_backlog" in web_sync["payload"]

            # gtk sync unchanged: zip, no web-only keys
            gtk_sm = gtk_sync["payload"]["state_machines"][0]
            assert "sm_zip_b64" in gtk_sm and "sm_json" not in gtk_sm
            assert "global_variables" not in gtk_sync["payload"]

            # global variable changes reach the web client only
            core_singletons.global_variable_manager.set_variable("web_test_var", 42)
            variables_payload = await asyncio.wait_for(
                _wait_for(web_ws, protocol.GLOBAL_VARIABLES_CHANGED), 10)
            assert any(variable["key"] == "web_test_var"
                       for variable in variables_payload["variables"])

            # run the state machine: web client gets history and log streams and FINISHED
            await web_ws.send(protocol.make_message(
                protocol.EXECUTION_COMMAND,
                {"command": "start", "state_machine_id": state_machine.state_machine_id}))

            web_seen = await asyncio.wait_for(_collect_types(
                web_ws, until_status="FINISHED"), 20)
            assert protocol.EXECUTION_HISTORY_EVENT in web_seen
            assert protocol.LOG_RECORD in web_seen

            # gtk client must never receive web-only message types
            gtk_seen = await asyncio.wait_for(_collect_types(
                gtk_ws, until_status="FINISHED"), 20)
            assert protocol.EXECUTION_HISTORY_EVENT not in gtk_seen
            assert protocol.LOG_RECORD not in gtk_seen
            assert protocol.GLOBAL_VARIABLES_CHANGED not in gtk_seen

            await web_ws.close()
            await gtk_ws.close()

        async def _wait_for(websocket, message_type):
            while True:
                message = protocol.parse_message(await websocket.recv())
                if message["type"] == message_type:
                    return message["payload"]

        async def _collect_types(websocket, until_status):
            seen = set()
            finished = False
            while not finished:
                message = protocol.parse_message(await websocket.recv())
                seen.add(message["type"])
                if message["type"] == protocol.EXECUTION_STATUS_CHANGED and \
                        message["payload"]["status"] == until_status:
                    finished = True
            # batched streams (history, logs) may flush shortly after FINISHED — drain briefly
            try:
                while True:
                    message = protocol.parse_message(await asyncio.wait_for(websocket.recv(), 1.5))
                    seen.add(message["type"])
            except asyncio.TimeoutError:
                pass
            return seen

        asyncio.run(scenario())

        core_singletons.state_machine_execution_engine.join(3)
        # reset the engine status for subsequent tests sharing the singleton
        core_singletons.state_machine_execution_engine.stop()
        core_singletons.state_machine_execution_engine.join(3)
    finally:
        if server:
            server.stop()
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_web_libraries_and_open_state_machine(caplog):
    testing_utils.initialize_environment_core()
    server = None
    try:
        import rafcon.core.singleton as core_singletons
        from rafcon.network.server import RemoteServer

        sm_path = testing_utils.get_test_sm_path(
            os.path.join("unit_test_state_machines", "test_custom_entry_point"))

        port = get_free_port()
        server = RemoteServer(host="127.0.0.1", port=port, max_clients=3)
        server.start()
        # the server must never block on interactive library prompts
        assert core_singletons.library_manager.show_dialog is False

        async def scenario():
            web_ws, web_sync = await connect_and_handshake(port, "web-client", flavor="web")
            gtk_ws, gtk_sync = await connect_and_handshake(port, "gtk-client")

            # web sync carries the library tree, gtk sync does not
            assert isinstance(web_sync["payload"]["libraries"], dict)
            assert "libraries" not in gtk_sync["payload"]

            # open a state machine by path -> broadcast with the flavor-matching representation
            await web_ws.send(protocol.make_message(protocol.OPEN_STATE_MACHINE, {"path": sm_path}))
            added_payload = await asyncio.wait_for(_wait_for(web_ws, protocol.STATE_MACHINE_ADDED), 10)
            assert "sm_json" in added_payload and "sm_zip_b64" not in added_payload
            gtk_added = await asyncio.wait_for(_wait_for(gtk_ws, protocol.STATE_MACHINE_ADDED), 10)
            assert "sm_zip_b64" in gtk_added and "sm_json" not in gtk_added

            # a bogus path must produce an ERROR reply instead of hanging on input()
            await web_ws.send(protocol.make_message(
                protocol.OPEN_STATE_MACHINE, {"path": "/does/not/exist"}))
            error_payload = await asyncio.wait_for(_wait_for(web_ws, protocol.ERROR), 10)
            assert error_payload["code"] == "command_failed"

            # close the just-opened state machine -> broadcast to all clients
            opened_sm_id = added_payload["state_machine_id"]
            await web_ws.send(protocol.make_message(
                protocol.CLOSE_STATE_MACHINE, {"state_machine_id": opened_sm_id}))
            removed = await asyncio.wait_for(_wait_for(web_ws, protocol.STATE_MACHINE_REMOVED), 10)
            assert removed["state_machine_id"] == opened_sm_id
            removed_gtk = await asyncio.wait_for(_wait_for(gtk_ws, protocol.STATE_MACHINE_REMOVED), 10)
            assert removed_gtk["state_machine_id"] == opened_sm_id

            # closing an unknown state machine -> ERROR reply
            await web_ws.send(protocol.make_message(
                protocol.CLOSE_STATE_MACHINE, {"state_machine_id": 424242}))
            error_payload = await asyncio.wait_for(_wait_for(web_ws, protocol.ERROR), 10)
            assert error_payload["code"] == "command_failed"

            await web_ws.close()
            await gtk_ws.close()

        async def _wait_for(websocket, message_type):
            while True:
                message = protocol.parse_message(await websocket.recv())
                if message["type"] == message_type:
                    return message["payload"]

        asyncio.run(scenario())
    finally:
        if server:
            server.stop()
        # the bogus open_state_machine path and unknown close id each provoke one logged server error
        testing_utils.shutdown_environment_only_core(caplog=caplog, expected_errors=2)


def test_web_static_server(tmp_path):
    from rafcon.network.web_server import WebServer

    dist = tmp_path / "dist"
    dist.mkdir()
    (dist / "index.html").write_text("<html>rafcon-web</html>")
    (dist / "app.js").write_text("// asset")

    port = get_free_port()
    server = WebServer(port=port, ws_port=12345, dist_path=str(dist))
    server.start()
    try:
        def get(path):
            with urllib.request.urlopen("http://127.0.0.1:{0}{1}".format(port, path), timeout=5) as response:
                return response.status, response.read().decode("utf-8")

        status, body = get("/config.json")
        assert status == 200
        config = json.loads(body)
        assert config["websocket_port"] == 12345

        status, body = get("/")
        assert status == 200 and "rafcon-web" in body

        status, body = get("/app.js")
        assert status == 200 and "asset" in body

        # SPA fallback: unknown extension-less path serves index.html
        status, body = get("/some/deep/route")
        assert status == 200 and "rafcon-web" in body
    finally:
        server.stop()


if __name__ == '__main__':
    pytest.main([__file__, '-xvs'])
