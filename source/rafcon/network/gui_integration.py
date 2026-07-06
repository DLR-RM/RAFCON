"""
.. module:: gui_integration
   :synopsis: Wires a RemoteClient into a running RAFCON GUI process

Applies all incoming server events to the local mirror core via ``GLib.idle_add``, so the regular
model/controller notification chain runs on the GTK main loop, exactly as for local changes.
"""

from gi.repository import GLib

import rafcon.network
from rafcon.network import mirror, protocol
from rafcon.network.client import RemoteClient
from rafcon.network.remote_engine import patch_execution_engine

from rafcon.utils import log
logger = log.get_logger(__name__)


def connect_to_remote_core(url):
    """Attach this GUI process to a remote RAFCON core

    :param str url: websocket url of the core server, e.g. ``ws://localhost:9999``
    :return: the started :class:`RemoteClient`
    """
    import rafcon.core.singleton as core_singletons

    rafcon.network.remote_session = True
    logger.warning("Remote session: this GUI is attached to a remote core. State machine editing is "
                   "not synchronized to the core and should be avoided.")

    client = RemoteClient(url, _dispatch_event)
    patch_execution_engine(core_singletons.state_machine_execution_engine, client)
    client.start()
    return client


def _dispatch_event(message):
    # called from the network thread; marshal onto the GTK main loop
    GLib.idle_add(_apply_event, message)


def _apply_event(message):
    import rafcon.core.singleton as core_singletons
    manager = core_singletons.state_machine_manager
    engine = core_singletons.state_machine_execution_engine
    message_type = message["type"]
    payload = message["payload"]

    try:
        if message_type == protocol.SYNC:
            for sm_info in payload["state_machines"]:
                mirror.add_state_machine_to_manager(manager, sm_info["sm_zip_b64"], sm_info["state_machine_id"])
                for status in sm_info.get("state_statuses", []):
                    mirror.apply_state_execution_status(manager, sm_info["state_machine_id"],
                                                        status["state_path"], status["status"])
            mirror.apply_execution_status(engine, payload["execution_status"])
            active_sm_id = payload.get("active_state_machine_id")
            if active_sm_id is not None and engine.finished_or_stopped():
                manager.active_state_machine_id = active_sm_id
        elif message_type == protocol.EXECUTION_STATUS_CHANGED:
            mirror.apply_execution_status(engine, payload["status"])
        elif message_type == protocol.STATE_EXECUTION_STATUS_CHANGED:
            mirror.apply_state_execution_status(manager, payload["state_machine_id"],
                                                payload["state_path"], payload["status"])
        elif message_type == protocol.STATE_MACHINE_ADDED:
            mirror.add_state_machine_to_manager(manager, payload["sm_zip_b64"], payload["state_machine_id"])
        elif message_type == protocol.STATE_MACHINE_REMOVED:
            if payload["state_machine_id"] in manager.state_machines:
                manager.remove_state_machine(payload["state_machine_id"])
        elif message_type == protocol.ERROR:
            logger.error("Remote core reported an error: {0}".format(payload))
        else:
            logger.debug("Ignoring event of unknown type '{0}'".format(message_type))
    except Exception:
        logger.exception("Could not apply remote event '{0}'".format(message_type))
    return False  # do not repeat the idle callback
