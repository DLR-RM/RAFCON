"""
.. module:: remote_engine
   :synopsis: Redirects the local (mirror) execution engine's command methods to a remote core

The command methods are re-bound on the singleton *instance*, so all existing GUI code
(menu bar, tool bar, right-click menus) keeps calling ``execution_engine.start()`` etc. unchanged
while the commands are actually forwarded over the websocket connection. Status updates flow back
via :func:`rafcon.network.mirror.apply_execution_status`, which uses the untouched
``set_execution_mode`` and thereby triggers the regular GUI notifications.
"""

from rafcon.utils import log
logger = log.get_logger(__name__)


def patch_execution_engine(execution_engine, remote_client):
    """Replace all execution command methods of the given engine with remote-forwarding versions

    :param execution_engine: the local mirror :class:`ExecutionEngine` singleton
    :param rafcon.network.client.RemoteClient remote_client: the connection to forward commands to
    """

    def forward(command, **kwargs):
        logger.debug("Forwarding execution command '{0}' to remote core".format(command))
        remote_client.send_execution_command(command, **kwargs)

    execution_engine.start = lambda state_machine_id=None, start_state_path=None: \
        forward("start", state_machine_id=state_machine_id, state_path=start_state_path)
    execution_engine.step_mode = lambda state_machine_id=None: \
        forward("step_mode", state_machine_id=state_machine_id)
    execution_engine.run_to_selected_state = lambda path, state_machine_id=None: \
        forward("run_to_selected_state", state_machine_id=state_machine_id, state_path=path)
    execution_engine.run_selected_state = lambda start_state_path=None, state_machine_id=None: \
        forward("run_selected_state", state_machine_id=state_machine_id, state_path=start_state_path)
    execution_engine.run_only_selected_state = lambda start_state_path=None, state_machine_id=None: \
        forward("run_only_selected_state", state_machine_id=state_machine_id, state_path=start_state_path)
    for command in ("pause", "stop", "step_into", "step_over", "step_out", "backward_step"):
        setattr(execution_engine, command, lambda _command=command: forward(_command))
