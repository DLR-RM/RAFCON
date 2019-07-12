from __future__ import absolute_import
import os

from rafcon.utils import log
logger = log.get_logger(__name__)


def pre_init():
    """
    The pre_init function of the plugin. Here rafcon-classes can be extended/monkey-patched or completely substituted.
    A example is given with the rafcon_execution_hooks_plugin.
    :return:
    """

    logger.info("Run pre-initiation hook of {} plugin.".format(__file__.split(os.path.sep)[-2]))

    # Example: Monkey-Path rafcon.core.script.Script class to print additional log-message while execution
    from rafcon.core.script import Script
    old_execute_method = Script.execute

    def new_execute_method(self, state, inputs=None, outputs=None, backward_execution=False):
        logger.debug("patched version of Script class is used.")
        result = old_execute_method(self, state, inputs, outputs, backward_execution)
        logger.debug("patched version of Script execute-method is finished with result: {}.".format(result))
        return result

    Script.execute = new_execute_method


def post_init(setup_config):
    """
    The post_init function of the plugin. Here observer can be registered to the observables and other pre-init
    functionality of the plugin should be triggered. A simple example is given with the rafcon_execution_hooks_plugin.
    A complex example is given with the rafcon_monitoring_plugin.
    :param setup_config:
    :return:
    """
    logger.info("Run post-initiation hook of {} plugin.".format(__file__.split(os.path.sep)[-2]))

    from . import core_template_observer
    # Example 1: initiate observer some elements of the execution engine
    core_template_observer.ExecutionEngineObserver()

    # Example 2: initiate observer execution status
    core_template_observer.ExecutionStatusObserver()

    from . import gtkmvc_template_observer
    # Example 3: gtkmvc3 general modification observer
    # initiate observer of root_state model-object which already implements a power full recursive notification pattern
    gtkmvc_template_observer.RootStateModificationObserver()

    # Example 4: gtkmvc3 meta signal observer
    gtkmvc_template_observer.MetaSignalModificationObserver()
