import logging

from rafcon.core.config import global_config
from rafcon.utils import log

from tests import utils as testing_utils


class CountingHandler(logging.Handler):
    def __init__(self):
        super(CountingHandler, self).__init__(level=1)
        self.levels = []

    def emit(self, record):
        self.levels.append(record.levelno)


def log_all_levels(handler):
    handler.levels.clear()
    logger = log.get_logger("test_logging_level")
    logger.verbose("v")
    logger.debug("d")
    logger.info("i")
    logger.warning("w")
    return set(handler.levels)


def test_logging_level_option(caplog):
    testing_utils.initialize_environment_core()
    handler = CountingHandler()
    rafcon_logger = logging.getLogger("rafcon")
    rafcon_logger.addHandler(handler)
    try:
        # default None keeps the level of logging.conf (VERBOSE)
        assert log_all_levels(handler) == {logging.VERBOSE, logging.DEBUG, logging.INFO, logging.WARNING}

        global_config.set_config_value("LOGGING_LEVEL", "INFO")
        assert log_all_levels(handler) == {logging.INFO, logging.WARNING}

        # a second source can only raise the level, removing it restores the core level
        log.set_logging_level_limit("gui", "WARNING")
        assert log_all_levels(handler) == {logging.WARNING}
        log.set_logging_level_limit("gui", "VERBOSE")
        assert log_all_levels(handler) == {logging.INFO, logging.WARNING}
        log.set_logging_level_limit("gui", None)

        # an invalid level is ignored with an error
        global_config.set_config_value("LOGGING_LEVEL", "no_level")
        assert log_all_levels(handler) == {logging.VERBOSE, logging.DEBUG, logging.INFO, logging.WARNING}

        global_config.set_config_value("LOGGING_LEVEL", None)
        assert log_all_levels(handler) == {logging.VERBOSE, logging.DEBUG, logging.INFO, logging.WARNING}
    finally:
        rafcon_logger.removeHandler(handler)
        log.set_logging_level_limit("gui", None)
        global_config.set_config_value("LOGGING_LEVEL", None)
        testing_utils.shutdown_environment_only_core(caplog=caplog, expected_warnings=6, expected_errors=1)  # 6 test warnings, invalid level
