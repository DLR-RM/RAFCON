# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: log
   :synopsis: A module caring about the logging capability of RAFCON

"""

import os
import logging
import logging.config
import json
import warnings
import importlib.resources as importlib_resources

# a dictionary to hold all loggers created so far
existing_loggers = {}

# Root namespace
rafcon_root = "rafcon"


class RAFCONDeprecationWarning(DeprecationWarning):
    pass


# Credits: https://stackoverflow.com/a/35804945
def add_logging_level(level_name, level_num, method_name=None):
    """Add new logging level

    Comprehensively adds a new logging level to the `logging` module and the currently configured logging class.

    `method_name` becomes a convenience method for both `logging` itself and the class returned by
    `logging.getLoggerClass()` (usually just `logging.Logger`). If `method_name` is not specified, `level_name.lower()`
    is used.

    :param str level_name: the level name
    :param int level_num: the level number/value
    :raises AttributeError: if the level
        name is already an attribute of the `logging` module or if the method name is already present

    Example
    -------
    >>> add_logging_level('TRACE', logging.DEBUG - 5)
    >>> logging.getLogger(__name__).setLevel("TRACE")
    >>> logging.getLogger(__name__).trace('that worked')
    >>> logging.trace('so did this')
    >>> logging.TRACE
    5

    """
    if not method_name:
        method_name = level_name.lower()

    if hasattr(logging, level_name):
        raise AttributeError('{} already defined in logging module'.format(level_name))
    if hasattr(logging, method_name):
        raise AttributeError('{} already defined in logging module'.format(method_name))
    if hasattr(logging.getLoggerClass(), method_name):
        raise AttributeError('{} already defined in logger class'.format(method_name))

    # This method was inspired by the answers to Stack Overflow post
    # http://stackoverflow.com/q/2183233/2988730, especially
    # http://stackoverflow.com/a/13638084/2988730
    def log_for_level(self, message, *args, **kwargs):
        if self.isEnabledFor(level_num):
            self._log(level_num, message, args, **kwargs)
    def log_to_root(message, *args, **kwargs):
        logging.log(level_num, message, *args, **kwargs)

    logging.addLevelName(level_num, level_name)
    setattr(logging, level_name, level_num)
    setattr(logging.getLoggerClass(), method_name, log_for_level)
    setattr(logging, method_name, log_to_root)


add_logging_level('VERBOSE', logging.DEBUG - 5)


# Load config from RAFCON_LOGGING_CONF if available, otherwise the default logging.conf
logging_conf_path = os.environ.get("RAFCON_LOGGING_CONF", str(importlib_resources.files(rafcon_root) / 'logging.conf'))
with open(logging_conf_path) as logging_conf_file:
    try:
        logging_config = json.load(logging_conf_file)
        logging.config.dictConfig(logging_config)
    except ValueError as e:
        # we can't use a logger here (chicken-egg-problem)
        print("Could not load {} (ValueError: {})".format(logging_conf_path, e))

warnings.simplefilter("once", RAFCONDeprecationWarning)
logging.captureWarnings(True)

# Level of the "rafcon" logger as set by the logging configuration. Other sources (the core config option LOGGING_LEVEL,
# the shown levels of the GUI logging console) can only raise it, see set_logging_level_limit.
_logging_conf_level = logging.getLogger(rafcon_root).level
_logging_level_limits = {}


def set_logging_level_limit(source, level):
    """Raise the level of the "rafcon" logger for one source, or remove the limit of that source with None

    Records below the resulting level are not created at all, so they are skipped for every handler (console, logging
    console of the GUI, log files). The resulting level is the highest of the logging configuration and all limits.

    :param str source: Who sets the limit, e.g. "core" or "gui"
    :param level: A level name like "INFO", a level number, or None
    :raises ValueError: if the level name is unknown
    """
    if isinstance(level, str):
        level_number = logging.getLevelName(level.upper())
        if not isinstance(level_number, int):
            raise ValueError("Unknown logging level: {}".format(level))
        level = level_number
    if level is None:
        _logging_level_limits.pop(source, None)
    else:
        _logging_level_limits[source] = level
    logging.getLogger(rafcon_root).setLevel(max([_logging_conf_level] + list(_logging_level_limits.values())))


def get_logger(name):
    """Returns a logger for the given name

    The function is basically a wrapper for logging.getLogger and only ensures that the namespace is within "rafcon."
    and that the propagation is enabled.

    :param str name: The namespace of the new logger
    :return: Logger object with given namespace
    :rtype: logging.Logger
    """
    if name in existing_loggers:
        return existing_loggers[name]

    # Ensure that all logger are within the RAFCON root namespace
    namespace = name if name.startswith(rafcon_root + ".") else rafcon_root + "." + name
    logger = logging.getLogger(namespace)
    logger.propagate = True
    existing_loggers[name] = logger

    return logger


# noinspection PyPep8Naming
# as this class represents a function (decorator)
class log_exceptions(object):
    """Decorator to catch all exceptions and log them"""

    def __init__(self, logger=None):
        """Constructor receives decorator parameters

        The decorator is intended for all asynchronous method, whose exceptions cannot be caught at the calling
        instance, as the execution is deferred.

        :param logging.Logger logger: The logger to log to
        """
        self.logger = logger

    def __call__(self, function):
        def wrapper(*args, **kwargs):
            """Catch all exceptions and log them as error message"""
            try:
                return function(*args, **kwargs)
            except:
                # Only create custom logger if no logger was passed and an exception occurred
                if not self.logger:
                    self.logger = get_logger(__name__)
                self.logger.exception("Unexpected error")

        return wrapper
