# Copyright (C) 2014-2017 DLR
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
from pkg_resources import resource_string


# a dictionary to hold all loggers created so far
existing_loggers = {}

# Root namespace
rafcon_root = "rafcon"

try:
    logging_config = json.loads(resource_string(rafcon_root, "logging.conf"))
    logging.config.dictConfig(logging_config)
except ValueError as e:
    print "Could not load logging.conf (ValueError: {})".format(e)  # we can't use a logger here (chicken-egg-problem)

additional_logging_conf_path = os.environ.get("RAFCON_LOGGING_CONF", "")
if os.path.isfile(additional_logging_conf_path):
    with open(additional_logging_conf_path) as additional_logging_conf:
        try:
            logging.config.dictConfig(json.load(additional_logging_conf))
        except ValueError as e:
            print "Could not load {} (ValueError: {})".format(additional_logging_conf_path, e)


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

    def __init__(self, logger=None, gtk_quit=False):
        """Constructor receives decorator parameters

        The decorator is intended for all asynchronous method, whose exceptions cannot be caught at the calling
        instance, as the execution is deferred.

        :param logging.Logger logger: The logger to log to
        :param bool gtk_quit: Flag whether to stop the GTK main loop
        """
        self.logger = logger
        self.gtk_quit = gtk_quit

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
                if self.gtk_quit:
                    from gtk import main_quit
                    main_quit()

        return wrapper
