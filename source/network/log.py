import logging
import sys


# a dictionary to hold all loggers created so far
existing_loggers = {}


def get_logger(name):
    """Generates and returns a logger object

    Returns a logger for a specific name i.e. a class name. There are several logging modes available:
    info, debug, warn, error

    :param name: the name of the new logger
    """

    if name in existing_loggers:
        return existing_loggers[name]

    full_formatter = logging.Formatter("%(asctime)s: %(levelname)8s - %(name)s:  %(message)s", "%H:%M:%S")

    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setFormatter(full_formatter)
    stdout_handler.setLevel(logging.DEBUG)

    stderr_handler = logging.StreamHandler(sys.stderr)
    stderr_handler.setFormatter(full_formatter)
    stderr_handler.setLevel(logging.ERROR)

    logger = logging.getLogger(name)
    logger.addHandler(stdout_handler)
    logger.addHandler(stderr_handler)

    logger.setLevel(logging.DEBUG)

    existing_loggers[name] = logger

    return logger
