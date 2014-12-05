import logging
import sys


def get_logger(name):
    class NoHigherLevelFilter(logging.Filter):
        def __init__(self, level):
            self.level = level

        def filter(self, record):
            return record.levelno < self.level


    full = logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s")

    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(full)
    stdout.setLevel(logging.DEBUG)
    error_filter = NoHigherLevelFilter(logging.ERROR)
    stdout.addFilter(error_filter)

    stderr = logging.StreamHandler(sys.stderr)
    stderr.setFormatter(full)
    stderr.setLevel(logging.ERROR)

    logger = logging.getLogger(name)
    logger.addHandler(stdout)
    logger.addHandler(stderr)

    logger.setLevel(logging.DEBUG)

    return logger
