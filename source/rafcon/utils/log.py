import logging
import sys


class NoHigherLevelFilter(logging.Filter):
    """
    A logging filter that filters out all logging records, whose level are smaller than the level specified in the
    constructor.

    :ivar level: the highest level the filter will pass
    """
    def __init__(self, level):
        logging.Filter.__init__(self)
        self.level = level

    def filter(self, record):
        """
        Filters all records, whose logging level is smaller than the level specified in the constructor
        :param record:
        :return:
        """
        return record.levelno < self.level


class LoggingViewHandler(logging.Handler):

    _logging_views = {}

    def __init__(self):
        super(LoggingViewHandler, self).__init__()

        try:
            unicode
            self._unicode = True
        except NameError:
            self._unicode = False

    @classmethod
    def set_logging_view(cls, name, text_view):
        cls._logging_views[name] = text_view

    @classmethod
    def remove_logging_view(cls, name):
        if name in cls._logging_views:
            del cls._logging_views[name]

    def emit(self, record):
        """Logs a new record

        If a logging view is given, it is used to log the new record to. The code is partially copied from the
        StreamHandler class
        :param record:
        :return:
        """
        try:
            # Shorten the source name of the record (remove rafcon.)
            if sys.version_info >= (2, 7):
                record.__setattr__("name", record.name.replace("rafcon.", ""))
            msg = self.format(record)
            fs = "%s"
            if not self._unicode:  # if no unicode support...
                entry = fs % msg
            else:
                try:
                    if isinstance(msg, unicode):
                        ufs = u'%s'
                        try:
                            entry = ufs % msg
                        except UnicodeEncodeError:
                            entry = fs % msg
                    else:
                            entry = fs % msg
                except UnicodeError:
                    entry = fs % msg

            for logging_view in self._logging_views.itervalues():
                logging_view.print_message(entry, record.levelno)
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            self.handleError(record)

# a dictionary to hold all loggers created so far
existing_loggers = {}

# global handler for logging on console and text views
logging_view_handler = None


def register_logging_view(name, logging_view):
    LoggingViewHandler.set_logging_view(name, logging_view)


def unregister_logging_view(name):
    LoggingViewHandler.remove_logging_view(name)


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
    no_higher_level_filter = NoHigherLevelFilter(logging.ERROR)
    stdout_handler.addFilter(no_higher_level_filter)

    stderr_handler = logging.StreamHandler(sys.stderr)
    stderr_handler.setFormatter(full_formatter)
    stderr_handler.setLevel(logging.ERROR)

    logging_view_handler = LoggingViewHandler()
    logging_view_handler.setFormatter(full_formatter)
    logging_view_handler.setLevel(logging.DEBUG)

    logger = logging.getLogger(name)
    logger.addHandler(stdout_handler)
    logger.addHandler(stderr_handler)
    logger.addHandler(logging_view_handler)

    logger.setLevel(logging.DEBUG)

    existing_loggers[name] = logger

    return logger