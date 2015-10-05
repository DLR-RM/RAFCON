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


class DebugTextViewFilter(logging.Filter):
    """ A class to filter the logging records and display them on to a gtk text view. This class cares for the debug
    level - logging records.

        :ivar logging_text_view: the text view to print the logging records to
    """
    def __init__(self):
        logging.Filter.__init__(self)
        self.logging_text_view = None

    def set_logging_test_view(self, logging_test_view):
        """
        Sets the logging view for this filter.

        :param logging_test_view: The logging test view to redirect all logging records to
        """
        self.logging_text_view = logging_test_view

    def filter(self, record):
        """
        Redirects a debug logging record to the test view
        :param record: the record to be redirected
        :return:
        """
        if not self.logging_text_view is None:
            #print "Redirecting debug output to the logging_text_view"
            if record.levelno == 10:
                self.logging_text_view.print_debug(format_log_record_for_view(record))
            elif record.levelno == 20:
                self.logging_text_view.print_info(format_log_record_for_view(record))
            elif record.levelno == 30:
                self.logging_text_view.print_warning(format_log_record_for_view(record))
        # Set this to false if the output should not be printed on stdout
        return True


class ErrorTextViewFilter(logging.Filter):
    """ A class to filter the logging records and display them on to a gtk text view. This class cares for the error
    level - logging records.

        :ivar logging_text_view: the text view to print the logging records to
    """
    def __init__(self):
        self.logging_text_view = None

    def set_logging_test_view(self, logging_test_view):
        """
        Sets the logging view for this filter.

        :param logging_test_view: The logging test view to redirect all logging records to
        """
        self.logging_text_view = logging_test_view

    def filter(self, record):
        """
        Redirects a error logging record to the test view
        :param record: the record to be redirected
        :return:
        """
        if not self.logging_text_view is None:
            #print "Redirecting error output to the logging_text_view"
            self.logging_text_view.print_error(format_log_record_for_view(record))
        # Set this to false if the output should not be printed on stdout
        return True


def format_log_record_for_view(record):
    """
    Format the logging record to start with the time, the logging mode (debug, warn etc.), the name of the logger and
    the finally the message.
    :param record: the record to get formatted
    :return:
    """
    # shortens the "name" attribute of the record by "rafcon." to save space
    if not sys.version_info < (2, 7):
        record.__setattr__("name", record.name.replace("rafcon.", ""))
    formatter = logging.Formatter("%(asctime)s: %(levelname)s - %(name)s:  %(message)s", "%H:%M:%S")
    # full_format logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s", "%H:%M:%S")
    return formatter.format(record)


debug_filter = DebugTextViewFilter()
error_filter = ErrorTextViewFilter()


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
    # stdout_handler.addFilter(debug_filter)

    stderr_handler = logging.StreamHandler(sys.stderr)
    stderr_handler.setFormatter(full_formatter)
    stderr_handler.setLevel(logging.ERROR)
    # stderr_handler.addFilter(error_filter)

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