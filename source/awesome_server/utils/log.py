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
    # shortens the "name" attribute of the record by "awesome_tool." to save space
    record.__setattr__("name", record.name.replace("awesome_tool.", ""))
    formatter = logging.Formatter("%(asctime)s: %(levelname)s - %(name)s:  %(message)s", "%H:%M:%S")
    return formatter.format(record)


debug_filter = DebugTextViewFilter()
error_filter = ErrorTextViewFilter()


# a dictionary to hold all loggers created so far
existing_loggers = {}

def get_logger(name):
    """
    Returns a logger for a specific name i.e. a class name. There are several logging modes available:
    info, debug, warn, error
    :param name: the name of the new logger
    :return:
    """

    if name in existing_loggers.iterkeys():
        return existing_loggers[name]

    full = logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s", "%H:%M:%S")

    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(full)
    stdout.setLevel(logging.DEBUG)
    no_higher_level_filter = NoHigherLevelFilter(logging.ERROR)
    stdout.addFilter(no_higher_level_filter)
    stdout.addFilter(debug_filter)

    stderr = logging.StreamHandler(sys.stderr)
    stderr.setFormatter(full)
    stderr.setLevel(logging.ERROR)
    stderr.addFilter(error_filter)

    logger = logging.getLogger(name)
    logger.addHandler(stdout)
    logger.addHandler(stderr)

    logger.setLevel(logging.DEBUG)

    existing_loggers[name] = logger

    return logger