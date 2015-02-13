import logging
import sys

# class CustomStreamHandler(logging.StreamHandler):
#
#     def __init__(self):
#         logging.StreamHandler.__init__(self)
#
#     def emit(self, record):
#         try:
#             msg = self.format(record)
#             self.stream.write(msg)
#             self.stream.write("\n")
#             self.flush()
#             print "Some dummy print"
#         except (KeyboardInterrupt, SystemExit):
#             raise
#         except:
#             self.handleError(record)

class NoHigherLevelFilter(logging.Filter):
    def __init__(self, level):
        logging.Filter.__init__(self)
        self.level = level

    def filter(self, record):
        return record.levelno < self.level


class DebugTextViewFilter(logging.Filter):
    def __init__(self):
        logging.Filter.__init__(self)
        self.logging_text_view = None

    def set_logging_test_view(self, logging_test_view):
        self.logging_text_view = logging_test_view

    def filter(self, record):
        if not self.logging_text_view is None:
            #print "Redirecting debug output to the logging_text_view"
            self.logging_text_view.print_debug(format_log_record_for_view(record))
        # Set this to false if the output should not be printed on stdout
        return True


class ErrorTextViewFilter(logging.Filter):
    def __init__(self):
        self.logging_text_view = None

    def set_logging_test_view(self, logging_test_view):
        self.logging_text_view = logging_test_view

    def filter(self, record):
        if not self.logging_text_view is None:
            #print "Redirecting error output to the logging_text_view"
            self.logging_text_view.print_error(format_log_record_for_view(record))
        # Set this to false if the output should not be printed on stdout
        return True


def format_log_record_for_view(record):
    formatter = logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s")
    return formatter.format(record)



debug_filter = DebugTextViewFilter()
error_filter = ErrorTextViewFilter()


def get_logger(name):
    full = logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s")

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

    return logger