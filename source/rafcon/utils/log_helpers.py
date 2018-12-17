# Copyright (C) 2017-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>

import logging
import sys


class NoHigherLevelFilter(logging.Filter):
    """Filter high log levels
    
    A logging filter that filters out all logging records, whose level are smaller than the level specified in the
    constructor.

    :ivar level: the highest level the filter will pass
    """
    def __init__(self, level):
        logging.Filter.__init__(self)
        self.level = level

    def filter(self, record):
        """Filter high log levels
        
        Filters all records, whose logging level is smaller than the level specified in the constructor
        
        :param record:
        :return:
        """
        return record.levelno < self.level


class LoggingViewHandler(logging.Handler):
    """A LoggingHandler for Gtk.TextViews
    
    The `LoggingViewHandler` prints log messages in special `Gtk.TextView`s that provide a `print_message` method. 
    The views must register themselves to the handler. There can be multiple views registered for one handler.
    """

    _logging_views = {}

    def __init__(self):
        super(LoggingViewHandler, self).__init__()

    @classmethod
    def add_logging_view(cls, name, text_view):
        cls._logging_views[name] = text_view

    @classmethod
    def remove_logging_view(cls, name):
        if name in cls._logging_views:
            del cls._logging_views[name]

    def emit(self, record):
        """Logs a new record

        If a logging view is given, it is used to log the new record to. The code is partially copied from the
        StreamHandler class.
        
        :param record:
        :return:
        """
        try:
            # Shorten the source name of the record (remove rafcon.)
            if sys.version_info >= (2, 7):
                record.__setattr__("name", record.name.replace("rafcon.", ""))
            msg = self.format(record)
            fs = "%s"
            try:
                ufs = u'%s'
                try:
                    entry = ufs % msg
                except UnicodeEncodeError:
                    entry = fs % msg
            except UnicodeError:
                entry = fs % msg

            for logging_view in self._logging_views.values():
                logging_view.print_message(entry, record.levelno)
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            self.handleError(record)
