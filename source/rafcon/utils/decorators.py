# Copyright (C) 2016-2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: decorators
   :synopsis: A module to collect all helpful decorators

"""

from rafcon.utils import log
logger = log.get_logger(__name__)


def avoid_parallel_execution(func):
    """ A decorator to avoid the parallel execution of a function. If the function is currently called, the second call
    is just skipped.

    :param func: The function to decorate
    :return:
    """
    def func_wrapper(*args, **kwargs):
        if not hasattr(func, "currently_executing"):
            func.currently_executing = False
        if not func.currently_executing:
            func.currently_executing = True
            return_value = func(*args, **kwargs)
            func.currently_executing = False
            return return_value
        else:
            logger.verbose("Avoid parallel execution of function {}".format(func))
            return
    return func_wrapper
