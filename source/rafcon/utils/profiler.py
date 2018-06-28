# Copyright (C) 2016-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import os
from rafcon.utils import log
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE
logger = log.get_logger("profiler")

_profilers = {}


def start(name):
    if not name:
        return
    try:
        import profiling.tracing
        _profilers[name] = profiling.tracing.TracingProfiler()
        _profilers[name].start()
        logger.debug("The profiler has been started")
    except ImportError:
        _profilers[name] = None
        logger.error("Cannot run profiler due to missing Python package 'profiling'")


def stop(name, result_path=RAFCON_TEMP_PATH_BASE, view=False):
    if name not in _profilers:
        return
    _profilers[name].stop()

    if view:
        _profilers[name].run_viewer()

    if os.path.isdir(result_path):
        import pickle
        import datetime
        date = datetime.datetime.now().strftime("%Y.%m.%d-%H:%M:%S.%f")
        result_file = os.path.join(result_path, "{}-{}.prf".format(name, date))
        result = _profilers[name].result()
        with open(result_file, 'wb') as f:
            pickle.dump((_profilers[name].__class__, result), f, pickle.HIGHEST_PROTOCOL)
        del _profilers[name]
        logger.info("The profiler result has been dumped. Run the following command for inspection:")
        logger.info("$ profiling view {}".format(result_file))
