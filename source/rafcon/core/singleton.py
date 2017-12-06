# Copyright (C) 2014-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: singleton
   :synopsis: A module to hold all singletons of the state machine

"""

import argparse
import threading

from rafcon.core.global_variable_manager import GlobalVariableManager
from rafcon.core.library_manager import LibraryManager
from rafcon.core.execution.execution_engine import ExecutionEngine
from rafcon.core.state_machine_manager import StateMachineManager

# thread id of the thread which created the core singletons
thread_identifier = threading.currentThread().ident

# This variable holds the global variable manager singleton
global_variable_manager = GlobalVariableManager()

# This variable holds the library manager singleton
library_manager = LibraryManager()

# This variable holds the global state machine manager object
state_machine_manager = StateMachineManager()

# This variable holds the execution engine singleton
state_machine_execution_engine = ExecutionEngine(state_machine_manager)

# signal that cause shut down
shut_down_signal = None

argument_parser = argparse.ArgumentParser(description='Start RAFCON', fromfile_prefix_chars='@')
