# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: singleton
   :synopsis: A module to hold all singletons of the GTK GUI

"""
import threading

from rafcon.core.config import global_config
from rafcon.gui.runtime_config import global_runtime_config
from rafcon.core.singleton import state_machine_manager,\
    global_variable_manager, state_machine_execution_engine, library_manager
from rafcon.gui.config import global_gui_config
from rafcon.gui.models.config_model import ConfigModel
from rafcon.gui.models.global_variable_manager import GlobalVariableManagerModel
from rafcon.gui.models.library_manager import LibraryManagerModel
from rafcon.gui.models.state_machine_execution_engine import StateMachineExecutionEngineModel
from rafcon.gui.models.state_machine_manager import StateMachineManagerModel

global_focus = None

# thread id of the thread which created the gui singletons -> supposed to be used to hold all mvc objects in one thread
thread_identifier = threading.currentThread().ident

# This variable holds the global state machine manager model as long as only one StateMachineMangerModel is allowed
state_machine_manager_model = StateMachineManagerModel(state_machine_manager)

library_manager_model = LibraryManagerModel(library_manager)

state_machine_execution_model = StateMachineExecutionEngineModel(state_machine_execution_engine)

global_variable_manager_model = GlobalVariableManagerModel(global_variable_manager)

main_window_controller = None

core_config_model = ConfigModel(global_config)
gui_config_model = ConfigModel(global_gui_config)
runtime_config_model = ConfigModel(global_runtime_config)
