# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: linkage_overview
   :synopsis: A module that holds the controller giving footage to the linkage overview.

"""

from gtkmvc3.model_mt import ModelMT

from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.container_state import ContainerState
from rafcon.core.states.library_state import LibraryState

from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.controllers.state_editor.io_data_port_list import InputPortListController, OutputPortListController
from rafcon.gui.controllers.state_editor.scoped_variable_list import ScopedVariableListController
from rafcon.gui.controllers.state_editor.outcomes import StateOutcomesEditorController
from rafcon.gui.models import LibraryStateModel, ContainerStateModel


class LinkageOverviewController(ExtendedController, ModelMT):
    def __init__(self, model, view):
        ExtendedController.__init__(self, model, view)

        self.add_controller('input_data_ports', InputPortListController(model, view.inputs_view))
        self.add_controller('output_data_ports', OutputPortListController(model, view.outputs_view))
        self.add_controller('outcomes', StateOutcomesEditorController(model, view.outcomes_view))

        scoped_variable_state_m = model.state_copy if isinstance(model, LibraryStateModel) else model
        self.add_controller('scoped_variables', ScopedVariableListController(scoped_variable_state_m, view.scope_view))

        if not isinstance(scoped_variable_state_m, ContainerStateModel):
            view['scoped_box'].destroy()

        view.inputs_view.show()
        view.outputs_view.show()
        view.scope_view.show()
        view.outcomes_view.show()
