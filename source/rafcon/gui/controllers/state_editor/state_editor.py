# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Michael Vilzmann <michael.vilzmann@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: state_editor
   :synopsis: A module that holds the state editor controller which provides footage all sub-state-element-controllers.

"""

from rafcon.core.states.library_state import LibraryState

from rafcon.gui.controllers.state_editor.data_flows import StateDataFlowsEditorController
from rafcon.gui.controllers.state_editor.description_editor import DescriptionEditorController
from rafcon.gui.controllers.state_editor.io_data_port_list import InputPortListController, OutputPortListController
from rafcon.gui.controllers.state_editor.linkage_overview import LinkageOverviewController
from rafcon.gui.controllers.state_editor.outcomes import StateOutcomesEditorController
from rafcon.gui.controllers.state_editor.overview import StateOverviewController
from rafcon.gui.controllers.state_editor.scoped_variable_list import ScopedVariableListController
from rafcon.gui.controllers.state_editor.source_editor import SourceEditorController
from rafcon.gui.controllers.state_editor.transitions import StateTransitionsEditorController
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.controllers.state_editor.semantic_data_editor import SemanticDataEditorController
from rafcon.gui.models import ContainerStateModel, AbstractStateModel, LibraryStateModel
from rafcon.gui.views.state_editor.state_editor import StateEditorView
from rafcon.gui.config import global_gui_config

from rafcon.utils import log
from rafcon.utils import plugins
logger = log.get_logger(__name__)


class StateEditorController(ExtendedController):
    """Controller handles the organization of the Logic-Data oriented State-Editor.
    Widgets concerning logic flow (outcomes and transitions) are grouped in the Logic Linkage expander.
    Widgets concerning data flow (data-ports and data-flows) are grouped in the data linkage expander.

    :param rafcon.gui.models.state.StateModel model: The state model
    """

    def __init__(self, model, view):
        """Constructor"""
        assert isinstance(model, AbstractStateModel)
        assert isinstance(view, StateEditorView)
        ExtendedController.__init__(self, model, view)

        if isinstance(model, LibraryStateModel) and not model.state_copy_initialized:
            model.enforce_generation_of_state_copy_model()
            logger.info("Respective state editor's state is most likely not drawn in the graphical editor. -> {0}"
                        "".format(model))
        sv_and_source_script_state_m = model.state_copy if isinstance(model, LibraryStateModel) else model

        self.add_controller('properties_ctrl', StateOverviewController(model, view.properties_view))

        self.inputs_ctrl = InputPortListController(model, view.inputs_view)
        self.add_controller('input_data_ports', self.inputs_ctrl)
        self.outputs_ctrl = OutputPortListController(model, view.outputs_view)
        self.add_controller('output_data_ports', self.outputs_ctrl)
        self.scopes_ctrl = ScopedVariableListController(sv_and_source_script_state_m, view.scopes_view)
        self.add_controller('scoped_variables', self.scopes_ctrl)
        self.add_controller('outcomes', StateOutcomesEditorController(model, view.outcomes_view))

        self.add_controller('transitions_ctrl', StateTransitionsEditorController(model, view.transitions_view))
        self.add_controller('data_flows_ctrl', StateDataFlowsEditorController(model, view.data_flows_view))

        self.add_controller('linkage_overview_ctrl', LinkageOverviewController(model, view.linkage_overview))

        self.add_controller('description_ctrl', DescriptionEditorController(model, view.description_view))
        if not isinstance(model, ContainerStateModel) and not isinstance(model, LibraryStateModel) or \
                isinstance(model, LibraryStateModel) and not isinstance(model.state_copy, ContainerStateModel):
            self.add_controller('source_ctrl', SourceEditorController(sv_and_source_script_state_m, view.source_view))
        else:
            view.source_view.get_top_widget().destroy()
        self.add_controller('semantic_data_ctrl', SemanticDataEditorController(model, view.semantic_data_view))

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        super(StateEditorController, self).register_view(view)
        view.prepare_the_labels()  # the preparation of the labels is done here to take into account plugin hook changes
        view['add_input_port_button'].connect('clicked', self.inputs_ctrl.on_add)
        view['add_output_port_button'].connect('clicked', self.outputs_ctrl.on_add)
        if isinstance(self.model, ContainerStateModel):
            view['add_scoped_variable_button'].connect('clicked', self.scopes_ctrl.on_add)

        view['remove_input_port_button'].connect('clicked', self.inputs_ctrl.on_remove)
        view['remove_output_port_button'].connect('clicked', self.outputs_ctrl.on_remove)
        if isinstance(self.model, ContainerStateModel):
            view['remove_scoped_variable_button'].connect('clicked', self.scopes_ctrl.on_remove)

        if isinstance(self.model, LibraryStateModel) or self.model.state.get_next_upper_library_root_state():
            view['add_input_port_button'].set_sensitive(False)
            view['remove_input_port_button'].set_sensitive(False)
            view['add_output_port_button'].set_sensitive(False)
            view['remove_output_port_button'].set_sensitive(False)
            view['add_scoped_variable_button'].set_sensitive(False)
            view['remove_scoped_variable_button'].set_sensitive(False)

        view.inputs_view.show()
        view.outputs_view.show()
        view.scopes_view.show()
        view.outcomes_view.show()
        view.transitions_view.show()
        view.data_flows_view.show()

        # show scoped variables if show content is enabled -> if disabled the tab stays and indicates a container state
        if isinstance(self.model, LibraryStateModel) and not self.model.show_content():
            view.scopes_view.hide()
            view.linkage_overview.scope_view.hide()

        # Container states do not have a source editor and library states does not show there source code
        # Thus, for those states we do not have to add the source controller and can hide the source code tab
        # logger.info("init state: {0}".format(model))
        lib_with_and_ES_as_root = isinstance(self.model, LibraryStateModel) and \
                                  not isinstance(self.model.state_copy, ContainerStateModel)
        if not isinstance(self.model, ContainerStateModel) and not isinstance(self.model, LibraryStateModel) or \
                lib_with_and_ES_as_root:
            view.source_view.show()
            if isinstance(self.model, LibraryStateModel) and not self.model.show_content():
                view.remove_source_tab()
            view.remove_scoped_variables_tab()
        else:
            view.scopes_view.show()
            if isinstance(self.model, LibraryStateModel) and \
                    (not self.model.show_content() or not isinstance(self.model.state_copy, ContainerStateModel)):
                view.remove_scoped_variables_tab()
            view.remove_source_tab()

        if global_gui_config.get_config_value("SEMANTIC_DATA_MODE", False):
            view.bring_tab_to_the_top('Semantic Data')
        else:
            if isinstance(self.model.state, LibraryState):
                view.bring_tab_to_the_top('Description')
            else:
                view.bring_tab_to_the_top('Linkage Overview')

        if isinstance(self.model, ContainerStateModel):
            self.scopes_ctrl.reload_scoped_variables_list_store()

        plugins.run_hook("post_state_editor_register_view", self)

    def rename(self):
        state_overview_controller = self.get_controller('properties_ctrl')
        state_overview_controller.rename()

    @ExtendedController.observe("meta_signal", signal=True)
    def show_content_changed(self, model, prop_name, info):
        meta_signal_message = info['arg']
        if meta_signal_message.change == 'show_content':
            if self.model.meta['gui']['show_content']:
                if isinstance(model.state_copy, ContainerStateModel):
                    self.view.insert_scoped_variables_tab()
                else:
                    self.view.insert_source_tab()
                self.view.linkage_overview.scope_view.show()
            else:
                if isinstance(model.state_copy, ContainerStateModel):
                    self.view.remove_scoped_variables_tab()
                else:
                    self.view.remove_source_tab()
                self.view.linkage_overview.scope_view.hide()

    @ExtendedController.observe("destruction_signal", signal=True)
    def state_destruction(self, model, prop_name, info):
        """ Close state editor when state is being destructed """
        import rafcon.gui.singleton as gui_singletons
        states_editor_ctrl = gui_singletons.main_window_controller.get_controller('states_editor_ctrl')
        state_identifier = states_editor_ctrl.get_state_identifier(self.model)
        states_editor_ctrl.close_page(state_identifier, delete=True)
