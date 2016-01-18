from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.controllers.state_overview import StateOverviewController
from rafcon.mvc.controllers.source_editor import  SourceEditorController
from rafcon.mvc.controllers.io_data_port_list import DataPortListController
from rafcon.mvc.controllers.scoped_variable_list import ScopedVariableListController
from rafcon.mvc.controllers.state_outcomes import StateOutcomesEditorController
from rafcon.mvc.controllers.linkage_overview import LinkageOverviewController

from rafcon.mvc.controllers.state_transitions import StateTransitionsEditorController
from rafcon.mvc.controllers.state_data_flows import StateDataFlowsEditorController
from rafcon.mvc.models import ContainerStateModel
from rafcon.mvc import gui_helper
from rafcon.mvc.config import global_gui_config
from rafcon.statemachine.states.library_state import LibraryState
from rafcon.utils import constants
from rafcon.utils import log

logger = log.get_logger(__name__)


class StateEditorController(ExtendedController):
    """Controller handles the organization of the Logic-Data oriented State-Editor.
    Widgets concerning logic flow (outcomes and transitions) are grouped in the Logic Linkage expander.
    Widgets concerning data flow (data-ports and data-flows) are grouped in the data linkage expander.
    """

    icons = {
        "Source": constants.ICON_SOURCE,
        "Data Linkage": constants.ICON_DLINK,
        "Logical Linkage": constants.ICON_LLINK,
        "Linkage Overview": constants.ICON_OVERV,
        "Description": constants.ICON_DESC
    }

    def __init__(self, model, view):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)

        self.add_controller('properties_ctrl', StateOverviewController(model, view['properties_view']))

        self.add_controller('inputs_ctrl', DataPortListController(model, view['inputs_view'], "input"))
        self.add_controller('outputs_ctrl', DataPortListController(model, view['outputs_view'], "output"))
        self.add_controller('scoped_ctrl', ScopedVariableListController(model, view['scopes_view']))
        self.add_controller('outcomes_ctrl', StateOutcomesEditorController(model, view['outcomes_view']))

        self.add_controller('transitions_ctrl', StateTransitionsEditorController(model, view['transitions_view']))
        self.add_controller('data_flows_ctrl', StateDataFlowsEditorController(model, view['data_flows_view']))

        self.add_controller('linkage_overview_ctrl', LinkageOverviewController(model, view['linkage_overview']))

        view['inputs_view'].show()
        view['outputs_view'].show()
        view['scopes_view'].show()
        view['outcomes_view'].show()
        view['transitions_view'].show()
        view['data_flows_view'].show()

        # Container states do not have a source editor and library states does not show there source code
        # Thus, for those states we do not have to add the source controller and can hide the source code tab
        # logger.info("init state: {0}".format(model))
        if not isinstance(model, ContainerStateModel) and not isinstance(model.state, LibraryState):
            self.add_controller('source_ctrl', SourceEditorController(model, view['source_view']))
            view['source_view'].show()
            scoped_var_page = view['ports_notebook'].page_num(view['scoped_variable_vbox'])
            view['ports_notebook'].remove_page(scoped_var_page)
        else:
            view['scopes_view'].show()
            source_page = view['main_notebook_1'].page_num(view['source_viewport'])
            view['main_notebook_1'].remove_page(source_page)

        view['description_text_view'].connect('focus-out-event', self.change_description)
        view['description_text_view'].connect('size-allocate', self.scroll_to_bottom)

        for i in range(view["main_notebook_1"].get_n_pages()):
            child = view["main_notebook_1"].get_nth_page(i)
            tab_label = view["main_notebook_1"].get_tab_label(child)
            if global_gui_config.get_config_value("USE_ICONS_AS_TAB_LABELS", True):
                tab_label_text = tab_label.get_text()
                view["main_notebook_1"].set_tab_label(child, gui_helper.create_tab_header_label(tab_label_text,
                                                                                                self.icons))
            else:
                tab_label.set_angle(270)
            view["main_notebook_1"].set_tab_reorderable(child, True)
            view["main_notebook_1"].set_tab_detachable(child, True)

        for i in range(view["main_notebook_2"].get_n_pages()):
            child = view["main_notebook_2"].get_nth_page(i)
            tab_label = view["main_notebook_2"].get_tab_label(child)
            if global_gui_config.get_config_value("USE_ICONS_AS_TAB_LABELS", True):
                tab_label_text = tab_label.get_text()
                view["main_notebook_2"].set_tab_label(child, gui_helper.create_tab_header_label(tab_label_text,
                                                                                                self.icons))
            else:
                tab_label.set_angle(270)
            view["main_notebook_2"].set_tab_reorderable(child, True)
            view["main_notebook_2"].set_tab_detachable(child, True)

        if isinstance(model, ContainerStateModel):
            self.get_controller('scoped_ctrl').reload_scoped_variables_list_store()

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        view['new_input_port_button'].connect('clicked',
                                              self.get_controller('inputs_ctrl').on_new_port_button_clicked)
        view['new_output_port_button'].connect('clicked',
                                               self.get_controller('outputs_ctrl').on_new_port_button_clicked)
        view['new_scoped_variable_button'].connect('clicked',
                                                   self.get_controller(
                                                       'scoped_ctrl').on_new_scoped_variable_button_clicked)

        view['delete_input_port_button'].connect('clicked',
                                                 self.get_controller('inputs_ctrl').on_delete_port_button_clicked)
        view['delete_output_port_button'].connect('clicked',
                                                  self.get_controller('outputs_ctrl').on_delete_port_button_clicked)
        view['delete_scoped_variable_button'].connect('clicked',
                                                      self.get_controller(
                                                          'scoped_ctrl').on_delete_scoped_variable_button_clicked)

        if isinstance(self.model.state, LibraryState):
            view['new_input_port_button'].set_sensitive(False)
            view['delete_input_port_button'].set_sensitive(False)
            view['new_output_port_button'].set_sensitive(False)
            view['delete_output_port_button'].set_sensitive(False)
            view['new_scoped_variable_button'].set_sensitive(False)
            view['delete_scoped_variable_button'].set_sensitive(False)

        state = self.model.state
        if isinstance(state, LibraryState):
            state = self.model.state.state_copy
            view['description_text_view'].set_sensitive(False)
        if state.description is not None:
            view['description_text_view'].get_buffer().set_text(state.description)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        # self.adapt(self.__state_property_adapter("name", "input_name"))

    def scroll_to_bottom(self, widget, data=None):
        scroller = self.view['description_scroller']
        adj = scroller.get_vadjustment()
        adj.set_value(adj.get_upper() - adj.get_page_size())

    def change_description(self, textview, otherwidget):
        if isinstance(self.model.state, LibraryState):
            return
        tbuffer = textview.get_buffer()
        entry_text = tbuffer.get_text(tbuffer.get_start_iter(), tbuffer.get_end_iter())

        if len(entry_text) > 0 and not self.model.state.description == entry_text:
            logger.debug("Changed description of state {0}".format(self.model.state.name))
            self.model.state.description = entry_text
            self.view['description_text_view'].get_buffer().set_text(self.model.state.description)

    def rename(self):
        state_overview_controller = self.get_controller('properties_ctrl')
        state_overview_controller.rename()

    @ExtendedController.observe("state_type_changed_signal", signal=True)
    def state_type_changed(self, model, prop_name, info):
        """Reopen state editor when state type is changed

        When the type of the observed state changes, a new model is created. The look of this controller'S view
        depends on the kind of model. Therefore, we have to destroy this editor and open a new one with the new model.
        """
        from rafcon.mvc.singleton import main_window_controller
        msg = info['arg']
        new_state_m = msg.new_state_m
        states_editor_ctrl = main_window_controller.get_controller('states_editor_ctrl')
        states_editor_ctrl.recreate_state_editor(self.model, new_state_m)
