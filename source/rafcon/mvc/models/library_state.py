import os
import copy

from rafcon.statemachine.states.state import State
from rafcon.statemachine.states.library_state import LibraryState
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.singleton import global_storage

from rafcon.mvc.models.abstract_state import AbstractStateModel
from rafcon.mvc.models.data_port import DataPortModel
from rafcon.mvc.models.outcome import OutcomeModel

from rafcon.mvc.models.abstract_state import state_to_state_model

from rafcon.utils import log
logger = log.get_logger(__name__)


class LibraryStateModel(AbstractStateModel):
    """This model class manages a LibraryState

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state).

    :param State state: The state to be managed
     """

    state_copy = None

    def __init__(self, state, parent=None, meta=None):
        """Constructor
        """
        super(LibraryStateModel, self).__init__(state, parent, meta)
        assert isinstance(state, LibraryState)

        model_class = state_to_state_model(state.state_copy)
        if model_class is not None:
            self.state_copy = model_class(state.state_copy, parent=self)
        else:
            logger.error("Unknown state type '{type:s}'. Cannot create model.".format(type=type(state)))

    def _load_input_data_port_models(self):
        """Reloads the input data port models directly from the the state
        """
        self.input_data_ports = []
        for input_data_port in self.state.input_data_ports.itervalues():
            self.input_data_ports.append(DataPortModel(input_data_port, self))

    def _load_output_data_port_models(self):
        """Reloads the output data port models directly from the the state
        """
        self.output_data_ports = []
        for output_data_port in self.state.output_data_ports.itervalues():
            self.output_data_ports.append(DataPortModel(output_data_port, self))

    def _load_outcome_models(self):
        """Reloads the input data port models directly from the the state
        """
        self.outcomes = []
        for outcome in self.state.outcomes.itervalues():
            self.outcomes.append(OutcomeModel(outcome, self))

    # ---------------------------------------- storage functions ---------------------------------------------
    def load_meta_data_for_state(self):
        meta_path = os.path.join(self.state.get_file_system_path(), StateMachineStorage.GRAPHICS_FILE)
        if os.path.exists(meta_path):
            tmp_meta = global_storage.storage_utils.load_dict_from_yaml(meta_path)

            # For backwards compatibility
            # move all meta data from editor to editor_opengl
            self.overwrite_editor_meta(tmp_meta)

            for input_data_port_model in self.input_data_ports:
                i_id = input_data_port_model.data_port.data_port_id
                self.overwrite_editor_meta(tmp_meta["input_data_port" + str(i_id)])
                input_data_port_model.meta = tmp_meta["input_data_port" + str(i_id)]
                del tmp_meta["input_data_port" + str(i_id)]
            for output_data_port_model in self.output_data_ports:
                o_id = output_data_port_model.data_port.data_port_id
                self.overwrite_editor_meta(tmp_meta["output_data_port" + str(o_id)])
                output_data_port_model.meta = tmp_meta["output_data_port" + str(o_id)]
                del tmp_meta["output_data_port" + str(o_id)]

            # check the type implicitly by checking if the state has the states attribute
            if hasattr(self.state, 'states'):
                for transition_model in self.transitions:
                    t_id = transition_model.transition.transition_id
                    self.overwrite_editor_meta(tmp_meta["transition" + str(t_id)])
                    transition_model.meta = tmp_meta["transition" + str(t_id)]
                    del tmp_meta["transition" + str(t_id)]
                for data_flow_model in self.data_flows:
                    d_id = data_flow_model.data_flow.data_flow_id
                    self.overwrite_editor_meta(tmp_meta["data_flow" + str(d_id)])
                    data_flow_model.meta = tmp_meta["data_flow" + str(d_id)]
                    del tmp_meta["data_flow" + str(d_id)]
                for scoped_variable_model in self.scoped_variables:
                    s_id = scoped_variable_model.scoped_variable.data_port_id
                    self.overwrite_editor_meta(tmp_meta["scoped_variable" + str(s_id)])
                    scoped_variable_model.meta = tmp_meta["scoped_variable" + str(s_id)]
                    del tmp_meta["scoped_variable" + str(s_id)]
            # assign the meta data to the state
            self.meta = tmp_meta
        # Print info only if the state has a location different from the tmp directory
        elif meta_path[0:5] != '/tmp/':
            logger.info("State '{0}' has no meta data. It will now be generated automatically.".format(self.state.name))

    def copy_meta_data_from_state_model(self, source_state):

        self.meta = copy.deepcopy(source_state.meta)
        counter = 0
        for input_data_port_model in self.input_data_ports:
            input_data_port_model.meta = \
                copy.deepcopy(source_state.input_data_ports[counter].meta)
            counter += 1
        counter = 0
        for output_data_port_model in self.output_data_ports:
            output_data_port_model.meta = \
                copy.deepcopy(source_state.output_data_ports[counter].meta)
            counter += 1

        if hasattr(self.state, 'states'):
            counter = 0
            for transition_model in self.transitions:
                transition_model.meta = \
                    copy.deepcopy(source_state.transitions[counter].meta)
                counter += 1
            counter = 0
            for data_flow_model in self.data_flows:
                data_flow_model.meta = \
                    copy.deepcopy(source_state.data_flows[counter].meta)
                counter += 1
            counter = 0
            for scoped_variable_model in self.scoped_variables:
                scoped_variable_model.meta = \
                    copy.deepcopy(source_state.scoped_variables[counter].meta)
                counter += 1

    def store_meta_data_for_state(self):
        meta_path = os.path.join(self.state.get_file_system_path(), StateMachineStorage.GRAPHICS_FILE)

        for input_data_port_model in self.input_data_ports:
            self.meta["input_data_port" + str(input_data_port_model.data_port.data_port_id)] = \
                input_data_port_model.meta

        for output_data_port_model in self.output_data_ports:
            self.meta["output_data_port" + str(output_data_port_model.data_port.data_port_id)] = \
                output_data_port_model.meta

        # add transition meta data and data_flow meta data to the state meta data before saving it to a yaml file
        if hasattr(self.state, 'states'):
            for transition_model in self.transitions:
                self.meta["transition" + str(transition_model.transition.transition_id)] = transition_model.meta
            for data_flow_model in self.data_flows:
                self.meta["data_flow" + str(data_flow_model.data_flow.data_flow_id)] = data_flow_model.meta
            for scoped_variable_model in self.scoped_variables:
                self.meta["scoped_variable" + str(scoped_variable_model.scoped_variable.data_port_id)] =\
                    scoped_variable_model.meta

        global_storage.storage_utils.write_dict_to_yaml(self.meta, meta_path)

    @staticmethod
    def dataport_compare_method(treemodel, iter1, iter2, user_data=None):
        path1 = treemodel.get_path(iter1)[0]
        path2 = treemodel.get_path(iter2)[0]
        name1 = treemodel[path1][0]
        name2 = treemodel[path2][0]
        name1_as_bits = ' '.join(format(ord(x), 'b') for x in name1)
        name2_as_bits = ' '.join(format(ord(x), 'b') for x in name2)
        if name1_as_bits == name2_as_bits:
            return 0
        elif name1_as_bits > name2_as_bits:
            return 1
        else:
            return -1
