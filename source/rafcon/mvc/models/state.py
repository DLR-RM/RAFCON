from gtkmvc import ModelMT

from gtk import ListStore
import os
import copy

from rafcon.statemachine.states.state import State
from rafcon.utils.vividict import Vividict
from rafcon.mvc.models.data_port import DataPortModel
from rafcon.mvc.models.outcome import OutcomeModel
import rafcon.statemachine.singleton
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.utils import log

from rafcon.statemachine.outcome import Outcome
from rafcon.statemachine.data_port import DataPort

logger = log.get_logger(__name__)


class StateModel(ModelMT):
    """This model class manages a State

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state).

    :param State state: The state to be managed
     """

    is_start = None
    state = None
    outcomes = []
    input_data_ports = []
    output_data_ports = []

    __observables__ = ("state", "input_data_ports", "output_data_ports", "outcomes", "is_start")

    def __init__(self, state, parent=None, meta=None):
        """Constructor
        """
        ModelMT.__init__(self)
        assert isinstance(state, State)

        self.state = state

        # True if no parent (for root_state) or state is parent start_state_id else False
        self.is_start = True if not isinstance(state.parent, State) or state.state_id == state.parent.start_state_id else False

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

        self.temp = Vividict()

        if isinstance(parent, StateModel):
            self.parent = parent
        else:
            self.parent = None

        self.register_observer(self)
        self.input_data_ports = []
        self.output_data_ports = []
        self.outcomes = []
        self.reload_input_data_port_models()
        self.reload_output_data_port_models()
        self.reload_outcome_models()

    def is_element_of_self(self, instance):

        if isinstance(instance, DataPort) and instance.data_port_id in self.state.input_data_ports:
            return True
        if isinstance(instance, DataPort) and instance.data_port_id in self.state.output_data_ports:
            return True
        if isinstance(instance, Outcome) and instance.outcome_id in self.state.outcomes:
            return True
        return False

    @ModelMT.observe("state", after=True, before=True)
    def model_changed(self, model, prop_name, info):
        """This method notifies the model lists and the parent state about changes

        The method is called each time, the model is changed. This happens, when the state itself changes or one of
        its children (outcomes, ports) changes. Changes of the children cannot be observed directly,
        therefore children notify their parent about their changes by calling this method.
        This method then checks, what has been changed by looking at the method that caused the change. In the
        following, it notifies the list in which the change happened about the change.
        E.g. one input data port changes its name. The model of the port observes itself and notifies the parent (
        i.e. the state model) about the change by calling this method with the information about the change. This
        method recognizes that the method "modify_input_data_port" caused the change and therefore triggers a notify
        on the list if input data port models.
        "_notify_method_before" is used as trigger method when the changing function is entered and
        "_notify_method_after" is used when the changing function returns. This changing function in the example
        would be "modify_input_data_port".
        :param model: The model that was changed
        :param prop_name: The property that was changed
        :param info: Information about the change (e.g. the name of the changing function)
        """

        # If this model has been changed (and not one of its child states), then we have to update all child models
        # This must be done before notifying anybody else, because other may relay on the updated models
        if hasattr(info, 'after') and info['after'] and self.state == info['instance']:
            self.update_models(model, prop_name, info)

        # mark the state machine this state belongs to as dirty
        active_flag_responsible = info["method_name"] == "active" or info["method_name"] == "child_execution" or \
                                  info["method_name"] == "state_execution_status"
        if isinstance(model, StateModel) and prop_name == "state" and active_flag_responsible:
            # do not track the active flag when marking the sm dirty
            pass
        else:
            if self.state.get_sm_for_state():
                own_sm_id = self.state.get_sm_for_state().state_machine_id
                if own_sm_id is not None:
                    rafcon.statemachine.singleton.state_machine_manager.state_machines[own_sm_id].marked_dirty = True

        # TODO the modify observation to notify the list has to be changed in the manner, that the element-models
        # notify there parent with there own instance as argument

        changed_list = None
        cause = None

        if isinstance(model, DataPortModel) and model.parent is self:
            if model in self.input_data_ports:
                changed_list = self.input_data_ports
                cause = "input_data_port_change"
            elif model in self.output_data_ports:
                changed_list = self.output_data_ports
                cause = "output_data_port_change"

        # Outcomes are not modelled as class, therefore the model passed when adding/removing an outcome is the
        # parent StateModel. Thus we have to look at the method that caused a property change.
        elif "modify_outcome" in info.method_name and self is model or \
                isinstance(info.instance, Outcome) and self is model.parent:
            changed_list = self.outcomes
            cause = "outcome_change"

        if not (cause is None or changed_list is None):
            if hasattr(info, 'before') and info['before']:
                changed_list._notify_method_before(info.instance, cause, (model,), info)
            elif hasattr(info, 'after') and info['after']:
                changed_list._notify_method_after(info.instance, cause, None, (model,), info)

        # Notify the parent state about the change (this causes a recursive call up to the root state)
        if self.parent is not None:
            self.parent.model_changed(model, prop_name, info)

    def get_outcome_model(self, outcome_id):
        """Searches and return the outcome model in this state model
        :param outcome_id: The outcome id to be searched
        :return: The model of the data flow or None if it is not found
        """
        for outcome_m in self.outcomes:
            if outcome_m.outcome.outcome_id == outcome_id:
                return outcome_m
        return None

    def get_data_port_model(self, data_port_id):
        """Searches and returns the model of a data port of a given state

        The method searches a port with the given id in the data ports of the given state model. If the state model
        is a container state, not only the input and output data ports are looked at, but also the scoped variables.
        :param state_m: The state model to search the data port in
        :param data_port_id: The data port id to be searched
        :return: The model of the data port or None if it is not found
        """
        def find_port_in_list(data_port_list):
            """Helper method to search for a port within a given list

            :param data_port_list: The list to search the data port in
            :return: The model of teh data port or None if it is not found
            """
            for port_m in data_port_list:
                if port_m.data_port.data_port_id == data_port_id:
                    return port_m
            return None

        port_m = find_port_in_list(self.input_data_ports)
        if port_m is not None:
            return port_m
        port_m = find_port_in_list(self.output_data_ports)
        if port_m is not None:
            return port_m
        return None

    def get_model_info(self, model):
        model_key = None
        if model == "input_data_port":
            model_list = self.input_data_ports
            data_list = self.state.input_data_ports
            model_name = "data_port"
            model_class = DataPortModel
        elif model == "output_data_port":
            model_list = self.output_data_ports
            data_list = self.state.output_data_ports
            model_name = "data_port"
            model_class = DataPortModel
        elif model == "outcome":
            model_list = self.outcomes
            data_list = self.state.outcomes
            model_name = "outcome"
            model_class = OutcomeModel
        else:
            raise AttributeError("Wrong model specified!")
        return model_list, data_list, model_name, model_class, model_key

    def update_models(self, model, name, info):
        """ This method is always triggered when the core state changes

            It keeps the following models/model-lists consistent:
            input-data-port models
            output-data-port models
            outcome models
        """
        model_list = None
        if "input_data_port" in info.method_name:
            (model_list, data_list, model_name, model_class, model_key) = self.get_model_info("input_data_port")
        elif "output_data_port" in info.method_name:
            (model_list, data_list, model_name, model_class, model_key) = self.get_model_info("output_data_port")
        elif "outcome" in info.method_name:
            (model_list, data_list, model_name, model_class, model_key) = self.get_model_info("outcome")

        if model_list is not None:
            if "add" in info.method_name:
                self.add_missing_model(model_list, data_list, model_name, model_class, model_key)
            elif "remove" in info.method_name:
                self.remove_additional_model(model_list, data_list, model_name, model_key)

    def reload_input_data_port_models(self):
        """Reloads the input data port models directly from the the state
        """
        self.input_data_ports = []
        for input_data_port in self.state.input_data_ports.itervalues():
            self.input_data_ports.append(DataPortModel(input_data_port, self))

    def reload_output_data_port_models(self):
        """Reloads the output data port models directly from the the state
        """
        self.output_data_ports = []
        for output_data_port in self.state.output_data_ports.itervalues():
            self.output_data_ports.append(DataPortModel(output_data_port, self))

    def reload_outcome_models(self):
        """Reloads the input data port models directly from the the state
        """
        self.outcomes = []
        for outcome in self.state.outcomes.itervalues():
            self.outcomes.append(OutcomeModel(outcome, self))

    def add_missing_model(self, model_list, data_list, model_name, model_class, model_key):
        for data in data_list.itervalues():
            found = False
            for model_item in model_list:
                model = model_item if model_key is None else model_list[model_item]
                if data is getattr(model, model_name):
                    found = True
                    break
            if not found:
                if model_key is None:
                    model_list.append(model_class(data, self))
                else:
                    model_list[getattr(data, model_key)] = model_class(data, self)
                return

    def remove_additional_model(self, model_list, data_list, model_name, model_key):
        for model_item in model_list:
            model = model_item if model_key is None else model_list[model_item]
            found = False
            for data in data_list.itervalues():
                if data is getattr(model, model_name):
                    found = True
                    break
            if not found:
                if model_key is None:
                    model_list.remove(model)
                else:
                    del model_list[model_item]
                return

    # ---------------------------------------- storage functions ---------------------------------------------
    def load_meta_data_for_state(self):
        #logger.debug("load graphics file from yaml for state model of state %s" % self.state.name)
        meta_path = os.path.join(self.state.get_file_system_path(), StateMachineStorage.GRAPHICS_FILE)
        if os.path.exists(meta_path):
            tmp_meta = rafcon.statemachine.singleton.global_storage.storage_utils.load_dict_from_yaml(meta_path)

            # For backwards compatibility
            # move all meta data from editor to editor_opengl
            if "editor" in tmp_meta['gui']:
                tmp_meta['gui']['editor_opengl'] = copy.deepcopy(tmp_meta['gui']['editor'])
                del tmp_meta['gui']['editor']

            for input_data_port_model in self.input_data_ports:
                i_id = input_data_port_model.data_port.data_port_id
                input_data_port_model.meta = tmp_meta["input_data_port" + str(i_id)]
                del tmp_meta["input_data_port" + str(i_id)]
            for output_data_port_model in self.output_data_ports:
                o_id = output_data_port_model.data_port.data_port_id
                output_data_port_model.meta = tmp_meta["output_data_port" + str(o_id)]
                del tmp_meta["output_data_port" + str(o_id)]

            # check the type implicitly by checking if the state has the states attribute
            if hasattr(self.state, 'states'):
                for transition_model in self.transitions:
                    t_id = transition_model.transition.transition_id
                    transition_model.meta = tmp_meta["transition" + str(t_id)]
                    del tmp_meta["transition" + str(t_id)]
                for data_flow_model in self.data_flows:
                    d_id = data_flow_model.data_flow.data_flow_id
                    data_flow_model.meta = tmp_meta["data_flow" + str(d_id)]
                    del tmp_meta["data_flow" + str(d_id)]
                for scoped_variable_model in self.scoped_variables:
                    s_id = scoped_variable_model.scoped_variable.data_port_id
                    scoped_variable_model.meta = tmp_meta["scoped_variable" + str(s_id)]
                    del tmp_meta["scoped_variable" + str(s_id)]
            # assign the meta data to the state
            self.meta = tmp_meta
        # Print warning only if the state has a location different from the tmp directory
        elif meta_path[0:5] != '/tmp/':
            logger.warn("path to load meta data for state model of state '{0}' does not exist".format(self.state.name))

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
        #logger.debug("store graphics file to yaml for state model of state %s" % self.state.name)
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

        rafcon.statemachine.singleton.global_storage.storage_utils.write_dict_to_yaml(self.meta, meta_path)

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
