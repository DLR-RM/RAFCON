from gtkmvc import ModelMT

from gtk import ListStore
import gtk
import os

from awesome_tool.statemachine.states.state import State
from table import TableDescriptor, ColumnDescriptor, AttributesRowDescriptor
from awesome_tool.utils.vividict import Vividict
from awesome_tool.mvc.models.data_port import DataPortModel
from awesome_tool.mvc.models.outcome import OutcomeModel
import awesome_tool.statemachine.singleton
from awesome_tool.statemachine.storage.storage import Storage
from awesome_tool.utils import log
from awesome_tool.statemachine.enums import StateType

from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.states.state import DataPort

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

    _table = TableDescriptor()
    _table.add_column(ColumnDescriptor(0, 'key', str))
    _table.add_column(ColumnDescriptor(1, 'value', str))
    _table.add_column(ColumnDescriptor(2, 'name', str))
    _table.add_column(ColumnDescriptor(3, 'editable', bool))
    _table.add_row(AttributesRowDescriptor(0, 'state_id', 'ID', editable=True))
    _table.add_row(AttributesRowDescriptor(1, 'name', 'Name'))

    def __init__(self, state, parent=None, meta=None):
        """Constructor
        """
        ModelMT.__init__(self)
        assert isinstance(state, State)

        self.state = state

        # True if no parent (for root_state) or state is parent start_state else False
        self.is_start = True if state.parent is None or state.state_id == state.parent.start_state else False

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

        if isinstance(parent, StateModel):
            self.parent = parent
        else:
            self.parent = None

        self.list_store = ListStore(*self._table.get_column_types())
        self.update_attributes()

        self.register_observer(self)
        self.input_data_ports = []
        self.output_data_ports = []
        self.outcomes = []
        self.reload_input_data_port_models()
        self.reload_output_data_port_models()

    def update_attributes(self):
        """Update table model with state model

        Clears all table rows from the model. Then add them back again by iterating over the state attributes and
        adding all items to the table model.
        """
        self.list_store.clear()
        for row in self._table.rows:
            key = row.key
            self.list_store.append([key, self.state.__getattribute__(key), row.name, row.editable])

    def update_row(self, row, value):
        """Update the list store row with the new model

        This method is being called by the controller, when the value within a TreeView has been changed by the user.
        The methods tries to update the state model with the new value and returns the exception object if the valid
        is not valid.

        :param row: The row/path within the ListStore
        :param value: The value set by the user
        :return: True when successful, the exception otherwise
        """
        attr = self.list_store[row][0]
        try:
            self.state.__setattr__(attr, value)
        except ValueError as error:
            return error
        return True

    def is_element_of_self(self, instance):

        if isinstance(instance, DataPort) and instance.data_port_id in self.state.input_data_ports:
            return True
        if isinstance(instance, DataPort) and instance.data_port_id in self.state.output_data_ports:
            return True
        if isinstance(instance, Outcome) and instance.outcome_id in self.state.outcomes:
            return True
        return False

    @ModelMT.observe("state", after=True, before=True)
    def model_changed(self, model, name, info):
        """This method notifies the model lists and the parent state about changes

        The method is called each time, the model is changed. This happens, when the state itself changes or one of
        its children (outcomes, ports) changes. Changes of the children cannot be observed directly,
        therefore children notify their parent about their changes by calling this method.
        This method then checks, what has been changed by looking at the method that caused the change. It then
        notifies the list in which the change happened about the change.
        E.g. one input data port changes its name. The model of the port observes itself and notifies the parent (
        i.e. the state model) about the change by calling this method with the information about the change. This
        method recognizes that the method "modify_input_data_port" caused the change and therefore triggers a notify
        on the list if input data port models.
        "_notify_method_before" is used as trigger method when the changing function is entered and
        "_notify_method_after" is used when the changing function returns. This changing function in the example
        would be "modify_input_data_port".
        :param model: The model that was changed
        :param name: The property that was changed
        :param info: Information about the change (e.g. the name of the changing function)
        """

        # mark the state machine this state belongs to as dirty
        own_sm_id = awesome_tool.statemachine.singleton.state_machine_manager.get_sm_id_for_state(self.state)
        awesome_tool.statemachine.singleton.global_storage.mark_dirty(own_sm_id)
    
        # TODO delete # prints if there is no bug ... latest 15th of march
        if hasattr(info, 'before') and info['before'] and (self is model.parent or isinstance(info.instance, DataPort)):
            # print info.method_name, "modify_input_data_port" in info.method_name, info
            if "modify_input_data_port" in info.method_name or \
                    (isinstance(info.instance, DataPort) and info.instance.data_port_id in self.state.input_data_ports):
                # print "NOTIFY before INPUT_DATA_PORTS"
                self.input_data_ports._notify_method_before(info.instance, "input_data_port_change", (model,), info)
            if "modify_output_data_port" in info.method_name or \
                    (isinstance(info.instance, DataPort) and info.instance.data_port_id in self.state.output_data_ports):
                # print "NOTIFY before OUTPUT_DATA_PORTS"
                self.output_data_ports._notify_method_before(info.instance, "output_data_port_change", (model,), info)

        elif hasattr(info, 'after') and info['after'] and (self is model.parent or isinstance(info.instance, DataPort)):
            if "modify_input_data_port" in info.method_name or \
                    (isinstance(info.instance, DataPort) and info.instance.data_port_id in self.state.input_data_ports):
                # print "NOTIFY after INPUT_DATA_PORTS"
                self.input_data_ports._notify_method_after(info.instance, "input_data_port_change", None, (model,), info)
            if "modify_output_data_port" in info.method_name or \
                    (isinstance(info.instance, DataPort) and info.instance.data_port_id in self.state.output_data_ports):
                # print "NOTIFY after OUTPUT_DATA_PORTS"
                self.output_data_ports._notify_method_after(info.instance, "output_data_port_change", None, (model,), info)

        if hasattr(info, 'before') and info['before'] and self is model.parent:
            if "modify_outcome" in info.method_name or info.method_name in ['add_outcome', 'remove_outcome'] or \
                    isinstance(info.instance, Outcome):
                # print "NOTIFY before OUTCOMES"
                self.outcomes._notify_method_before(info.instance, "outcome_change", (model,), info)

        elif hasattr(info, 'after') and info['after'] and self is model.parent:
            if "modify_outcome" in info.method_name or info.method_name in ['add_outcome', 'remove_outcome'] or \
                    isinstance(info.instance, Outcome):
                # print "NOTIFY after OUTCOMES"
                self.outcomes._notify_method_after(info.instance, "outcome_change", None, (model,), info)

        # Notify the parent state about the change (this causes a recursive call up to the root state)
        if self.parent is not None:
            self.parent.model_changed(model, name, info)

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

    @ModelMT.observe("state", after=True)
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
        meta_path = os.path.join(self.state.script.path, Storage.GRAPHICS_FILE)
        if os.path.exists(meta_path):
            tmp_meta = awesome_tool.statemachine.singleton.global_storage.load_dict_from_yaml(meta_path)

            if self.state.state_type is not StateType.EXECUTION:
                # add meta to transition and data flow
                counter = 0
                for transition_model in self.transitions:
                    transition_model.meta = tmp_meta["transition" + str(counter)]
                    counter += 1
                counter = 0
                for data_flow_model in self.data_flows:
                    data_flow_model.meta = tmp_meta["data_flow" + str(counter)]
                    counter += 1
                # delete transition and data flow from tmp data
                for i in range(len(self.transitions)):
                    del tmp_meta["transition" + str(i)]
                for i in range(len(self.data_flows)):
                    del tmp_meta["data_flow" + str(i)]
            # assign the meta data to the state
            self.meta = tmp_meta
        else:
            logger.warn("path to load meta data for state model of state %s does not exist" % self.state.name)

    def store_meta_data_for_state(self):
        #logger.debug("store graphics file to yaml for state model of state %s" % self.state.name)
        meta_path = os.path.join(self.state.script.path, Storage.GRAPHICS_FILE)

        # add transition meta data and data_flow meta data to the state meta data before saving it to a yaml file
        if self.state.state_type is not StateType.EXECUTION:
            counter = 0
            for transition_model in self.transitions:
                self.meta["transition" + str(counter)] = transition_model.meta
                counter += 1
            counter = 0
            for data_flow_model in self.data_flows:
                self.meta["data_flow" + str(counter)] = data_flow_model.meta
                counter += 1

        awesome_tool.statemachine.singleton.global_storage.write_dict_to_yaml(self.meta, meta_path)

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
