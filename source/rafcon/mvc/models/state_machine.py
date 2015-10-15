from gtkmvc import ModelMT, Signal

from rafcon.mvc.models.abstract_state import MetaSignalMsg, Notification
from rafcon.mvc.models import ContainerStateModel, StateModel
from rafcon.mvc.selection import Selection

from rafcon.statemachine.state_machine import StateMachine
from rafcon.statemachine.states.container_state import ContainerState

from rafcon.mvc.config import global_gui_config
from rafcon.utils.vividict import Vividict
from rafcon.utils import log
logger = log.get_logger(__name__)


class StateMachineModel(ModelMT):
    """This model class manages a :class:`rafcon.statemachine.state_machine.StateMachine`

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state machine).

    :param StateMachine state_machine: The state machine to be controlled and modified
    """

    state_machine = None
    selection = None
    root_state = None
    meta_signal = Signal()
    state_meta_signal = Signal()

    __observables__ = ("state_machine", "root_state", "selection", "meta_signal", "state_meta_signal")

    def __init__(self, state_machine, sm_manager_model, meta=None):
        """Constructor
        """
        ModelMT.__init__(self)  # pass columns as separate parameters

        assert isinstance(state_machine, StateMachine)

        self.state_machine = state_machine

        root_state = self.state_machine.root_state
        if isinstance(root_state, ContainerState):
            self.root_state = ContainerStateModel(root_state)
        else:
            self.root_state = StateModel(root_state)

        self.root_state.register_observer(self)
        self.register_observer(self)

        self.sm_manager_model = sm_manager_model

        self.selection = Selection()

        from rafcon.mvc.history import History
        HISTORY_ENABLED = global_gui_config.get_config_value('HISTORY_ENABLED')
        logger.info("is history enabled: %s" % HISTORY_ENABLED)
        self.history = History(self)
        if not HISTORY_ENABLED:
            self.history.fake = True

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()
        self.meta_signal = Signal()
        self.state_meta_signal = Signal()

        self.temp = Vividict()

    @ModelMT.observe("state_machine", after=True)
    def root_state_changed(self, model, prop_name, info):
        if info['method_name'] == 'root_state':
            if self.state_machine.root_state != self.root_state.state:
                new_root_state = self.state_machine.root_state
                self.root_state.unregister_observer(self)
                if isinstance(new_root_state, ContainerState):
                    self.root_state = ContainerStateModel(new_root_state)
                else:
                    self.root_state = StateModel(new_root_state)
                self.root_state.register_observer(self)

    @ModelMT.observe("state_machine", after=True)
    def marked_dirty_flag_changed(self, model, prop_name, info):
        if not self.state_machine.old_marked_dirty == self.state_machine.marked_dirty:
            if self.state_machine.marked_dirty:
                self.sm_manager_model.state_machine_mark_dirty = self.state_machine.state_machine_id
            else:
                self.sm_manager_model.state_machine_un_mark_dirty = self.state_machine.state_machine_id

    @ModelMT.observe("state", after=True)
    def name_of_root_state_changed(self, model, prop_name, info):
        if info["method_name"] is "name":
            if self.state_machine.marked_dirty:
                self.sm_manager_model.state_machine_mark_dirty = self.state_machine.state_machine_id
            else:
                self.sm_manager_model.state_machine_un_mark_dirty = self.state_machine.state_machine_id


    @ModelMT.observe("state", before=True)
    @ModelMT.observe("outcomes", before=True)
    @ModelMT.observe("is_start", before=True)
    @ModelMT.observe("states", before=True)
    @ModelMT.observe("transitions", before=True)
    @ModelMT.observe("data_flows", before=True)
    @ModelMT.observe("input_data_ports", before=True)
    @ModelMT.observe("scoped_variables", before=True)
    def root_state_model_before_change(self, model, prop_name, info):
        if not self._list_modified(prop_name, info):
            self.state_machine.root_state_before_change(model=info['model'], prop_name=info['prop_name'],
                                                        instance=info['instance'],
                                                        method_name=info['method_name'], args=info['args'],
                                                        kwargs=info['kwargs'])


    @ModelMT.observe("state", after=True)
    @ModelMT.observe("outcomes", after=True)
    @ModelMT.observe("is_start", after=True)
    @ModelMT.observe("states", after=True)
    @ModelMT.observe("transitions", after=True)
    @ModelMT.observe("data_flows", after=True)
    @ModelMT.observe("input_data_ports", after=True)
    @ModelMT.observe("output_data_ports", after=True)
    @ModelMT.observe("scoped_variables", after=True)
    def root_state_model_after_change(self, model, prop_name, info):
        if not self._list_modified(prop_name, info):
            self.state_machine.root_state_after_change(model=info['model'], prop_name=info['prop_name'],
                                                       instance=info['instance'],
                                                       method_name=info['method_name'], result=info['result'],
                                                       args=info['args'], info=info['kwargs'])

    @ModelMT.observe("meta_signal", signal=True)
    def meta_changed(self, model, prop_name, info):
        if model is not self:
            msg = info.arg
            self.state_meta_signal.emit(msg)

    @staticmethod
    def _list_modified(prop_name, info):
        """Check whether the given operation is a list operation

        The function checks whether the property that has been changed is a list. If so, the operation is investigated
        further. If the operation is a basic list operation, the function return True.
        :param prop_name: The property that was changed
        :param info: Dictionary with information about the operation
        :return: True if the operation was a list operation, False else
        """
        if prop_name in ["states", "transitions", "data_flows", "input_data_ports", "output_data_ports",
                         "scoped_variables", "outcomes"]:
            if info['method_name'] in ["append", "extend", "insert", "pop", "remove", "reverse", "sort",
                                       "__delitem__", "__setitem__"]:
                return True
        return False

    def get_state_model_by_path(self, path):
        path_elems = path.split('/')
        path_elems.pop(0)
        current_state_model = self.root_state
        for state_id in path_elems:
            if state_id in current_state_model.states:
                current_state_model = current_state_model.states[state_id]
            else:
                logger.error("path element %s of '%s' could not be found root_state is %s %s" %
                             (current_state_model.state.get_path(), path.split('/'),
                              self.root_state.state.name, self.root_state.state.state_id))
                assert False
        return current_state_model
