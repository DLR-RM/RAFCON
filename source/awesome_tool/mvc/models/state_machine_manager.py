from gtkmvc import ModelMT
from mvc.models.state_machine import StateMachineModel, Selection
from statemachine.state_machine_manager import StateMachineManager
from utils.vividict import Vividict

from utils import log
logger = log.get_logger(__name__)


class StateMachineManagerModel(ModelMT):
    """This model class manages a StateMachineManager

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state machine manager).
    Additional to the data of the StateMachineManager its model and the models of statemachines hold by those
    these model stores and made observable the selected state machine of the view which have not to be the same
    as the active running one.

    :param StateMachineManager state_machine_manager: The state machine manager to be managed
    """

    state_machine_manager = None
    selected_state_machine_id = None
    state_machines = {}

    __observables__ = ("state_machine_manager", "selected_state_machine_id", "state_machines")

    def __init__(self, state_machine_manager, meta=None):
        """Constructor
        """
        ModelMT.__init__(self)  # pass columns as separate parameters
        self.register_observer(self)

        assert isinstance(state_machine_manager, StateMachineManager)

        self.state_machine_manager = state_machine_manager
        self.state_machines = {}
        for sm_id, sm in state_machine_manager.state_machines.iteritems():
            self.state_machines[sm_id] = StateMachineModel(sm)

        self.selected_state_machine_id = self.state_machines.keys()[0]

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

    def delete_state_machine_models(self):
        sm_keys = self.state_machines.keys()
        for key in sm_keys:
            del self.state_machines[key]

    @ModelMT.observe("state_machine_manager", after=True)
    def model_changed(self, model, prop_name, info):
        if info["method_name"] == "add_state_machine":
            logger.debug("Add new state machine model ... ")
            for sm_id, sm in self.state_machine_manager.state_machines.iteritems():
                if sm_id not in self.state_machines:
                    logger.debug("Create new state machine model for state machine with id %s", sm.state_machine_id)
                    self.state_machines[sm_id] = StateMachineModel(sm)
                    #TODO: check when meta data cannot be loaded
                    logger.debug("Load meta data for state machine with state machine id %s " % str(sm_id))
                    self.state_machines[sm_id].root_state.load_meta_data_for_state()
                    self.selected_state_machine_id = sm_id
        elif info["method_name"] == "remove_state_machine":
            sm_id_to_delete = None
            for sm_id, sm_m in self.state_machines.iteritems():
                if sm_id not in self.state_machine_manager.state_machines:
                    sm_id_to_delete = sm_id
                    if self.selected_state_machine_id == sm_id:
                        self.selected_state_machine_id = None
                    break

            logger.debug("Delete state machine model for state machine with id %s", sm_id_to_delete)
            del self.state_machines[sm_id_to_delete]

    def get_selected_state_machine_model(self):
        return self.state_machines[self.selected_state_machine_id]

    # def get_active_state_machine_model(self):
    #     return self.state_machines[self.state_machine_manager.active_state_machine_id]

