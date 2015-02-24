from gtkmvc import ModelMT
from mvc.models.state_machine import StateMachineModel
from statemachine.state_machine_manager import StateMachineManager
from utils.vividict import Vividict


class StateMachineManagerModel(ModelMT):
    """This model class manages a StateMachineManager

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state machine manager).
    Additional to the data of the StateMachineManager its model and the models of statemachines hold by those
    these model stores and made observable the selected state machine of the view which have not to be the same
    as the active running one.

    :param StateMachineManager state_machine_manager: The state machine manager to be managed
    """

    state_machine_manager = None
    selected_state_machine = None
    state_machines = {}

    __observables__ = ("state_machine_manager", "selected_state_machine", "state_machines")

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

        self.selected_state_machine = self.state_machines.keys()[0]

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

    @ModelMT.observe("state_machine_manager", after=True)
    def model_changed(self, model, prop_name, info):
        pass

    def get_active_state_machine_model(self):
        return self.state_machines[self.state_machine_manager.active_state_machine_id]

