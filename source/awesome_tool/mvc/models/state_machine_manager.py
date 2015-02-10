from gtkmvc import ModelMT
from statemachine.state_machine_manager import StateMachineManager
from mvc.models.container_state import ContainerStateModel
from utils.vividict import Vividict


class StateMachineManagerModel(ModelMT):
    """This model class manages a StateMachineManager

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state machine manager).

    :param StateMachineManager state_machine_manager: The state machine manager to be managed
    """

    state_machine_manager = None
    root_state = None

    __observables__ = ("state_machine_manager", "root_state")

    def __init__(self, state_machine_manager, meta=None):
        """Constructor
        """
        ModelMT.__init__(self)  # pass columns as separate parameters
        self.register_observer(self)

        assert isinstance(state_machine_manager, StateMachineManager)

        self.state_machine_manager = state_machine_manager
        self.root_state = ContainerStateModel(self.state_machine_manager.root_state)

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

    @ModelMT.observe("state_machine_manager", after=True)
    def model_changed(self, model, prop_name, info):
        pass

