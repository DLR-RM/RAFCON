import rafcon.mvc.singleton
from gtkmvc import Observer
from rafcon.mvc.utils.notification_overview import NotificationOverview

from rafcon.utils import log
logger = log.get_logger(__name__)


class RootStateModificationObserver(Observer):

    def __init__(self):
        logger.info("Initiate gtkmvc observer")
        # register self as observer of rafcon.mvc.singleton.state_machine_manager_model
        Observer.__init__(self, rafcon.mvc.singleton.state_machine_manager_model)
        self.state_machine_manager_model = rafcon.mvc.singleton.state_machine_manager_model

        # register self as observer of already existing StateMachineModels in state_machines list
        for sm_id, sm_m in self.state_machine_manager_model.state_machines.iteritems():
            self.observe_model(sm_m)

    @Observer.observe("state_machines", after=True)
    def register_new_state_machines(self, model, prop_name, info):
        """ The method register self as observer newly added StateMachineModels after those were added to the list of
        state_machines hold by observed StateMachineMangerModel. The method register as observer of observable
        StateMachineMangerModel.state_machines."""
        if info['method_name'] == '__setitem__':
            self.observe_model(info['args'][1])
            logger.info(NotificationOverview(info))
        elif info['method_name'] == '__delitem__':
            pass
        else:
            logger.warning(NotificationOverview(info))

    @Observer.observe("state_machines", before=True)  # register as observer of observable StateMachineMangerModel.state_machines
    def relieve_state_machines(self, model, prop_name, info):
        """ The method relieves observed models before those get removed from the list of state_machines hold by
        observed StateMachineMangerModel. The method register as observer of observable
        StateMachineMangerModel.state_machines."""
        if info['method_name'] == '__setitem__':
            pass
        elif info['method_name'] == '__delitem__':
            self.relieve_model(self.state_machine_manager_model.state_machines[info['args'][0]])
            logger.info(NotificationOverview(info))
        else:
            logger.warning(NotificationOverview(info))

    @Observer.observe("state_machine", after=True)
    def all_after_notification(self, model, prop_name, info):
        """ The method logs all changes that notified recursively trough the hierarchies of the states after the change
        occurs in the rafcon.statemachine object. The method register as observer of observable
        StateMachineModel.state_machine of any observed StateMachineModel.
        :param model: StateMachineModel that is represents the state_machine which has been changed
        :param prop_name: Name of property that notifies -> here always 'state_machine'
        :param info: Dictionary that hold recursive notification information like models, property and method names
        :return:
        """
        logger.debug(NotificationOverview(info))