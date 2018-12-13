import rafcon.gui.singleton
from gtkmvc3.observer import Observer
from rafcon.gui.utils.notification_overview import NotificationOverview

from rafcon.utils import log
logger = log.get_logger(__name__)


class RootStateModificationObserver(Observer):

    def __init__(self):
        self.logger = log.get_logger(type(self).__name__)
        self.logger.info("Initiate gtkmvc3 observer")
        # register self as observer of rafcon.gui.singleton.state_machine_manager_model
        Observer.__init__(self, rafcon.gui.singleton.state_machine_manager_model)
        self.state_machine_manager_model = rafcon.gui.singleton.state_machine_manager_model

        # register self as observer of already existing StateMachineModels in state_machines list
        for sm_id, sm_m in list(self.state_machine_manager_model.state_machines.items()):
            self.observe_model(sm_m)

    @Observer.observe("state_machines", after=True)
    def register_new_state_machines(self, model, prop_name, info):
        """ The method register self as observer newly added StateMachineModels after those were added to the list of
        state_machines hold by observed StateMachineMangerModel. The method register as observer of observable
        StateMachineMangerModel.state_machines."""
        if info['method_name'] == '__setitem__':
            self.observe_model(info['args'][1])
            self.logger.info(NotificationOverview(info))
        elif info['method_name'] == '__delitem__':
            pass
        else:
            self.logger.warning(NotificationOverview(info))

    @Observer.observe("state_machines", before=True)  # register as observer of observable StateMachineMangerModel.state_machines
    def relieve_state_machines(self, model, prop_name, info):
        """ The method relieves observed models before those get removed from the list of state_machines hold by
        observed StateMachineMangerModel. The method register as observer of observable
        StateMachineMangerModel.state_machines."""
        if info['method_name'] == '__setitem__':
            pass
        elif info['method_name'] == '__delitem__':
            self.relieve_model(self.state_machine_manager_model.state_machines[info['args'][0]])
            self.logger.info(NotificationOverview(info))
        else:
            self.logger.warning(NotificationOverview(info))

    @Observer.observe("state_machine", after=True)
    def all_after_notification(self, model, prop_name, info):
        """ The method logs all changes that notified recursively trough the hierarchies of the states after the change
        occurs in the rafcon.core object. The method register as observer of observable
        StateMachineModel.state_machine of any observed StateMachineModel.
        :param model: StateMachineModel that is represents the state_machine which has been changed
        :param prop_name: Name of property that notifies -> here always 'state_machine'
        :param info: Dictionary that hold recursive notification information like models, property and method names
        :return:
        """
        self.logger.debug(NotificationOverview(info))


class MetaSignalModificationObserver(Observer):

    def __init__(self):
        self.logger = log.get_logger(type(self).__name__)
        self.logger.info("Initiate gtkmvc3 meta-signal observer")
        # register self as observer of rafcon.gui.singleton.state_machine_manager_model
        Observer.__init__(self, rafcon.gui.singleton.state_machine_manager_model)
        self.state_machine_manager_model = rafcon.gui.singleton.state_machine_manager_model

        # register self as observer of already existing StateMachineModels in state_machines list
        for sm_id, sm_m in list(self.state_machine_manager_model.state_machines.items()):
            self.observe_model(sm_m)
            self.observe_model(sm_m.root_state)

    @Observer.observe("state_machines", after=True)
    def register_new_state_machines(self, model, prop_name, info):
        """ The method register self as observer newly added StateMachineModels after those were added to the list of
        state_machines hold by observed StateMachineMangerModel. The method register as observer of observable
        StateMachineMangerModel.state_machines."""
        if info['method_name'] == '__setitem__':
            self.observe_model(info['args'][1])
            self.observe_model(info['args'][1].root_state)
            # self.logger.info(NotificationOverview(info))
        elif info['method_name'] == '__delitem__':
            pass
        else:
            self.logger.warning(NotificationOverview(info))

    @Observer.observe("state_machines", before=True)  # register as observer of observable StateMachineMangerModel.state_machines
    def relieve_state_machines(self, model, prop_name, info):
        """ The method relieves observed models before those get removed from the list of state_machines hold by
        observed StateMachineMangerModel. The method register as observer of observable
        StateMachineMangerModel.state_machines."""
        if info['method_name'] == '__setitem__':
            pass
        elif info['method_name'] == '__delitem__':
            self.relieve_model(self.state_machine_manager_model.state_machines[info['args'][0]])
            if self.state_machine_manager_model.state_machines[info['args'][0]].root_state:
                self.relieve_model(self.state_machine_manager_model.state_machines[info['args'][0]].root_state)
                # otherwise relieved by root_state assign notification
            # self.logger.info(NotificationOverview(info))
        else:
            self.logger.warning(NotificationOverview(info))

    @Observer.observe('root_state', assign=True)
    def observe_root_state_assignments(self, model, prop_name, info):
        """ The method relieves observed root_state models and observes newly assigned root_state models.
        """
        if info['old']:
            self.relieve_model(info['old'])
        if info['new']:
            self.observe_model(info['new'])
            self.logger.info("Exchange observed old root_state model with newly assigned one. sm_id: {}"
                             "".format(info['new'].state.parent.state_machine_id))

    @Observer.observe("meta_signal", signal=True)
    def observe_meta_signal_changes(self, changed_model, prop_name, info):
        """" The method prints the structure of all meta_signal-notifications as log-messages.
        """
        self.logger.info(NotificationOverview(info))
