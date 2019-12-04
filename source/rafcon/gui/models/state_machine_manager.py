# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>
import os

from gtkmvc3.model_mt import ModelMT

from rafcon.core.state_machine_manager import StateMachineManager

from rafcon.gui.models.state_machine import StateMachineModel

from rafcon.utils.vividict import Vividict
from rafcon.utils import log
from rafcon.utils.timer import measure_time

logger = log.get_logger(__name__)


class StateMachineManagerModel(ModelMT):
    """This model class manages a StateMachineManager

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state machine
    manager).
    Additional to the data of the StateMachineManager its model and the models of state machines hold by those
    these model stores and made observable the selected state machine of the view which have not to be the same
    as the active running one.

    :param StateMachineManager state_machine_manager: The state machine manager to be managed
    """

    # TODO static variable in StateMachineManagerModel
    __sm_manager_creation_counter = 0
    state_machine_manager = None
    selected_state_machine_id = None
    state_machines = {}

    __observables__ = ("state_machine_manager", "selected_state_machine_id", "state_machines")

    def __init__(self, state_machine_manager, meta=None):
        """Constructor"""
        ModelMT.__init__(self)  # pass columns as separate parameters
        self.register_observer(self)

        assert isinstance(state_machine_manager, StateMachineManager)

        self.state_machine_manager = state_machine_manager
        self.state_machines = {}
        for sm_id, sm in state_machine_manager.state_machines.items():
            self.state_machines[sm_id] = StateMachineModel(sm)

        self._selected_state_machine_id = None
        if len(self.state_machines) > 0:
            self.selected_state_machine_id = list(self.state_machines.keys())[0]

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

        # check if the sm_manager_model exists several times
        self.__class__.__sm_manager_creation_counter += 1
        if self.__class__.__sm_manager_creation_counter == 2:
            logger.error("Sm_manager_model exists several times!")
            os._exit(0)

    @property
    def core_element(self):
        return self.state_machine_manager

    @ModelMT.observe("state_machine_manager", after=True)
    # @measure_time
    def model_changed(self, model, prop_name, info):
        if isinstance(info['result'], Exception):
            from rafcon.gui.utils.notification_overview import NotificationOverview
            logger.exception("An '{0}' exception was raised in the core. "
                             "Details about the origin:\n{1}"
                             "".format(type(info['result']).__name__, NotificationOverview(info)))
            return

        if info["method_name"] == "add_state_machine":
            logger.debug("Add new state machine model ... ")
            sm_id = info['result']
            if sm_id in self.state_machine_manager.state_machines and sm_id not in self.state_machines:
                logger.debug("Create new state machine model for state machine with id %s", sm_id)
                sm = self.state_machine_manager.state_machines[sm_id]
                with sm.modification_lock():
                    self.state_machines[sm_id] = StateMachineModel(sm)
                    from rafcon.gui.models.abstract_state import AbstractStateModel
                    logger.verbose("Number of created state models {}".format(AbstractStateModel.state_counter))
                    self.selected_state_machine_id = sm_id
            else:
                logger.error("Model of state machine {0} is supposed to not exist but the state machine object should."
                             "".format(sm_id))

        elif info["method_name"] == "remove_state_machine":
            sm_id_to_delete = info['result'].state_machine_id
            if sm_id_to_delete is not None:
                logger.debug("Delete state machine model for state machine with id %s", sm_id_to_delete)
                if self.selected_state_machine_id == sm_id_to_delete:
                    self.selected_state_machine_id = None
                sm_m = self.state_machines[sm_id_to_delete]
                sm_m.prepare_destruction()
                del self.state_machines[sm_id_to_delete]
                sm_m.destroy()

    def get_state_machine_model(self, state_m):
        """ Get respective state machine model for handed state model

        :param state_m: State model for which the state machine model should be found
        :return: state machine model
        :rtype: rafcon.gui.models.state_machine.StateMachineModel
        """
        return self.state_machines[state_m.state.get_state_machine().state_machine_id]

    def get_selected_state_machine_model(self):
        """ Get selected state machine model

        :return: state machine model
        :rtype: rafcon.gui.models.state_machine.StateMachineModel
        """
        if self.selected_state_machine_id is None:
            return None

        return self.state_machines[self.selected_state_machine_id]

    @property
    def selected_state_machine_id(self):
        """Property for the _selected_state_machine_id field
        :rtype: int
        """
        return self._selected_state_machine_id

    @selected_state_machine_id.setter
    def selected_state_machine_id(self, selected_state_machine_id):
        if selected_state_machine_id is not None:
            if not isinstance(selected_state_machine_id, int):
                raise TypeError("selected_state_machine_id must be of type int")
        self._selected_state_machine_id = selected_state_machine_id
