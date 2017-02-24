# Copyright

from gtkmvc import ModelMT
from gtkmvc import Observable

from rafcon.core.state_machine_manager import StateMachineManager
from rafcon.core.singleton import state_machine_execution_engine
from rafcon.core.execution.execution_engine import ExecutionEngine

from rafcon.utils.vividict import Vividict
from rafcon.utils import log

from rafcon.gui.models.state_machine import StateMachineModel


logger = log.get_logger(__name__)


class StateMachineExecutionEngineModel(ModelMT, Observable):

    execution_engine = None

    __sm_execution_engine_counter = 0
    __observables__ = ("execution_engine", )

    def __init__(self, execution_engine, meta=None):
        """Constructor"""
        ModelMT.__init__(self)  # pass columns as separate parameters
        Observable.__init__(self)
        self.register_observer(self)

        assert isinstance(execution_engine, ExecutionEngine)
        self.execution_engine = execution_engine

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

        # check if the sm_manager_model exists several times
        # this is obsolete as the monitoring plugin creates its own state machine execution engine and thus
        # also a new model

        # self.__class__.__sm_execution_engine_counter+= 1
        # if self.__class__.__sm_execution_engine_counter == 2:
        #     logger.error("Sm_manager_model exists several times!")
        #     import os
        #     os._exit(0)

    @property
    def core_element(self):
        return self.execution_engine

