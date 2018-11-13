# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gtkmvc3.model_mt import ModelMT

from rafcon.core.execution.execution_engine import ExecutionEngine

from rafcon.utils.vividict import Vividict
from rafcon.utils import log

logger = log.get_logger(__name__)


class StateMachineExecutionEngineModel(ModelMT):

    execution_engine = None

    __sm_execution_engine_counter = 0
    __observables__ = ("execution_engine", )

    def __init__(self, execution_engine, meta=None):
        """Constructor"""
        ModelMT.__init__(self)  # pass columns as separate parameters
        self.register_observer(self)

        assert isinstance(execution_engine, ExecutionEngine)
        self.execution_engine = execution_engine

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

    @property
    def core_element(self):
        return self.execution_engine

