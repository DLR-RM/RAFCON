# Copyright (C) 2014-2017 DLR
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

from copy import copy, deepcopy
from gtkmvc3.model_mt import ModelMT

from rafcon.gui.models.state_element import StateElementModel
from rafcon.core.state_elements.data_flow import DataFlow
from rafcon.utils import log
logger = log.get_logger(__name__)


class DataFlowModel(StateElementModel):
    """This model class manages a DataFlow

    :param rafcon.core.data_flow.DataFlow data_flow: The data flow to be wrapped
    :param rafcon.gui.models.abstract_state.AbstractStateModel parent: The state model of the state element
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state element model
     """

    data_flow = None

    __observables__ = ("data_flow",)

    def __init__(self, data_flow, parent, meta=None):
        """Constructor
        """
        super(DataFlowModel, self).__init__(parent, meta)

        assert isinstance(data_flow, DataFlow)
        self.data_flow = data_flow

    def __str__(self):
        return "Model of DataFlow: {0}".format(self.data_flow)

    def __copy__(self):
        data_flow = copy(self.data_flow)
        data_flow_m = self.__class__(data_flow, parent=None, meta=None)
        data_flow_m.meta = deepcopy(self.meta)
        return data_flow_m

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @property
    def core_element(self):
        return self.data_flow

    @ModelMT.observe("data_flow", before=True, after=True)
    def model_changed(self, model, prop_name, info):
        super(DataFlowModel, self).model_changed(model, prop_name, info)

    def prepare_destruction(self):
        super(DataFlowModel, self).prepare_destruction()
        self.data_flow = None
