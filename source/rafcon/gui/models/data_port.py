# Copyright (C) 2015-2017 DLR
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

from copy import copy, deepcopy
from gtkmvc3.model_mt import ModelMT

from rafcon.gui.models.state_element import StateElementModel
from rafcon.core.state_elements.data_port import DataPort, InputDataPort
from rafcon.utils import log

logger = log.get_logger(__name__)


class DataPortModel(StateElementModel):
    """This model class manages a DataPort

    :param rafcon.core.data_port.DataPort data_port: The input/output data port to be wrapped
    :param rafcon.gui.models.abstract_state.AbstractStateModel parent: The state model of the state element
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state element model
     """

    data_port = None

    __observables__ = ("data_port",)

    def __init__(self, data_port, parent, meta=None):
        """Constructor
        """
        super(DataPortModel, self).__init__(parent, meta)

        assert isinstance(data_port, DataPort)
        self.data_port = data_port

    def __str__(self):
        return "Model of DataPort: {0}".format(self.data_port)

    def __copy__(self):
        data_port = copy(self.data_port)
        data_port_m = self.__class__(data_port, parent=None, meta=None)
        data_port_m.meta = deepcopy(self.meta)
        return data_port_m

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @property
    def core_element(self):
        return self.data_port

    @ModelMT.observe("data_port", before=True, after=True)
    def model_changed(self, model, prop_name, info):
        super(DataPortModel, self).model_changed(model, prop_name, info)

    def prepare_destruction(self):
        super(DataPortModel, self).prepare_destruction()
        self.data_port = None

    def _meta_data_editor_gaphas2opengl(self, vividict):
        if 'rel_pos' in vividict:
            rel_pos = vividict['rel_pos']
            del vividict['rel_pos']
            vividict['inner_rel_pos'] = (rel_pos[0], -rel_pos[1])
        return vividict

    def _meta_data_editor_opengl2gaphas(self, vividict):
        from rafcon.gui.helpers.meta_data import contains_geometric_info
        if self.parent:
            rel_pos = vividict['inner_rel_pos']
            state_size = self.parent.get_meta_data_editor()['size']
            if contains_geometric_info(rel_pos) and contains_geometric_info(state_size):
                if isinstance(self.data_port, InputDataPort):
                    vividict['rel_pos'] = (0, -rel_pos[1])
                else:  # OutputDataPort
                    vividict['rel_pos'] = (state_size[1], -rel_pos[1])
            else:
                del vividict['inner_rel_pos']
        return vividict
