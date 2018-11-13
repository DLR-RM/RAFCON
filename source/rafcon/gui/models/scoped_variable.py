# Copyright (C) 2015-2017 DLR
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
from rafcon.core.state_elements.scope import ScopedVariable
from rafcon.utils import log

logger = log.get_logger(__name__)


class ScopedVariableModel(StateElementModel):
    """This model class manages a ScopedVariable

    :param rafcon.core.scoped_variable.ScopedVariable scoped_variable: The scoped variable to be wrapped
    :param rafcon.gui.models.abstract_state.AbstractStateModel parent: The state model of the state element
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state element model
     """

    scoped_variable = None

    __observables__ = ("scoped_variable",)

    def __init__(self, scoped_variable, parent, meta=None):
        """Constructor
        """
        super(ScopedVariableModel, self).__init__(parent, meta)

        assert isinstance(scoped_variable, ScopedVariable)
        self.scoped_variable = scoped_variable

    def __str__(self):
        return "Model of ScopedVariable: {0}".format(self.scoped_variable)

    def __copy__(self):
        scoped_variable = copy(self.scoped_variable)
        scoped_variable_m = self.__class__(scoped_variable, parent=None, meta=None)
        scoped_variable_m.meta = deepcopy(self.meta)
        return scoped_variable_m

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @property
    def core_element(self):
        return self.scoped_variable

    @ModelMT.observe("scoped_variable", before=True, after=True)
    def model_changed(self, model, prop_name, info):
        super(ScopedVariableModel, self).model_changed(model, prop_name, info)

    def prepare_destruction(self):
        super(ScopedVariableModel, self).prepare_destruction()
        self.scoped_variable = None

    def _meta_data_editor_gaphas2opengl(self, vividict):
        if 'rel_pos' in vividict:
            rel_pos = vividict['rel_pos']
            del vividict['rel_pos']
            vividict['inner_rel_pos'] = (rel_pos[0], -rel_pos[1])
        return vividict

    def _meta_data_editor_opengl2gaphas(self, vividict):
        from rafcon.gui.helpers.meta_data import contains_geometric_info
        rel_pos = vividict['inner_rel_pos']
        if contains_geometric_info(rel_pos):
            vividict['rel_pos'] = (rel_pos[0], 0)
        else:
            del vividict['inner_rel_pos']
        return vividict
