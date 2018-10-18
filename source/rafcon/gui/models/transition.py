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
from rafcon.core.state_elements.transition import Transition
from rafcon.utils import log
logger = log.get_logger(__name__)


def mirror_waypoints(vividict):
    if isinstance(vividict['waypoints'], list):
        old_waypoint_list = vividict['waypoints']
        new_waypoint_list = []
        for wp in old_waypoint_list:
            new_waypoint_list.append((wp[0], -wp[1]))
            vividict['waypoints'] = new_waypoint_list
    else:
        del vividict['waypoints']
    return vividict


class TransitionModel(StateElementModel):
    """This model class manages a Transition

    :param rafcon.core.transition.Transition transition: The transition to be wrapped
    :param rafcon.gui.models.abstract_state.AbstractStateModel parent: The state model of the state element
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state element model
     """

    transition = None

    __observables__ = ("transition",)

    def __init__(self, transition, parent, meta=None):
        """Constructor
        """
        super(TransitionModel, self).__init__(parent, meta)

        assert isinstance(transition, Transition)
        self.transition = transition

    def __str__(self):
        return "Model of Transition: {0}".format(self.transition)

    def __copy__(self):
        transition = copy(self.transition)
        transition_m = self.__class__(transition, parent=None, meta=None)
        transition_m.meta = deepcopy(self.meta)
        return transition_m

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @property
    def core_element(self):
        return self.transition

    @ModelMT.observe("transition", before=True, after=True)
    def model_changed(self, model, prop_name, info):
        super(TransitionModel, self).model_changed(model, prop_name, info)

    def prepare_destruction(self):
        super(TransitionModel, self).prepare_destruction()
        self.transition = None

    def _meta_data_editor_gaphas2opengl(self, vividict):
        return mirror_waypoints(vividict)

    def _meta_data_editor_opengl2gaphas(self, vividict):
        return mirror_waypoints(vividict)
