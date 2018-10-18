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
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gtkmvc3.model_mt import ModelMT

from rafcon.core.singleton import global_variable_manager

from rafcon.utils.vividict import Vividict
from rafcon.utils import log
logger = log.get_logger(__name__)


class GlobalVariableManagerModel(ModelMT):
    """This Model class manages the global variable manager."""

    global_variable_manager = global_variable_manager

    __observables__ = ("global_variable_manager",)

    def __init__(self, meta=None):
        """Constructor"""
        ModelMT.__init__(self)  # pass columns as separate parameters

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()
