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

import hashlib
from gtkmvc3.model_mt import ModelMT
from gtkmvc3.observable import Signal

from rafcon.utils.hashable import Hashable
from rafcon.utils.vividict import Vividict


class MetaModel(ModelMT):

    _parent = None
    meta = None

    def __init__(self, meta=None):
        ModelMT.__init__(self)

        if isinstance(meta, dict):
            self.meta = Vividict(meta)
        else:
            self.meta = Vividict()
        self.temp = Vividict()

        self.meta_signal = Signal()

    def _meta_data_editor_gaphas2opengl(self, vividict):
        """Convert meta data of editor from gaphas to OpenGL

        This should be implemented in the child classes if needed.

        :param Vividict vividict: meta data for the gaphas editor
        :return: meta data for the OpenGL editor
        :rtype: Vividict
        """
        return vividict

    def _meta_data_editor_opengl2gaphas(self, vividict):
        """Convert meta data of editor from OpenGL to gaphas

        This should be implemented in the child classes if needed.

        :param Vividict vividict: meta data for the OpenGL editor
        :return: meta data for the gaphas editor
        :rtype: Vividict
        """
        return vividict

    def get_meta_data_editor(self, for_gaphas=True):
        """Returns the editor for the specified editor

        This method should be used instead of accessing the meta data of an editor directly. It return the meta data
        of the editor available (with priority to the one specified by `for_gaphas`) and converts it if needed.

        :param bool for_gaphas: True (default) if the meta data is required for gaphas, False if for OpenGL
        :return: Meta data for the editor
        :rtype: Vividict
        """
        meta_gaphas = self.meta['gui']['editor_gaphas']
        meta_opengl = self.meta['gui']['editor_opengl']
        assert isinstance(meta_gaphas, Vividict) and isinstance(meta_opengl, Vividict)

        # Use meta data of editor with more keys (typically one of the editors has zero keys)
        # TODO check if the magic length condition in the next line can be improved (consistent behavior getter/setter?)
        parental_conversion_from_opengl = self._parent and self._parent().temp['conversion_from_opengl']
        from_gaphas = len(meta_gaphas) > len(meta_opengl) or (len(meta_gaphas) == len(meta_opengl) and for_gaphas and
                                                              not parental_conversion_from_opengl)
        # Convert meta data if meta data target and origin differ
        if from_gaphas and not for_gaphas:
            self.meta['gui']['editor_opengl'] = self._meta_data_editor_gaphas2opengl(meta_gaphas)
        elif not from_gaphas and for_gaphas:
            self.meta['gui']['editor_gaphas'] = self._meta_data_editor_opengl2gaphas(meta_opengl)

        # only keep meta data for one editor
        del self.meta['gui']['editor_opengl' if for_gaphas else 'editor_gaphas']
        return self.meta['gui']['editor_gaphas'] if for_gaphas else self.meta['gui']['editor_opengl']

    def do_convert_meta_data_if_no_data(self, for_gaphas):
        if not self.meta['gui']['editor_gaphas'] and for_gaphas:
            self.meta['gui']['editor_gaphas'] = self._meta_data_editor_opengl2gaphas(self.meta['gui']['editor_opengl'])
        elif not self.meta['gui']['editor_opengl'] and not for_gaphas:
            self.meta['gui']['editor_opengl'] = self._meta_data_editor_gaphas2opengl(self.meta['gui']['editor_gaphas'])

    def set_meta_data_editor(self, key, meta_data, from_gaphas=True):
        """Sets the meta data for a specific key of the desired editor

        :param str key: The meta data key, separated by dots if it is nested
        :param meta_data: The value to be set
        :param bool from_gaphas: If the data comes from a gaphas editor
        """
        self.do_convert_meta_data_if_no_data(from_gaphas)
        meta_gui = self.meta['gui']
        meta_gui = meta_gui['editor_gaphas'] if from_gaphas else meta_gui['editor_opengl']

        key_path = key.split('.')
        for key in key_path:
            if isinstance(meta_gui, list):
                meta_gui[int(key)] = meta_data
                break
            if key == key_path[-1]:
                meta_gui[key] = meta_data
            else:
                meta_gui = meta_gui[key]

        return self.get_meta_data_editor(for_gaphas=from_gaphas)

    def meta_data_hash(self, obj_hash=None):
        """Creates a hash with the meta data of the model

        :param obj_hash: The hash object (see Python hashlib)
        :return: The updated hash object
        """
        if obj_hash is None:
            obj_hash = hashlib.sha256()
        self.update_meta_data_hash(obj_hash)
        return obj_hash

    def update_meta_data_hash(self, obj_hash):
        """Should be implemented by derived classes to update the hash with their meta data fields

        :param obj_hash: The hash object (see Python hashlib)
        """
        Hashable.update_hash_from_dict(obj_hash, self.meta)

    def prepare_destruction(self):
        """Prepares the model for destruction

        """
        self._Observer__PROP_TO_METHS.clear()
        self._Observer__METH_TO_PROPS.clear()
        self._Observer__PAT_TO_METHS.clear()
        self._Observer__METH_TO_PAT.clear()
        self._Observer__PAT_METH_TO_KWARGS.clear()
