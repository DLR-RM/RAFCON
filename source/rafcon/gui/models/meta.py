
from gtkmvc import ModelMT, Signal

from rafcon.utils.vividict import Vividict


class MetaModel(ModelMT):

    def __init__(self, meta=None):
        ModelMT.__init__(self)

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()
        self.temp = Vividict()

        self.meta_signal = Signal()

    @staticmethod
    def _meta_data_editor_gaphas2opengl(vividict):
        """Convert meta data of editor from gaphas to OpenGL

        This should be implemented in the child classes if needed.

        :param Vividict vividict: meta data for the gaphas editor
        :return: meta data for the OpenGL editor
        :rtype: Vividict
        """
        return vividict

    @staticmethod
    def _meta_data_editor_opengl2gaphas(vividict):
        """Convert meta data of editor from OpenGL to gaphas

        This should be implemented in the child classes if needed.

        :param Vividict vividict: meta data for the OpenGL editor
        :return: meta data for the gaphas editor
        :rtype: Vividict
        """
        return vividict

    def get_meta_data_editor(self, for_gaphas=True):
        """Returns the editor for the specified editor

        This method should be used instead of accessing the meta data of an editor directly. It return sthe meta data
        of the editor available (with priority to the one specified by `for_gaphas`) and converts it if needed.

        :param bool for_gaphas: True (default) if the meta data is required for gaphas, False if for OpenGL
        :return: Meta data for the editor
        :rtype: Vividict
        """
        meta_gaphas = self.meta['gui']['editor_gaphas']
        meta_opengl = self.meta['gui']['editor_opengl']
        assert isinstance(meta_gaphas, Vividict) and isinstance(meta_opengl, Vividict)

        # Use meta data of editor with more keys (typically one of the editors has zero keys)
        from_gaphas = len(meta_gaphas) > len(meta_opengl) or (len(meta_gaphas) == len(meta_opengl) and for_gaphas)

        # Convert meta data if meta data target and origin differ
        if from_gaphas and not for_gaphas:
            self.meta['gui']['editor_opengl'] = self._meta_data_editor_gaphas2opengl(meta_gaphas)
        elif not from_gaphas and for_gaphas:
            self.meta['gui']['editor_gaphas'] = self._meta_data_editor_opengl2gaphas(meta_opengl)

        # only keep meta data for one editor
        del self.meta['gui']['editor_opengl' if for_gaphas else 'editor_gaphas']

        return self.meta['gui']['editor_gaphas'] if for_gaphas else self.meta['gui']['editor_opengl']

    def set_meta_data_editor(self, key, meta_data, from_gaphas=True):
        """Sets the meta data for a specific key of the desired editor

        :param str key: The meta data key, separated by dots if it is nested
        :param meta_data: The value to be set
        :param bool from_gaphas: If the data comes from a gaphas editor
        """
        meta_gui = self.meta['gui']
        meta_gui = meta_gui['editor_gaphas'] if from_gaphas else meta_gui['editor_opengl']

        key_path = key.split('.')
        for key in key_path:
            if key == key_path[-1]:
                meta_gui[key] = meta_data
            else:
                meta_gui = meta_gui[key]

        return self.get_meta_data_editor(for_gaphas=from_gaphas)
