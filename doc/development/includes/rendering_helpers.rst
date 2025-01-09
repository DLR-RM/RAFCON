This function can be used when developing with cairo:

.. code-block:: python

    def pixel_to_cairo(self, pixel):
        """Helper function to convert pixels to cairo units

        The conversion is depending on the view. The critical parameter is the current zooming factor. The equation is:
        cairo units = pixels / zoom factor

        :param float pixel: Number of pixels to convert
        :return: Number of cairo units corresponding to given number of pixels
        :rtype: float
        """
        zoom = self.get_zoom_factor()
        return pixel / zoom