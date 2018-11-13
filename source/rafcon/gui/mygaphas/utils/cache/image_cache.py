# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from builtins import object
from cairo import ImageSurface, FORMAT_ARGB32, Context, Error
from gi.repository import Gtk
from gi.repository import Gdk
# Gtk TODO
# from Gtk.gdk import CairoContext


from math import ceil, sqrt

from rafcon.gui.config import global_gui_config


MAX_ALLOWED_AREA = 5000. * 5000.


class ImageCache(object):
    def __init__(self, multiplicator=2):
        """The ImageCache class can be used for caching ImageSurfaces

        The intentional use is for drawing methods. Instead of directly drawing to the cairo context, it is drawn on
        an ImageSurface. This allows the drawing to be buffered. The properties used for drawing are remembered. If
        the drawing routine is called again with the same parameters as before, the image is just copied.

        :param float multiplicator: The zoom factor is multiplied with this value to prepare the cached image for
          higher zoom levels.
        """
        self.__image = None
        self.__width = None
        self.__height = None
        self.__zoom = None
        self.__format = FORMAT_ARGB32
        self.__last_parameters = {}
        self.__zoom_multiplicator = multiplicator
        self.__limiting_multiplicator = 1

    def get_cached_image(self, width, height, zoom, parameters=None, clear=False):
        """Get ImageSurface object, if possible, cached

        The method checks whether the image was already rendered. This is done by comparing the passed size and
        parameters with those of the last image. If they are equal, the cached image is returned. Otherwise,
        a new ImageSurface with the specified dimensions is created and returned.
        :param width: The width of the image
        :param height: The height of the image
        :param zoom: The current scale/zoom factor
        :param parameters: The parameters used for the image
        :param clear: If True, the cache is emptied, thus the image won't be retrieved from cache
        :returns: The flag is True when the image is retrieved from the cache, otherwise False; The cached image
          surface or a blank one with the desired size; The zoom parameter when the image was stored
        :rtype: bool, ImageSurface, float
        """
        global MAX_ALLOWED_AREA
        if not parameters:
            parameters = {}

        if self.__compare_parameters(width, height, zoom, parameters) and not clear:
            return True, self.__image, self.__zoom

        # Restrict image surface size to prevent excessive use of memory
        while True:
            try:
                self.__limiting_multiplicator = 1
                area = width * zoom * self.__zoom_multiplicator * height * zoom * self.__zoom_multiplicator
                if area > MAX_ALLOWED_AREA:
                    self.__limiting_multiplicator = sqrt(MAX_ALLOWED_AREA / area)

                image = ImageSurface(self.__format, int(ceil(width * zoom * self.multiplicator)),
                                     int(ceil(height * zoom * self.multiplicator)))
                break  # If we reach this point, the area was successfully allocated and we can break the loop
            except Error:
                MAX_ALLOWED_AREA *= 0.8

        self.__set_cached_image(image, width, height, zoom, parameters)
        return False, self.__image, zoom

    def copy_image_to_context(self, context, position, rotation=0, zoom=None):
        """Draw a cached image on the context

        :param context: The Cairo context to draw on
        :param position: The position od the image
        """
        if not zoom:
            zoom = self.__zoom
        zoom_multiplicator = zoom * self.multiplicator
        context.save()
        context.scale(1. / zoom_multiplicator, 1. / zoom_multiplicator)

        image_position = round(position[0] * zoom_multiplicator), round(position[1] * zoom_multiplicator)
        context.translate(*image_position)
        context.rotate(rotation)
        context.set_source_surface(self.__image, 0, 0)

        context.paint()
        context.restore()

    @property
    def multiplicator(self):
        return self.__zoom_multiplicator * self.__limiting_multiplicator

    def get_context_for_image(self, zoom):
        """Creates a temporary cairo context for the image surface

        :param zoom: The current scaling factor
        :return: Cairo context to draw on
        """
        cairo_context = Context(self.__image)
        cairo_context.scale(zoom * self.multiplicator, zoom * self.multiplicator)
        return cairo_context

    def __set_cached_image(self, image, width, height, zoom, parameters):
        self.__image = image
        self.__width = width
        self.__height = height
        self.__zoom = zoom
        self.__last_parameters = parameters

    def __compare_parameters(self, width, height, zoom, parameters):
        """Compare parameters for equality

        Checks if a cached image is existing, the the dimensions agree and finally if the properties are equal. If
        so, True is returned, else False,
        :param width: The width of the image
        :param height: The height of the image
        :param zoom: The current scale/zoom factor
        :param parameters: The parameters used for the image
        :return: True if all parameters are equal, False else
        """

        # Deactivated caching
        if not global_gui_config.get_config_value('ENABLE_CACHING', True):
            return False

        # Empty cache
        if not self.__image:
            return False

        # Changed image size
        if self.__width != width or self.__height != height:
            return False

        # Current zoom greater then prepared zoom
        if zoom > self.__zoom * self.__zoom_multiplicator:
            return False

        # Current zoom much smaller than prepared zoom, causes high memory usage and imperfect anti-aliasing
        if zoom < self.__zoom / self.__zoom_multiplicator:
            return False

        # Changed drawing parameter
        for key in parameters:
            try:
                if key not in self.__last_parameters or self.__last_parameters[key] != parameters[key]:
                    return False
            except (AttributeError, ValueError):
                # Some values cannot be compared and raise an exception on comparison (e.g. numpy.ndarray). In this
                # case, just return False and do not cache.
                try:
                    # Catch at least the ndarray-case, as this could occure relatively often
                    import numpy
                    if isinstance(self.__last_parameters[key], numpy.ndarray):
                        return numpy.array_equal(self.__last_parameters[key], parameters[key])
                except ImportError:
                    return False
                return False

        return True
