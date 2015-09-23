
from cairo import ImageSurface, FORMAT_ARGB32, Context
from gtk.gdk import CairoContext

from math import ceil

from rafcon.mvc.config import global_gui_config


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
        self.__multiplicator = multiplicator

    def get_cached_image(self, width, height, zoom, parameters={}, clear=False):
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
        if self.__compare_parameters(width, height, zoom, parameters) and not clear:
            return True, self.__image, self.__zoom

        max_allowed_size_length = 5000
        max_side_length = max(int(ceil(width * zoom * self.__multiplicator)),
                              int(ceil(height * zoom * self.__multiplicator)))
        if max_side_length > max_allowed_size_length:
            zoom /= max_side_length / max_allowed_size_length
        image = ImageSurface(self.__format, int(ceil(width * zoom * self.__multiplicator)),
                                            int(ceil(height * zoom * self.__multiplicator)))

        self.__set_cached_image(image, width, height, zoom, parameters)
        return False, self.__image, zoom

    def copy_image_to_context(self, context, position, zoom=None):
        """Draw a cached image on the context

        :param context: The Cairo context to draw on
        :param position: The position od the image
        """
        if not zoom:
            zoom = self.__zoom
        context.save()
        context.scale(1. / (zoom * self.__multiplicator), 1. / (zoom * self.__multiplicator))
        context.set_source_surface(self.__image, round(position[0] * zoom * self.__multiplicator),
                                                 round(position[1] * zoom * self.__multiplicator))
        context.paint()
        context.restore()

    @property
    def multiplicator(self):
        return self.__multiplicator

    def get_context_for_image(self, zoom):
        """Creates a temporary cairo context for the image surface

        :param zoom: The current scaling factor
        :return: Cairo context to draw on
        """
        cairo_context = Context(self.__image)
        c = CairoContext(cairo_context)
        c.scale(zoom * self.__multiplicator, zoom * self.__multiplicator)
        return c

    def __set_cached_image(self, image, width, height, zoom, parameters={}):
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
        if not global_gui_config.get_config_value('ENABLE_CACHING', True):
            return False

        if not self.__image:
            return False

        if self.__width != width or self.__height != height:
            return False

        if zoom > self.__zoom * self.__multiplicator:
            return False

        for key in parameters:
            if key not in self.__last_parameters or self.__last_parameters[key] != parameters[key]:
                return False

        return True