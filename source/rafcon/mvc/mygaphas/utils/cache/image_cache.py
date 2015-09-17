
from cairo import ImageSurface
from cairo import FORMAT_ARGB32


class ImageCache(object):

    def __init__(self):
        """The ImageCache class can be used for caching ImageSurfaces

        The intentional use is for drawing methods. Instead of directly drawing to the cairo context, it is drawn on
        an ImageSurface. This allows the drawing to be buffered. The properties used for drawing are remembered. If
        the drawing routine is called again with the same parameters as before, the image is just copied.
        """
        self.__image = None
        self.__format = FORMAT_ARGB32
        self.__last_parameters = {}

    def get_cached_image(self, width, height, parameters={}):
        """Get ImageSurface object, if possible, cached

        The method checks whether the image was already rendered. This is done by comparing the passed size and
        parameters with those of the last image. If they are equal, the cached image is returned. Otherwise,
        a new ImageSurface with the specified dimensions is created and returned.
        :param width: The width of the image
        :param height: The height of the image
        :param parameters: The parameters used for the image
        :return: bool, ImageSurface -- The flag is True when the image is retrieved from the cache, otherwise False
        """
        if self.__compare_parameters(width, height, parameters):
            return self.__image
        else:
            self.__image = ImageSurface(self.__format, int(width), int(height))
            return None  # False, self.__image

    def set_cached_image(self, image, parameters={}):
        self.__image = image
        self.__last_parameters = parameters

    def __compare_parameters(self, width, height, parameters):
        """Compare parameters for equality

        Checks if a cached image is existing, the the dimensions agree and finally if the properties are equal. If
        so, True is returned, else False,
        :param width: The width of the image
        :param height: The height of the image
        :param parameters: The parameters used for the image
        :return: True if all parameters are equal, False else
        """
        # if not isinstance(self.__image, ImageSurface):
        if not self.__image:
            return False

        if self.__image.get_width() != int(width) or self.__image.get_height() != int(height):
            return False

        for key in parameters:
            if key not in self.__last_parameters or self.__last_parameters[key] != parameters[key]:
                return False

        return True