
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
        self.__width = None
        self.__height = None
        self.__zoom = None
        self.__format = FORMAT_ARGB32
        self.__last_parameters = {}

    def get_cached_image(self, width, height, zoom, parameters={}):
        """Get ImageSurface object, if possible, cached

        The method checks whether the image was already rendered. This is done by comparing the passed size and
        parameters with those of the last image. If they are equal, the cached image is returned. Otherwise,
        a new ImageSurface with the specified dimensions is created and returned.
        :param width: The width of the image
        :param height: The height of the image
        :param zoom: The current scale/zoom factor
        :param parameters: The parameters used for the image
        :return: bool, ImageSurface -- The flag is True when the image is retrieved from the cache, otherwise False
        """
        if self.__compare_parameters(width, height, zoom, parameters):
            return True, self.__image, self.__zoom

        image = ImageSurface(self.__format, int(width * zoom), int(height * zoom))
        self.__set_cached_image(image, width, height, zoom, parameters)
        return False, self.__image, zoom

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
        # if not isinstance(self.__image, ImageSurface):
        if not self.__image:
            return False

        if self.__width != width or self.__height != height:
            return False

        # TODO: Maybe implement zoom factor comparison
        print "equal size"
        for key in parameters:
            if key not in self.__last_parameters or self.__last_parameters[key] != parameters[key]:
                return False
        print "equal"
        return True