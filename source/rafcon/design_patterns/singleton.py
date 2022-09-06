class Singleton:
    """
    The Singleton class that ensures the other classes to not be instantiated more than once.
    """

    def __init__(self, cls):
        self._cls = cls
        self._instance = None

    def instance(self, *args, **kwargs):
        """
        Returns the single instantiation of the class
        """

        if self._instance is None:
            self._instance = self._cls(*args, **kwargs)
        return self._instance

    def __call__(self):
        raise TypeError('The singleton must be accessed through instance()')

    def __instancecheck__(self, instance):
        return isinstance(instance, self._cls)
