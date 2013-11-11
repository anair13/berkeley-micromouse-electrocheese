class _Col(object):
    """Please don't use this."""
    def __init__(self, data=[]):
        self.__data = data

    def __getitem__(self, key):
        return self.__data[key]

    def __setitem__(self, key, val):
        self.__data[key] = val

class Grid(object):
    def __init__(self, width, height, default_value=0, constructor=None):
        # Public variables
        self.width = width
        self.height = height

        if constructor != None:
            v = constructor()
        else:
            v = default_value

        # Private variables
        self.__grid = [_Col([v for j in range(0, height)]) for i in range(0, width)]

    def __getitem__(self, key):
        """Get column at x = key
        >>> g = Grid(16, 16)
        >>> g[3][3]
        0
        >>> g.insert(3, 3, 2)
        >>> g[3][3]
        2
        """
        return self.__grid[key]

    def insert(self, x, y, val):
        """Set value at x, y to val

        I would call this "set" if it wasn't reserved :(
        >>> g = Grid(16, 16)
        >>> g.insert(4, 4, 3)
        >>> g[4][4]
        3
        >>> g.insert(17, 5, 1)
        Traceback (most recent call last):
         ...
        AssertionError: X coordinate larger than grid width
        """
        assert x < self.width, "X coordinate larger than grid width"
        assert y < self.height, "Y coordinate larger than grid height"
        self.__grid[x][y] = val
