class Grid(object):
    def __init__(self, width, height, default_value=0):
        # Public variables
        self.width = width
        self.height = height

        # Private variables
        self.__grid = [[default_value for j in range(0, height)] for i in range(0, width)]

    def at(self, x, y):
        """Get value at (x, y)
        >>> g = Grid(16, 16)
        >>> g.at(3, 3)
        0
        >>> g.insert(3, 3, 2)
        >>> g.at(3, 3)
        2
        """
        return self.__grid[x][y]

    def insert(self, x, y, val):
        """Set value at x, y to val
        >>> g = Grid(16, 16)
        >>> g.insert(4, 4, 3)
        >>> g.at(4, 4)
        3
        >>> g.insert(17, 5, 1)
        Traceback (most recent call last):
         ...
        AssertionError: X coordinate larger than grid width
        """
        assert x < self.width, "X coordinate larger than grid width"
        assert y < self.height, "Y coordinate larger than grid height"
        self.__grid[x][y] = val