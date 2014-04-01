from constants import RIGHT, UP, LEFT, DOWN

class Tile(object):
    """
    Tile object for Maze class
    """

    def __init__(self, walls=None):
        """
        >>> t = Tile()
        >>> t.hasWall(RIGHT)
        False
        """
        if walls == None:
            self.__walls = [False, False, False, False]
        else:
            self.__walls = walls

    def setWall(self, direction, val):
        """Sets whether there is a wall on a certain tile
        WARNING: This does not update the neighboring tile.  Use setTile() as supplised by Maze.py
        >>> t = Tile()
        >>> t.setWall(LEFT, True)
        >>> t.hasWall(LEFT)
        True
        >>> t.hasWall(RIGHT)
        False
        """
        self.__walls[direction] = val

    def hasWall(self, direction):
        """Check if whether there is a wall in a certain direction
        >>> t = Tile()
        >>> t.hasWall(UP)
        False
        >>> t.setWall(UP, True)
        >>> t.hasWall(UP)
        True
        """
        return self.__walls[direction]
