class Tile(object):
    """
    Tile object for Maze class
    """
    RIGHT, UP, LEFT, DOWN = 0, 1, 2, 3

    def __init__(self, walls=[False, False, False, False]):
        self.__walls = walls

    def setWall(self, direction, val):
        """Sets whether there is a wall on a certain tile
        WARNING: This does not update the neighboring tile.  Use setTile() as supplised by Maze.py
        >>> t = Tile()
        >>> t.setWall(Tile.LEFT, True)
        >>> t.hasWall(Tile.LEFT)
        True
        >>> t.hasWall(Tile.RIGHT)
        False
        """
        self.__walls[direction] = val

    def hasWall(self, direction):
        """Check if whether there is a wall in a certain direction
        >>> t = Tile()
        >>> t.hasWall(Tile.UP)
        False
        >>> t.setWall(Tile.UP, True)
        >>> t.hasWall(Tile.UP)
        True
        """
        return self.__walls[direction]
