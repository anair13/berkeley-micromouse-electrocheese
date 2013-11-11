from Constants import RIGHT, UP, LEFT, DOWN, getOpposite
from Grid import Grid
from Tile import Tile

class Maze(object):
    """
    Manages location of walls etc.
    """
    width = 16
    height = 16
    
    def __init__(self):
        self._grid = Grid(Maze.width, Maze.height, Tile())

    def setTile(self, x, y, direction, val):
        """Sets the wall state of a tile (x, y) in <direction>
        >>> m = Maze()
        >>> m.hasWall(4, 4, RIGHT)
        False
        >>> m.setTile(4, 4, RIGHT, True)
        >>> m.hasWall(4, 4, RIGHT)
        True
        >>> m.hasWall(5, 4, LEFT) # Also updates neighboring wall
        True
        """
        assert x < Maze.width, "X coordinate larger than grid width"
        assert y < Maze.height, "Y coordinate larger than grid height"
        self._grid.at(x, y).setWall(direction, val)
        self.getTileDelta(x, y, direction).setWall(getOpposite(direction), val)

    def hasWall(self, x, y, direction):
        """Returns whether there is a wall at (x, y) in <direction>
        >>> m = Maze()
        >>> m.hasWall(4, 4, RIGHT)
        False
        >>> m.setTile(4, 4, RIGHT, True)
        >>> m.hasWall(4, 4, RIGHT)
        True
        """
        return self._grid.at(x, y).hasWall(direction)

    def getTileDelta(self, x, y, direction, spaces=1):
        """Get tile <spaces> away in direction from (x, y)
        >>> t = "screw tests"
        """
        assert True, "TODO: Test for out of bound walls"
        return self._grid.at(x, y)
