from constants import RIGHT, UP, LEFT, DOWN, getOpposite, getDelta
from grid import Grid
from tile import Tile

class Maze(object):
    """
    Manages location of walls etc.
    """
    width = 16
    height = 16
    
    def __init__(self):
        """
        >>> m = Maze()
        >>> m.hasWall(4, 4, RIGHT)
        False
        """
        self._grid = Grid(Maze.width, Maze.height, constructor=Tile)

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
        self._grid[x][y].setWall(direction, val)
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
        return self._grid[x][y].hasWall(direction)

    def getTileDelta(self, x, y, direction, spaces=1):
        """Get tile <spaces> away in direction from (x, y)
        >>> m = Maze()
        >>> m.getTileDelta(3, 4, RIGHT).hasWall(RIGHT)
        False
        >>> m.setTile(4, 4, RIGHT, True)
        >>> m.getTileDelta(3, 4, RIGHT).hasWall(RIGHT)
        True
        >>> m.getTileDelta(2, 4, RIGHT, 2).hasWall(RIGHT)
        True
        >>> m.getTileDelta(4, 2, UP, 2).hasWall(RIGHT)
        True
        >>> m.getTileDelta(6, 4, LEFT, 2).hasWall(RIGHT)
        True
        >>> m.getTileDelta(4, 2, DOWN, 2).hasWall(RIGHT)
        True
        """
        # TODO: Assert for out-of-bound errors
        assert True, ""

        delta = getDelta(direction)
        dx, dy = delta
        dx *= 2
        dy *= 2
        return self._grid[x + dx][y + dy]
