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

    def generateMaze(self):
        # TODO
        """Generate maze using Prim's algorithm""" 
        pass

    def fillWithWalls(self):
        """Set every wall on every tile
        >>> m = Maze()
        >>> m.hasWall(4, 4, RIGHT)
        False
        >>> m.hasWall(6, 6, RIGHT)
        False
        >>> m.fillWithWalls()
        >>> m.hasWall(4, 4, RIGHT)
        True
        >>> m.hasWall(6, 6, RIGHT)
        True
        """
        # TODO: Make this efficient
        for i in range(0, Maze.width):
            for j in range(0, Maze.height):
                fill = lambda direction: self.setTile(i, j, direction, True)
                [fill(d) for d in [RIGHT, UP, LEFT, DOWN]]

    def printMaze(self):
        """Print entire maze

        I'm not writing a test for this"""
        self.fillWithWalls() # temporary
        for j in range(0, Maze.height):
            s = "_"
            for i in range(0, Maze.width):
                if self.hasWall(i, j, UP):
                    s += "_"
                else:
                    s += " "
                s += "_"
            print(s)
            print("| " * Maze.width + "|")
        print("_" * (Maze.width * 2 + 1))
            

    def setTile(self, x, y, direction, val):
        """Sets the wall state of a tile (x, y) in <direction>
        >>> m = Maze()
        >>> m.hasWall(4, 4, RIGHT)
        False
        >>> m.setTile(4, 4, RIGHT, True)
        >>> m.hasWall(4, 4, RIGHT)
        True
        >>> m.hasWall(6, 6, RIGHT)
        False
        >>> m.hasWall(5, 4, LEFT) # Also updates neighboring wall
        True
        """
        assert x < Maze.width, "X coordinate larger than grid width"
        assert y < Maze.height, "Y coordinate larger than grid height"
        self._grid[x][y].setWall(direction, val)
        try:
            self.getTileDelta(x, y, direction).setWall(getOpposite(direction), val)
        except AssertionError:
            pass

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
        >>> m.getTileDelta(4, 6, DOWN, 2).hasWall(RIGHT)
        True
        """
        delta = getDelta(direction)
        dx, dy = delta
        dx *= spaces
        dy *= spaces
        assert x + dx >= 0, "X coordinate less than 0"
        assert x + dx < self.width, "X coordinate greater than width"
        assert y + dy >= 0, "Y coordinate less than 0"
        assert y + dy < self.height, "Y coordinate greater than height"
        return self._grid[x + dx][y + dy]

# Temporary
m = Maze()
m.printMaze()
