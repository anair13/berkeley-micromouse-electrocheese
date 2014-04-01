"""Static namespace only"""

RIGHT, UP, LEFT, DOWN = 0, 1, 2, 3

def getOpposite(direction):
    """Returns opposite direction
    >>> getOpposite(LEFT) == RIGHT
    True
    >>> getOpposite(UP) == DOWN
    True
    >>> getOpposite(DOWN) == LEFT
    False
    """
    opposites = [LEFT, DOWN, RIGHT, UP]
    return opposites[direction]

def getDelta(direction):
    """Returns single space offset in direction as tuple (dx, dy)
    >>> getDelta(LEFT)
    (-1, 0)
    >>> getDelta(UP)
    (0, 1)
    """
    deltas = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    return deltas[direction]
