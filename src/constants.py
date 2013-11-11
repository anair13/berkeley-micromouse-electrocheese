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
