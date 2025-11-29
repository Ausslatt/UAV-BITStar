# state.py
# State representation for BIT* (2D)

import math

class State(object):
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

    def as_tuple(self):
        """Return (x, y) for printing or logging."""
        return (self.x, self.y)



