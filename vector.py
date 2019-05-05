import math
import logging

logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler)


class Vector(object):
    """Class to hold a directional vector for use with kRPC

    The vector is always a 3 element vector representing a direction in space
    """

    def __init__(self, *args):
        """Initialise the vector with the three dimensional values

        Parameters:
            `values`: iterable containing 3 or more numbers, only the first 3 values are
                    used one for each of the three spatial dimensions
        """
        values = []
        if args == None:
            values = [0, 0, 0]
        else:
            for a in args:
                try:
                    iter(a)
                    values = values + list(a)
                except TypeError:
                    values.append(a)
            while len(values) < 3:
                values.append(0)
        self.vector = list(values[:3])

    def __str__(self):
        """Display the vector as a list"""
        return self.vector.__str__()

    def __iter__(self):
        """Allow the vector to be iterated over to extract each element"""
        i = 0
        while i < 3:
            yield self.vector[i]
            i += 1

    def mag(self):
        """Return the magnitude of the passed in vector"""
        return math.sqrt(sum([x ** 2 for x in self.vector]))

    def dot(self, other):
        """Return the dot product of the self and other vector"""
        return sum([x * y for x, y in zip(self.vector, other.vector)])

    def angle(self, other):
        """Return the angle between two vectors"""
        dot = self.dot(other)
        if dot == 0:
            return 90
        else:
            cos_theta = dot / (self.mag() * other.mag())
            return math.degrees(math.acos(cos_theta))
