import math
import unittest

from vector import Vector


class VectorTestSuite(unittest.TestCase):
    def test_constructors(self):

        # Literal values
        self.assertEquals(Vector(1).__str__(), "[1, 0, 0]")
        self.assertEquals(Vector(1, 2).__str__(), "[1, 2, 0]")
        self.assertEquals(Vector(1, 2, 3).__str__(), "[1, 2, 3]")
        self.assertEquals(Vector(1, 2, 3, 4).__str__(), "[1, 2, 3]")
        # Tuples
        self.assertEquals(Vector((1,)).__str__(), "[1, 0, 0]")
        self.assertEquals(Vector((1, 2)).__str__(), "[1, 2, 0]")
        self.assertEquals(Vector((1, 2, 3)).__str__(), "[1, 2, 3]")
        self.assertEquals(Vector((1, 2, 3, 4)).__str__(), "[1, 2, 3]")
        # Lists
        self.assertEquals(Vector([1]).__str__(), "[1, 0, 0]")
        self.assertEquals(Vector([1, 2]).__str__(), "[1, 2, 0]")
        self.assertEquals(Vector([1, 2, 3]).__str__(), "[1, 2, 3]")
        self.assertEquals(Vector([1, 2, 3, 4]).__str__(), "[1, 2, 3]")
        # Vectors
        self.assertEquals(Vector(Vector(1, 2, 3)).__str__(), "[1, 2, 3]")

    def test_mag(self):
        self.assertEqual(Vector(1).mag(), 1)
        self.assertEqual(Vector(1).mag(), 1)
        self.assertEqual(Vector(2).mag(), 2)
        self.assertEqual(Vector(3).mag(), 3)
        self.assertEqual(Vector(1, 2).mag(), math.sqrt(5))
        self.assertEqual(Vector(1, 3).mag(), math.sqrt(10))
        self.assertEqual(Vector(2, 3).mag(), math.sqrt(13))
        self.assertEqual(Vector(1, 2, 3).mag(), math.sqrt(14))
        self.assertEqual(Vector(-1, 2, 3).mag(), math.sqrt(14))
        self.assertEqual(Vector(1, -2, 3).mag(), math.sqrt(14))
        self.assertEqual(Vector(1, 2, -3).mag(), math.sqrt(14))

    def test_dot(self):
        self.assertEqual(Vector((1, 2, 3)).dot(Vector((1, 2, 3))), 14)
        self.assertEqual(Vector((1, 2, 3)).dot(Vector((3, 2, 1))), 10)
        self.assertEqual(Vector((1, 2, 3)).dot(Vector((-1, 2, 3))), 12)
        self.assertEqual(Vector((1, 2, 3)).dot(Vector((-3, 2, 1))), 4)
        self.assertEqual(Vector((1, 2, 3)).dot(Vector((1, -2, 3))), 6)
        self.assertEqual(Vector((1, 2, 3)).dot(Vector((3, -2, 1))), 2)
        self.assertEqual(Vector((1, 2, 3)).dot(Vector((1, 2, -3))), -4)
        self.assertEqual(Vector((1, 2, 3)).dot(Vector((3, 2, -1))), 4)
        self.assertEqual(Vector((1, 2, 3)).dot(Vector((1, -2, -3))), -12)
        self.assertEqual(Vector((1, 2, 3)).dot(Vector((-3, -2, 1))), -4)

    def test_angle(self):
        # Test using two dimensional vectors
        test_data = [
            Vector((1, 0, 0)),
            Vector((1, 1, 0)),
            Vector((0, 1, 0)),
            Vector((-1, 1, 0)),
            Vector((-1, 0, 0)),
            Vector((-1, -1, 0)),
            Vector((0, -1, 0)),
            Vector((1, -1, 0)),
            # Repeat to make wrap arounds easy to test
            Vector((1, 0, 0)),
            Vector((1, 1, 0)),
            Vector((0, 1, 0)),
            Vector((-1, 1, 0)),
            Vector((-1, 0, 0)),
            Vector((-1, -1, 0)),
            Vector((0, -1, 0)),
            Vector((1, -1, 0)),
        ]
        full_circle = range(0, 8)

        def test_it(step, expected):
            for i in full_circle:
                self.assertAlmostEqual(
                    test_data[i].angle(test_data[i + step]), expected, 5
                )
                self.assertAlmostEqual(
                    test_data[i + step].angle(test_data[i]), expected, 5
                )

        # Check angle between vector and itself is 0
        for v in test_data:
            self.assertAlmostEqual(v.angle(Vector(v)), 0, 4)

        # Calculates the shortest angle (unsigned)
        test_it(1, 45)
        test_it(2, 90)
        test_it(3, 135)
        test_it(4, 180)
        test_it(5, 135)
        test_it(6, 90)
        test_it(7, 45)
        test_it(8, 0)


if __name__ == "__main__":
    unittest.main()
