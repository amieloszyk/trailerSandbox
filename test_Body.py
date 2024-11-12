import unittest

from CoordinateSystems import Position
from Body import isPointInOutline

class TestBody(unittest.TestCase):
    def test_isPointInOutline(self):
        outline = [
            Position(0, 0),
            Position(0, 5),
            Position(5, 5),
            Position(5, 0)
        ]

        # Test a point inside the outline
        point_inside = Position(2, 2)
        self.assertTrue(isPointInOutline(point_inside, outline))

        # Test a point outside the outline
        point_outside = Position(6, 6)
        self.assertFalse(isPointInOutline(point_outside, outline))

        # Test a point on the outline
        point_on_outline = Position(0, 0)
        self.assertFalse(isPointInOutline(point_on_outline, outline))

        # Test a point on the edge of the outline
        point_on_edge = Position(0, 5)
        self.assertFalse(isPointInOutline(point_on_edge, outline))

        # Test a point inside the outline with concave shape
        concave_outline = [
            Position(0, 0),
            Position(0, 5),
            Position(2, 3),
            Position(5, 5),
            Position(5, 0)
        ]
        point_inside_concave = Position(2, 2)
        self.assertTrue(isPointInOutline(point_inside_concave, concave_outline))

        # Test a point outside the outline with concave shape
        point_outside_concave = Position(4, 4)
        self.assertFalse(isPointInOutline(point_outside_concave, concave_outline))

if __name__ == '__main__':
    unittest.main()
