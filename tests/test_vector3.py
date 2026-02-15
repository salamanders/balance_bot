import unittest
from balance_bot.utils import Vector3

class TestVector3(unittest.TestCase):
    def test_arithmetic(self):
        v1 = Vector3(1.0, 2.0, 3.0)
        v2 = Vector3(4.0, 5.0, 6.0)

        # Add
        v3 = v1 + v2
        self.assertEqual(v3.x, 5.0)
        self.assertEqual(v3.y, 7.0)
        self.assertEqual(v3.z, 9.0)

        # Sub
        v4 = v2 - v1
        self.assertEqual(v4.x, 3.0)
        self.assertEqual(v4.y, 3.0)
        self.assertEqual(v4.z, 3.0)

        # Mul (Scalar)
        v5 = v1 * 2.0
        self.assertEqual(v5.x, 2.0)
        self.assertEqual(v5.y, 4.0)
        self.assertEqual(v5.z, 6.0)

        # Div (Scalar)
        v6 = v2 / 2.0
        self.assertEqual(v6.x, 2.0)
        self.assertEqual(v6.y, 2.5)
        self.assertEqual(v6.z, 3.0)

        # Neg
        v7 = -v1
        self.assertEqual(v7.x, -1.0)
        self.assertEqual(v7.y, -2.0)
        self.assertEqual(v7.z, -3.0)
