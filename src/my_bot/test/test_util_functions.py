import unittest
from utils.bib import degrees_to_radians

class TestUtilFunctions(unittest.TestCase):

    def test_degrees_to_radians(self):
        self.assertAlmostEqual(degrees_to_radians(0), 0.0)
        self.assertAlmostEqual(degrees_to_radians(180), 3.141592653589793)
        self.assertAlmostEqual(degrees_to_radians(90), 1.5707963267948966)

if __name__ == '__main__':
    unittest.main()