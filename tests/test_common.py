import unittest

from common import mix


class MixTests(unittest.TestCase):
    def test_all_nine_key_combinations(self):
        cases = {
            frozenset(): (0.0, 0.0),
            frozenset({"up"}): (1.0, 1.0),
            frozenset({"down"}): (-1.0, -1.0),
            frozenset({"left"}): (-1.0, 1.0),
            frozenset({"right"}): (1.0, -1.0),
            frozenset({"up", "left"}): (0.0, 1.0),
            frozenset({"up", "right"}): (1.0, 0.0),
            frozenset({"down", "left"}): (-1.0, 0.0),
            frozenset({"down", "right"}): (0.0, -1.0),
        }

        for keys, expected in cases.items():
            with self.subTest(keys=keys):
                self.assertEqual(mix(keys), expected)


if __name__ == "__main__":
    unittest.main()
