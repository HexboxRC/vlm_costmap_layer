"""Validate oracle geometry independently of the layer implementation."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from publish_ground_truth import build_grid


class OracleTest(unittest.TestCase):
    def test_glass_is_lethal_grass_unpenalized_and_other_cells_unobserved(self):
        grid = build_grid(Path(__file__).resolve().parents[1] / 'worlds/arena.sdf')

        def cost(x, y):
            ix, iy = int((x + 6.25) / .05), int((y + 6.25) / .05)
            return grid.data[iy * grid.metadata.size_x + ix]

        self.assertEqual(cost(0, 4), 254)
        self.assertEqual(cost(.2, 4), 255)
        self.assertEqual(cost(0, 2.5), 255)
        self.assertEqual(cost(0, -4), 0)
        self.assertEqual(cost(2, -4), 255)
        self.assertEqual(cost(-4, 4), 255)
        self.assertEqual(len(grid.data), grid.metadata.size_x * grid.metadata.size_y)

    def test_intermediate_cost_is_preserved_and_invalid_cost_rejected(self):
        path = Path(__file__).resolve().parents[1] / 'worlds/arena.sdf'
        self.assertIn(120, build_grid(path, glass_cost=120).data)
        with self.assertRaises(ValueError):
            build_grid(path, glass_cost=255)
