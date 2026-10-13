import unittest
from pantograph_clearance import evaluate


class CircularFitTests(unittest.TestCase):
    def test_source_offsets_and_clearances(self):
        rows = evaluate()["interfaces"]
        self.assertAlmostEqual(rows[0]["native_center_offset_mm"], .005876210293366559)
        self.assertAlmostEqual(rows[1]["native_center_offset_mm"], .007678724238958496)
        for r in rows:
            self.assertTrue(r["nominal_circles_fit"])
            self.assertTrue(r["polygonal_display_fit"])
            self.assertFalse(r["source_geometry_moved_or_scaled"])
            self.assertFalse(r["axial_stack_confirmed"])
            self.assertAlmostEqual(r["tie_radial_margin_mm"], .05)

    def test_nominal_zero_clearance_is_not_polygon_fit(self):
        for r in evaluate(8)["interfaces"]:
            self.assertTrue(r["nominal_circles_fit"])
            self.assertFalse(r["polygonal_display_fit"])
            self.assertAlmostEqual(r["tie_radial_margin_mm"], 0)
            self.assertGreater(r["outer_radial_margin_mm"], .09)

    def test_oversized_pin_and_required_clearance_fail(self):
        self.assertFalse(any(r["nominal_circles_fit"] for r in evaluate(8.01)["interfaces"]))
        self.assertFalse(any(r["nominal_circles_fit"] for r in evaluate(7.9, .051)["interfaces"]))

    def test_invalid_parameters(self):
        for bad in [True, None, "8", float('nan'), float('inf'), 0, -1, 21]:
            with self.assertRaises(ValueError):
                evaluate(bad)

    def test_existing_outer_axis_is_not_confused_with_tie_axis_study(self):
        for row in evaluate(8)["interfaces"]:
            self.assertTrue(row["nominal_circles_fit"])
            self.assertFalse(row["outer_axis_comparison"]["nominal_circles_fit"])
            self.assertAlmostEqual(row["outer_axis_comparison"]["maximum_common_pin_diameter_mm"],8-2*row["native_center_offset_mm"])
        for row in evaluate()["interfaces"]:
            self.assertTrue(row["outer_axis_comparison"]["nominal_circles_fit"])
            self.assertGreater(row["outer_axis_comparison"]["tie_radial_margin_mm"],.042)


if __name__ == '__main__':
    unittest.main()
