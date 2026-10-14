import math
from pathlib import Path
import unittest

from amr_edge_lab.core import Grid, Mission, State, astar, project_depth, raycast, warehouse
from amr_edge_lab.offline import run


class PlannerTests(unittest.TestCase):
    def test_known_shortest_path(self):
        grid = Grid(5, 5, 1.0, frozenset())
        route = astar(grid, (0, 0), (4, 4))
        self.assertEqual(len(route), 5)

    def test_no_corner_cutting(self):
        grid = Grid(2, 2, 1.0, frozenset({(0, 1), (1, 0)}))
        self.assertEqual(astar(grid, (0, 0), (1, 1)), [])

    def test_obstacle_and_bounds(self):
        grid = Grid(4, 4, 1, frozenset({(2, 2)}))
        self.assertEqual(astar(grid, (0, 0), (2, 2)), [])
        with self.assertRaises(ValueError):
            astar(grid, (-1, 0), (3, 3))

    def test_inflation_and_route_validity(self):
        grid = warehouse().inflated(0.3)
        route = astar(grid, (10, 10), (68, 48))
        self.assertTrue(route)
        self.assertTrue(all(c not in grid.occupied for c in route))
        self.assertIn((1, 2), Grid(5, 5, 1, frozenset({(2, 2)})).inflated(1).occupied)

    def test_raycast(self):
        self.assertAlmostEqual(raycast(Grid(10, 10, 1, frozenset({(5, 2)})), 2.5, 2.5, 0), 2.5)


class DepthTests(unittest.TestCase):
    def test_optical_to_base_units_and_invalid_values(self):
        points = project_depth([2.0, math.nan, math.inf, 0.0], 2, 2, 2, 2, 0, 0, stride=1)
        self.assertEqual(points, [(2.0, 0.0, 0.35)])

    def test_projection_sign(self):
        points = project_depth([2, 2], 2, 1, 2, 2, 0, 0, stride=1)
        self.assertEqual(points[1], (2, -1, 0.35))

    def test_bad_shape_and_intrinsics(self):
        for data, fx in [([], 1), ([1], 0)]:
            with self.assertRaises(ValueError):
                project_depth(data, 1, 1, fx, 1, 0, 0)


class MissionTests(unittest.TestCase):
    def test_cancel_before_acceptance_waits_for_actual_result(self):
        m = Mission(); m.start(); m.request_cancel()
        self.assertEqual(m.state, State.WAITING)
        m.accepted(True)
        self.assertEqual(m.state, State.CANCELING)
        with self.assertRaises(RuntimeError): m.start()
        m.finished("canceled")
        self.assertEqual(m.state, State.CANCELED)

    def test_rejection_and_restart(self):
        m = Mission(); m.start(); m.accepted(False)
        self.assertEqual(m.state, State.FAILED)
        m.start(); m.accepted(True); m.finished("succeeded")
        self.assertEqual(m.state, State.SUCCEEDED)

    def test_cancel_success_race_uses_server_result(self):
        m = Mission(); m.start(); m.accepted(True); m.request_cancel(); m.finished("succeeded")
        self.assertEqual(m.state, State.SUCCEEDED)

    def test_scenarios_and_evidence_exports(self):
        for scenario, expected in [("normal", "SUCCEEDED"), ("blocked", "SUCCEEDED"), ("canceled", "CANCELED"), ("unreachable", "FAILED")]:
            with self.subTest(scenario=scenario):
                tmp = Path("artifacts/tests") / scenario
                result = run(scenario, tmp)
                self.assertEqual(result["result"], expected)
                self.assertEqual(result["replans"], int(scenario == "blocked"))
                self.assertTrue((Path(tmp) / "replay.html").is_file())


if __name__ == "__main__": unittest.main()
