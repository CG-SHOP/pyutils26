from cgshop2026_pyutils.schemas import CGSHOP2026Instance, CGSHOP2026Solution
from cgshop2026_pyutils.geometry import Point, is_triangulation
from cgshop2026_pyutils.verify import check_for_errors


def test_instance_1():
    points = [((0, 2)), (0, 0), (5, 0), (5, 2), (4, 1), (1, 1)]
    triang_1 = [(0, 5), (0, 4), (1, 4), (1, 5), (2, 4), (3, 4), (4, 5)]
    triang_2 = [(0, 5), (1, 5), (2, 4), (2, 5), (3, 4), (3, 5), (4, 5)]
    triang_3 = [(0, 5), (1, 4), (1, 5), (1, 3), (2, 4), (3, 4), (3, 5)]
    instance = CGSHOP2026Instance(
        instance_uid="test_instance_1",
        points_x=[x for x, y in points],
        points_y=[y for x, y in points],
        triangulations=[triang_1, triang_2, triang_3],
    )
    for triang in instance.triangulations:
        points = [Point(x, y) for x, y in zip(instance.points_x, instance.points_y)]
        assert is_triangulation(points, triang, verbose=False), (
            f"Triangulation {triang} is not valid for the given points."
        )
    flips_1 = []
    flips_2 = [[(3, 5), (2, 5)]]
    flips_3 = [[(1, 3)], [(3, 5)]]
    solution = CGSHOP2026Solution(
        instance_uid="test_instance_1", flips=[flips_1, flips_2, flips_3]
    )
    errors = check_for_errors(instance, solution)
    assert not errors, f"Errors found in solution: {errors}"
