from .schemas import CGSHOP2026Instance, CGSHOP2026Solution
from .geometry import FlippableTriangulation, Point


def check_for_errors(
    instance: CGSHOP2026Instance, solution: CGSHOP2026Solution, full_recompute: bool = False, verbose: bool = False
) -> list[str]:
    """
    Verifies the given solution against the provided instance and returns a list of error messages if any issues are found.
    """
    points = [Point(x, y) for x, y in zip(instance.points_x, instance.points_y)]
    triangulations = [
        FlippableTriangulation.from_points_edges(points, edges)
        for edges in instance.triangulations
    ]
    for tri, flip_sequence in zip(triangulations, solution.flips):
        if verbose:
            print(f"Verifying flips for triangulation with {len(tri.get_edges())} edges.")
        for parallel_flips in flip_sequence:
            for edge in parallel_flips:
                try:
                    tri.add_flip(edge)
                except ValueError as e:
                    return [f"Error when flipping edge {edge} in triangulation: {e}"]
            tri.commit()
            if full_recompute:
                tri._flip_map._rebuild_flip_map()
    if verbose:
        print("Final triangulations computed, checking for equality...")
    for i in range(1, len(triangulations)):
        if triangulations[i] != triangulations[0]:
            return [
                f"Final triangulations do not match. Triangulation 0 and {i} differ."
            ]
    return []
