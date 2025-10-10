from collections import defaultdict
from ._bindings import is_triangulation, compute_triangles, Point, do_cross, Segment

class FlipPartnerMap:
    """
    This class maintains a mapping of flippable edges in a triangulation to their flip partners.
    It allows checking if an edge is flippable, performing flips, and identifying conflicting flips.

    An edge is flippable if it is shared by exactly two triangles and the quadrilateral formed by these triangles is convex.
    This convexity can be checked by verifying that the edge to flip and its flip partner cross each other properly (i.e., they intersect at a point that is not an endpoint of either segment).

    All edges that belong to the two triangles incident to a flippable edge are considered conflicting flips, as flipping one of them would invalidate the other.
    """
    def __init__(self, points: list, edges: set[tuple[int, int]]):
        self.points = points
        self.edges = edges
        self.flip_map = {}

    @staticmethod
    def build(points: list, edges: list[tuple[int, int]]) -> "FlipPartnerMap":
        edge_ = {(min(u, v), max(u, v)) for u, v in edges}
        instance = FlipPartnerMap(points, edge_)
        instance.flip_map = instance._build_flip_map()
        return instance
    
    def compute_triangles(self) -> list[tuple[int, int, int]]:
        """
        Computes the triangles formed by the current edges in the flip map.
        """
        return compute_triangles(self.points, [edge for edge in self.edges])

    def _build_flip_map(self) -> dict[tuple[int, int], tuple[int, int]]:
        triangles = self.compute_triangles()
        # 1. Collect the triangles each edge is incident to.
        edge_to_triangles = defaultdict(list)
        for tri in triangles:
            edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])]
            for u, v in edges:
                if u > v:
                    u, v = v, u  # Store edges in a consistent order
                edge_to_triangles[(u, v)].append(tri)
        # 2. Build the flip map for edges that are shared by exactly two triangles.
        flip_map = {}
        for edge, tris in edge_to_triangles.items():
            if len(tris) == 2:
                tri1, tri2 = tris
                # Find the vertices opposite to the edge in each triangle
                opp1 = next(v for v in tri1 if v not in edge)
                opp2 = next(v for v in tri2 if v not in edge)
                if do_cross(Segment(self.points[edge[0]], self.points[edge[1]]), Segment(self.points[opp1], self.points[opp2])):
                    flip_map[edge] = (opp1, opp2)
        return flip_map
    
    def is_flippable(self, edge: tuple[int, int]) -> bool:
        """
        Checks if the given edge is flippable.
        """
        u, v = edge
        if u > v:
            u, v = v, u
        return (u, v) in self.flip_map
    
    def conflicting_flips(self, edge: tuple[int, int]) -> set[tuple[int, int]]:
        """
        These are the edges that cannot be flipped if the given edge is flipped.
        """
        u, v = edge
        if u > v:
            u, v = v, u
        if (u, v) not in self.flip_map:
            raise ValueError("Edge is not flippable")
        opp1, opp2 = self.flip_map[(u, v)]
        conflicting = set()
        for e in [(u, opp1), (v, opp1), (u, opp2), (v, opp2)]:
            if e[0] > e[1]:
                e = (e[1], e[0])
            if e in self.flip_map:
                conflicting.add(e)
        return conflicting
    
    def flip(self, edge: tuple[int, int]) -> tuple[int, int]:
        """
        Will flip the given edge and update the flip map accordingly.
        It will throw an error if the edge is not flippable.
        """
        u, v = edge
        if u > v:
            u, v = v, u
        if (u, v) not in self.edges:
            raise ValueError("Edge does not exist in the triangulation")
        if (u, v) not in self.flip_map:
            raise ValueError("Edge is not flippable")
        opp1, opp2 = self.flip_map.pop((u, v))
        new_edge = (opp1, opp2)
        if new_edge[0] > new_edge[1]:
            new_edge = (new_edge[1], new_edge[0])
        self.flip_map[new_edge] = (u, v)
        self.edges.remove((u, v))
        self.edges.add(new_edge)
        return new_edge
    
    def deep_copy(self) -> "FlipPartnerMap":
        copy = FlipPartnerMap(self.points, self.edges.copy())

        copy.flip_map = self.flip_map.copy()
        return copy
    
    def flippable_edges(self) -> list[tuple[int, int]]:
        """
        Returns a list of all currently flippable edges.
        """
        return list(self.flip_map.keys())
    
    def get_flip_partner(self, edge: tuple[int, int]) -> tuple[int, int]:
        """
        Returns the flip partner of the given edge.
        """
        u, v = edge
        if u > v:
            u, v = v, u
        if (u, v) not in self.flip_map:
            raise ValueError("Edge is not flippable")
        return self.flip_map[(u, v)]