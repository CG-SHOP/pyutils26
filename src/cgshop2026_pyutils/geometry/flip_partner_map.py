from collections import defaultdict
from ._bindings import compute_triangles, do_cross, Segment

def normalize_edge(v: int, w: int) -> tuple[int, int]:
    """Returns a tuple representing the edge in a consistent order (min, max)."""
    return (v, w) if v < w else (w, v)

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
        self.edge_to_triangles = defaultdict(list)

    @staticmethod
    def build(points: list, edges: list[tuple[int, int]]) -> "FlipPartnerMap":
        edge_ = {(min(u, v), max(u, v)) for u, v in edges}
        instance = FlipPartnerMap(points, edge_)
        instance.flip_map = {}
        instance._build_flip_map()
        return instance

    def compute_triangles(self) -> list[tuple[int, int, int]]:
        """
        Computes the triangles formed by the current edges in the flip map.
        """
        return compute_triangles(self.points, [edge for edge in self.edges])

    def _build_flip_map(self):
        triangles = self.compute_triangles()
        # 1. Collect the triangles each edge is incident to.
        self.edge_to_triangles = defaultdict(list)
        for tri in triangles:
            edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])]
            for u, v in edges:
                norm_edge = normalize_edge(u, v)
                self.edge_to_triangles[norm_edge].append(tri)
        # 2. Build the flip map for edges that are shared by exactly two triangles.
        self.flip_map = {}  # clear existing map
        for norm_edge in self.edge_to_triangles:
            self._update_flip_partner(norm_edge=norm_edge)

    
    def _update_flip_partner(self, norm_edge: tuple[int, int]):
        tris = self.edge_to_triangles[norm_edge]
        if len(tris) == 2:
            if opp_edge := self._check_flippability(norm_edge, tris[0], tris[1]):
                self.flip_map[norm_edge] = opp_edge
            elif norm_edge in self.flip_map:
                del self.flip_map[norm_edge]
        elif norm_edge in self.flip_map:
            del self.flip_map[norm_edge]
    
    def _check_flippability(self, norm_edge: tuple[int, int], triang_1: tuple[int, int, int], triang_2: tuple[int, int, int]) -> tuple[int, int]|None:
        opp1 = next(v for v in triang_1 if v not in norm_edge)
        opp2 = next(v for v in triang_2 if v not in norm_edge)
        if do_cross(
            Segment(self.points[norm_edge[0]], self.points[norm_edge[1]]),
            Segment(self.points[opp1], self.points[opp2]),
        ):
            return (opp1, opp2)
        return None

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
        u, v = normalize_edge(*edge)
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
        u, v = normalize_edge(*edge)
        if (u, v) not in self.edges:
            raise ValueError("Edge does not exist in the triangulation")
        if (u, v) not in self.flip_map:
            raise ValueError("Edge is not flippable")
        old_edge = (u, v)
        opp1, opp2 = self.flip_map.pop(old_edge)
        new_edge = normalize_edge(opp1, opp2)
        self.flip_map[new_edge] = old_edge

        # Update the triangles incident to the old edge
        self.edge_to_triangles.pop(old_edge)

        # Update triangles for the new edge
        new_tri_1 = (opp1, opp2, u)
        new_tri_2 = (opp1, opp2, v)
        self.edge_to_triangles[new_edge] = [new_tri_1, new_tri_2]

        # we need to replace the old triangle in the surrounding edges. This part is a bit ugly
        def contains_old_edge(tri):
            return old_edge[0] in tri and old_edge[1] in tri
        e1 = normalize_edge(opp1, u)
        self.edge_to_triangles[e1] = [new_tri_1]+[t for t in self.edge_to_triangles[e1] if not contains_old_edge(t)]
        e2 = normalize_edge(opp2, u)
        self.edge_to_triangles[e2] = [new_tri_1]+[t for t in self.edge_to_triangles[e2] if not contains_old_edge(t)]
        e3 = normalize_edge(opp1, v)
        self.edge_to_triangles[e3] = [new_tri_2]+[t for t in self.edge_to_triangles[e3] if not contains_old_edge(t)]
        e4 = normalize_edge(opp2, v)
        self.edge_to_triangles[e4] = [new_tri_2]+[t for t in self.edge_to_triangles[e4] if not contains_old_edge(t)]

        # Update the edges set
        self.edges.remove(old_edge)
        self.edges.add(new_edge)

        # The edges now could have become flippable or unflippable, so we need to update them all
        self._update_flip_partner(e1)
        self._update_flip_partner(e2)
        self._update_flip_partner(e3)
        self._update_flip_partner(e4)

        return new_edge

    def deep_copy(self) -> "FlipPartnerMap":
        copy = FlipPartnerMap(self.points, self.edges.copy())
        copy.flip_map = self.flip_map.copy()
        copy.edge_to_triangles = self.edge_to_triangles.copy()
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
        u, v = normalize_edge(*edge)
        if (u, v) not in self.flip_map:
            raise ValueError("Edge is not flippable")
        return self.flip_map[(u, v)]
