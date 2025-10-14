from .flip_partner_map import FlipPartnerMap, normalize_edge
from ._bindings import is_triangulation


class FlippableTriangulation:
    """
    This class represents a triangulation that supports edge flips.
    It allows you to verify if a a solution is valid but can also be used as a component
    to build your optimization algorithm.
    """

    def __init__(self, flip_map: FlipPartnerMap):
        # Do not validate or build here to allow cheap copies/forks.
        self._flip_map = flip_map
        self._flip_queue = []
        self._conflicting_edges = set()

    def __eq__(self, other: object) -> bool:
        """
        Checks if two triangulations are equal (same edges and same pending flips).
        """
        if not isinstance(other, FlippableTriangulation):
            return False
        if self._flip_map.edges != other._flip_map.edges:
            return False
        flip_queue_set = {normalize_edge(*e) for e in self._flip_queue}
        other_flip_queue_set = {normalize_edge(*e) for e in other._flip_queue}
        return flip_queue_set == other_flip_queue_set

    @staticmethod
    def from_points_edges(
        points: list, edges: list[tuple[int, int]]
    ) -> "FlippableTriangulation":
        """
        Validates input and builds the internal flip map.
        Use this factory when creating an instance from raw points/edges.
        """
        if not is_triangulation(points, edges, verbose=False):
            raise ValueError(
                "The provided edges do not form a valid triangulation of the given points."
            )
        flip_map = FlipPartnerMap.build(points, edges)
        return FlippableTriangulation(flip_map)

    def fork(self) -> "FlippableTriangulation":
        """
        Creates a copy of the triangulation that can be modified independently.
        """
        return FlippableTriangulation(self._flip_map.deep_copy())
    
    def get_edges(self) -> list[tuple[int, int]]:
        """
        Returns the list of edges in the triangulation.
        """
        return list(self._flip_map.edges)

    def add_flip(self, edge: tuple[int, int]) -> tuple[int, int]:
        """
        Flips the given edge in the triangulation. It adds this flip to the list of pending flips.
        Ir will throw an error if the edge is not flippable.

        Args:
            edge: The edge to flip, represented as a tuple of vertex indices.

        Returns:
            The new edge created by the flip operation.
        """
        edge = normalize_edge(*edge)
        if edge in self._conflicting_edges:
            raise ValueError("Edge flip conflicts with previously added flips.")
        if not self._flip_map.is_flippable(edge):
            raise ValueError("Edge is not flippable.")
        if edge in self._flip_queue:
            raise ValueError("Edge flip already pending.")
        conflicts = self._flip_map.conflicting_flips(edge)
        self._conflicting_edges.update(conflicts)
        self._flip_queue.append(edge)
        return self._flip_map.get_flip_partner(edge)

    def commit(self):
        """
        Commits all pending flips to the triangulation.
        """
        for edge in self._flip_queue:
            self._flip_map.flip(edge)
        self._flip_queue.clear()
        self._conflicting_edges.clear()

    def possible_flips(self) -> list[tuple[int, int]]:
        """
        Returns a list of all edges that can currently be flipped.
        """
        return [
            e
            for e in self._flip_map.flippable_edges()
            if e not in self._conflicting_edges and e not in self._flip_queue
        ]

    def get_flip_partner(self, edge: tuple[int, int]) -> tuple[int, int]:
        """Return the flip partner of a flippable edge.

        This is a convenience wrapper around the underlying flip map to avoid
        accessing the internal `_flip_map` attribute from user code.

        Args:
            edge: The edge to query (order independent).

        Returns:
            The partner edge (u,v) that would replace the given edge when flipped.

        Raises:
            ValueError: If the edge is not flippable.
        """
        edge = normalize_edge(*edge)
        return self._flip_map.get_flip_partner(edge)
