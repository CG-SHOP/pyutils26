# An edge is represented as a tuple of two vertex indices.
Edge = tuple[int, int]
# A triangle is represented as a tuple of three vertex integers
Triangle = tuple[int, int, int]
# A set of parallel flips is a list of such edges that can be flipped simultaneously. The order of edges in this list does not matter.
ParallelFlips = list[Edge]
# A sequence of parallel flip sets is a list of such sets, representing the order in which they are applied.
ParallelFlipSequence = list[ParallelFlips]
