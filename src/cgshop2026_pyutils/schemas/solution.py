from typing import Literal
from pydantic import BaseModel, Field, NonNegativeInt, computed_field

# An edge flip is represented as a tuple of two vertex indices.
# A set of parallel flips is a list of such edges that can be flipped simultaneously. The order of edges in this list does not matter.
ParallelFlips = list[tuple[NonNegativeInt, NonNegativeInt]]
# A sequence of parallel flip sets is a list of such sets, representing the order in which they are applied.
ParallelFlipSequence = list[ParallelFlips]


class CGSHOP2026Solution(BaseModel):
    """
    This schema represents a solution for the CGSHOP 2026 challenge.
    It contains a sequence of parallel flip sets for each triangulation in the instance,
    leading to a common triangulation.
    The common triangulation does not need to be specified explicitly.
    Also the flip partners do not need to be specified, as they can be derived from the instance.
    """

    content_type: Literal["CGSHOP2026_Solution"] = "CGSHOP2026_Solution"

    instance_uid: str = Field(..., description="Unique identifier of the instance.")

    flips: list[ParallelFlipSequence] = Field(
        ...,
        description="For each triangulation in the instance, a sequence of parallel flip sets that lead to a common triangulation.",
    )

    meta: dict = Field(
        default_factory=dict,
        description="Optional metadata about the solution, e.g., the name of the algorithm used to compute it.",
    )

    @computed_field
    def objective_value(self) -> NonNegativeInt:
        """
        Computes the objective value of the solution, which is the total number of parallel flip sets used across all triangulations.
        """
        return sum(len(seq) for seq in self.flips)
