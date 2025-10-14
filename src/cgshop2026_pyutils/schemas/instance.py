from typing import Literal
from pydantic import BaseModel, Field, NonNegativeInt


class CGSHOP2026Instance(BaseModel):
    """
    This schema represents an instance for the CGSHOP 2026 challenge.
    It contains a set of points in the plane and a list of triangulations
    of these points.
    """

    content_type: Literal["CGSHOP2026_Instance"] = "CGSHOP2026_Instance"

    instance_uid: str = Field(..., description="Unique identifier of the instance.")

    points_x: list[int] = Field(
        ..., description="List of x-coordinates of points in the plane."
    )

    points_y: list[int] = Field(
        ..., description="List of y-coordinates of points in the plane."
    )

    triangulations: list[list[tuple[NonNegativeInt, NonNegativeInt]]] = Field(
        ...,
        description="List of triangulations, each represented as a "
        "list of edges given as pairs of indices into "
        "the `points_x` and `points_y` lists.",
    )
