# pyutils26 (cgshop2026-pyutils)

Utilities for working with CG:SHOP 2026 triangulation flip instances & solutions.

This library provides:

- Typed Pydantic schemas for instances & solutions
- Geometry primitives & fast predicates (backed by C++ / CGAL via pybind11)
- Triangulation flip exploration utilities (`FlipPartnerMap`, `FlippableTriangulation`)
- Solution verification helpers
- Zip archive reading/writing with robust validation & safety checks
- Simple visualization helpers for edges and flip sequences

This library is also used internally by us.

Check out [example_verification.ipynb](https://github.com/CG-SHOP/pyutils26/blob/main/example_verification.ipynb) for a quick demo on a simple instance.
Check out [example_zip.ipynb](https://github.com/CG-SHOP/pyutils26/blob/main/example_zip.ipynb) for a demo of the ZIP utilities that allows you to verify a whole archive of solutions safely.

---

## Installation

Stable release (PyPI):
```bash
pip install --verbose cgshop2026-pyutils
```

From a local clone (editable for development):
```bash
git clone https://github.com/CG-SHOP/pyutils26
cd pyutils26
pip install --verbose -e .
```

Direct from GitHub without cloning:
```bash
pip install --verbose git+https://github.com/CG-SHOP/pyutils26
```

Because parts of the package are compiled (CGAL + pybind11), the first install may take a few minutes.

System dependencies (Ubuntu example):
```bash
sudo apt update
sudo apt install -y build-essential python3-dev
```

Run tests to confirm everything works:
```bash
pytest -q
```

> Tip: Keep the package up to date. We will iterate during the competition period.

---

## Quick Start

Create an instance, build triangulations, apply flips, and verify a solution:

```python
from cgshop2026_pyutils.schemas import CGSHOP2026Instance, CGSHOP2026Solution
from cgshop2026_pyutils.geometry import Point, FlippableTriangulation
from cgshop2026_pyutils.verify import check_for_errors

# Define points (square) and two triangulations that will be flipped to a common form
points_x = [0, 1, 0, 1]
points_y = [0, 0, 1, 1]
triangulations = [  # Each triangulation is a list of interior edges
	[(0, 3)],        # diagonal 0-3
	[(1, 2)],        # diagonal 1-2 (the flip partner)
]

instance = CGSHOP2026Instance(
	instance_uid="demo-square",
	points_x=points_x,
	points_y=points_y,
	triangulations=triangulations,
)

# A solution that flips the diagonal in the first triangulation to match the second.
# flips is: one list per triangulation -> sequence of parallel flip sets -> each set is a list of edges
solution = CGSHOP2026Solution(
	instance_uid="demo-square",
	flips=[ [[(0,3)]] , [] ]  # flip edge (0,3) in triangulation 0; triangulation 1 already in target form
)

errors = check_for_errors(instance, solution)
print("Errors:", errors or "None ✔")
```

---

## Data Model (Schemas)

| Schema | Purpose |
|--------|---------|
| `CGSHOP2026Instance` | Holds points `(points_x, points_y)` and a list of triangulations (each a list of interior edges). |
| `CGSHOP2026Solution` | Holds a sequence of parallel flip sets for each triangulation leading them to a common triangulation. |

Key fields:
- `CGSHOP2026Instance.triangulations`: list of triangulations; each triangulation is a list of edges `(u,v)` with `u < v` preferred.
- `CGSHOP2026Solution.flips`: for each triangulation a list (sequence) of parallel flip sets; each parallel set is a list of edges that can be flipped simultaneously.
- `CGSHOP2026Solution.objective_value`: computed total number of parallel steps.

Loading & saving:
```python
from cgshop2026_pyutils.io import read_instance, read_solution

instance = read_instance("sample.instance.json")
solution = read_solution("sample.solution.json")
```

---

## Geometry / Triangulation API

Available via `from cgshop2026_pyutils.geometry import *`:

| Symbol | Type | Description |
|--------|------|-------------|
| `Point` | class (C++ binding) | Immutable 2D point supporting `.x()` / `.y()`. |
| `Segment` | class (C++ binding) | Segment primitive. |
| `is_triangulation(points, edges)` | function | Validates that edges (plus convex hull) form a triangulation on given points. |
| `compute_triangles(points, edges)` | function | Returns list of triangles (triples of point indices). |
| `do_cross(seg_a, seg_b)` | function | Segment intersection test (used for flippability). |
| `FlipPartnerMap` | class | Maintains flippable edges → partner mapping; supports flips and conflict analysis. |
| `FlippableTriangulation` | class | High-level wrapper: queue flips, commit them, fork, enumerate possible flips. |
| `expand_edges_by_convex_hull_edges(points, edges)` | function | Adds convex hull boundary to an edge set. |
| `draw_edges` | function | Matplotlib helper to plot points + edges. |
| `draw_flips` | function | Visualize triangulation plus queued flips & partners. |

### Building a triangulation
```python
from cgshop2026_pyutils.geometry import Point, FlippableTriangulation

points = [Point(0,0), Point(1,0), Point(0,1), Point(1,1)]
edges  = [(0,3)]  # one diagonal; hull edges are implicit

tri = FlippableTriangulation.from_points_edges(points, edges)
print("Possible flips:", tri.possible_flips())  # -> [(0,3)]

new_edge = tri.add_flip((0,3))  # returns partner (1,2)
tri.commit()
print("After flip, possible flips:", tri.possible_flips())
```

### Exploring multiple independent branches
```python
forked = tri.fork()
# Now forked and tri can diverge independently
```

### Detecting conflicts & partner edge
```python
edge = tri.possible_flips()[0]
partner = tri.get_flip_partner(edge)  # new convenience method
# For conflicts we still rely on the underlying map (may be wrapped later)
conflicts = tri._flip_map.conflicting_flips(edge)
print(edge, "partner=", partner, "conflicts=", conflicts)
```

---

## Verification API

Use `check_for_errors(instance, solution)` to validate that applying the flip sequences makes all triangulations converge to the same final triangulation.

Returns a list of error strings (empty list means success):
```python
from cgshop2026_pyutils.verify import check_for_errors
errs = check_for_errors(instance, solution)
if errs:
	print("Invalid solution:\n", "\n".join(errs))
else:
	print("Solution valid ✔")
```

Common errors:
- Non-flippable edge attempted
- Duplicate / conflicting flips in the same parallel set
- Final triangulations mismatch

---

## ZIP Utilities

Safely package & iterate over many solutions.

Writing:
```python
from cgshop2026_pyutils.zip.zip_writer import ZipWriter

with ZipWriter("solutions_bundle.zip") as zw:
	zw.add_solution(solution)  # or zw.add_instance(instance)
```

Reading & validating:
```python
from cgshop2026_pyutils.zip.zip_processor import ZipSolutionIterator, BadSolutionFile
from cgshop2026_pyutils.zip.zip_reader_errors import ZipReaderError

try:
	for sol in ZipSolutionIterator("solutions_bundle.zip"):
		print(sol.instance_uid, sol.objective_value)
except (ZipReaderError, BadSolutionFile) as e:
	print("ZIP problem:", e)
```

Safety checks include:
- File name sanitization (no absolute paths / traversal)
- Per-file size limit (default 250MB)
- Total decompressed size limit (default 2GB)
- CRC integrity check

---

## Visualization

```python
from cgshop2026_pyutils.geometry import draw_edges, draw_flips, Point, FlippableTriangulation
import matplotlib.pyplot as plt

points = [Point(0,0), Point(1,0), Point(0,1), Point(1,1)]
tri = FlippableTriangulation.from_points_edges(points, [(0,3)])
tri.add_flip((0,3))

fig, ax = plt.subplots(1,2, figsize=(6,3))
draw_edges(points, [(0,3)], ax=ax[0], show_indices=True)
draw_flips(tri, ax=ax[1], show_indices=True, title="Pending flip")
plt.show()
```

---

## File Formats

Instance JSON (minimal example):
```json
{
  "content_type": "CGSHOP2026_Instance",
  "instance_uid": "demo-square",
  "points_x": [0,1,0,1],
  "points_y": [0,0,1,1],
  "triangulations": [[ [0,3] ], [ [1,2] ]]
}
```

Solution JSON:
```json
{
  "content_type": "CGSHOP2026_Solution",
  "instance_uid": "demo-square",
  "flips": [ [[ [0,3] ]], [] ],
  "meta": {"author": "you"}
}
```

---

## Development

Rebuild native extension after C++ changes:
```bash
python setup.py develop
```

Install in editable mode:
```bash
pip install -e .
```

Run tests & lints (if ruff configured):
```bash
pytest -q
```

---

## Contributing

Issues & PRs welcome. Please:
1. Add/adjust unit tests for new behavior
2. Keep public API documented here
3. Run the test suite before submitting

---

## Changelog

- **0.3.0** (2025-11-04): Performance improvements.
- **0.2.1** (2025-11-03): Also accepting ".sol.json" as solution file extensions in `ZipProcessor`.
- **0.2.0** (2025-10-14): Added `get_edges()` to `FlippableTriangulation`; improved flip map rebuilding; enhanced error messages.
- **0.1.0** (2025-10-14): Initial release

---

## License

See `LICENSE` (likely MIT or similar—refer to the repository file).
