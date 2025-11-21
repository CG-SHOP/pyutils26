"""
Test that all modules can be imported successfully.
This catches compatibility issues with typing features across Python versions.
"""

import sys
import pytest


def test_python_version():
    """Verify we're running on supported Python version."""
    assert sys.version_info >= (3, 10), "Python 3.10+ required"


def test_import_geometry_module():
    """Test that the main geometry module imports successfully."""
    from cgshop2026_pyutils import geometry
    assert geometry is not None


def test_import_geometry_classes():
    """Test that key geometry classes can be imported."""
    from cgshop2026_pyutils.geometry import (
        Point,
        Segment,
        FlippableTriangulation,
        FlipPartnerMap,
        is_triangulation,
        compute_triangles,
        do_cross,
    )
    assert Point is not None
    assert Segment is not None
    assert FlippableTriangulation is not None
    assert FlipPartnerMap is not None
    assert is_triangulation is not None
    assert compute_triangles is not None
    assert do_cross is not None


def test_import_schemas():
    """Test that schema classes can be imported."""
    from cgshop2026_pyutils.schemas import (
        CGSHOP2026Instance,
        CGSHOP2026Solution,
    )
    assert CGSHOP2026Instance is not None
    assert CGSHOP2026Solution is not None


def test_import_io():
    """Test that IO functions can be imported."""
    from cgshop2026_pyutils.io import (
        read_instance,
        read_solution,
    )
    assert read_instance is not None
    assert read_solution is not None


def test_import_verify():
    """Test that verification module can be imported."""
    from cgshop2026_pyutils.verify import check_for_errors
    assert check_for_errors is not None


def test_import_zip_utilities():
    """Test that ZIP utilities can be imported."""
    from cgshop2026_pyutils.zip import (
        ZipSolutionIterator,
        ZipWriter,
    )
    assert ZipSolutionIterator is not None
    assert ZipWriter is not None


def test_import_instance_database():
    """Test that instance database classes can be imported."""
    from cgshop2026_pyutils.instance_database import InstanceDatabase
    assert InstanceDatabase is not None


def test_typing_extensions_compatibility():
    """Test that typing features work correctly across Python versions."""
    # This test ensures override and Self are available
    if sys.version_info >= (3, 12):
        from typing import override
    else:
        from typing_extensions import override
    
    if sys.version_info >= (3, 11):
        from typing import Self
    else:
        from typing_extensions import Self
    
    assert override is not None
    assert Self is not None


def test_override_decorator_usage():
    """Test that @override decorator works in actual classes."""
    from cgshop2026_pyutils.geometry import FlippableTriangulation
    from cgshop2026_pyutils.zip import ZipSolutionIterator
    
    # Just verify these classes can be instantiated (basic smoke test)
    # The fact they import successfully means @override is working
    assert FlippableTriangulation is not None
    assert ZipSolutionIterator is not None


def test_bindings_module_types():
    """Test that C++ binding types are available."""
    from cgshop2026_pyutils.geometry._bindings import (
        Point,
        Segment,
        FieldNumber,
    )
    
    # Test basic type instantiation
    p = Point(0, 0)
    assert p is not None
    
    fn = FieldNumber(42)
    assert fn is not None
    
    s = Segment(Point(0, 0), Point(1, 1))
    assert s is not None


def test_create_simple_triangulation():
    """Smoke test: create a simple triangulation to ensure everything works."""
    from cgshop2026_pyutils.geometry import Point, is_triangulation
    
    # Simple triangle
    points = [Point(0, 0), Point(1, 0), Point(0, 1)]
    edges = [(0, 1), (1, 2), (2, 0)]
    
    result = is_triangulation(points, edges, verbose=False)
    assert result is True


if __name__ == "__main__":
    # Allow running this test file directly
    pytest.main([__file__, "-v"])
