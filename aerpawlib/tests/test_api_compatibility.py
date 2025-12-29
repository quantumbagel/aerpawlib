"""
Cross-API compatibility tests.

These tests verify that the different API versions (legacy, v1, v2) maintain
compatibility where expected, and that migrations between versions work correctly.
"""
import os
import sys

import pytest

# Add the parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))


class TestVectorNEDCompatibility:
    """Tests for VectorNED compatibility across API versions."""

    def test_v1_vector_same_behavior_as_legacy(self):
        """Test that v1 VectorNED behaves the same as legacy."""
        from aerpawlib.legacy.util import VectorNED as LegacyVectorNED
        from aerpawlib.v1.util import VectorNED as V1VectorNED

        # Test same operations produce same results
        legacy_v = LegacyVectorNED(3, 4, 5)
        v1_v = V1VectorNED(3, 4, 5)

        # Hypot should be the same
        assert abs(legacy_v.hypot() - v1_v.hypot()) < 0.0001

        # Addition should work the same
        legacy_sum = legacy_v + LegacyVectorNED(1, 2, 3)
        v1_sum = v1_v + V1VectorNED(1, 2, 3)
        assert legacy_sum.north == v1_sum.north
        assert legacy_sum.east == v1_sum.east
        assert legacy_sum.down == v1_sum.down

    def test_v2_vector_compatible_with_legacy(self):
        """Test that v2 VectorNED is API compatible with legacy."""
        from aerpawlib.legacy.util import VectorNED as LegacyVectorNED
        from aerpawlib.v2.types import VectorNED as V2VectorNED

        legacy_v = LegacyVectorNED(3, 4, 5)
        v2_v = V2VectorNED(3, 4, 5)

        # Both should have north, east, down
        assert legacy_v.north == v2_v.north
        assert legacy_v.east == v2_v.east
        assert legacy_v.down == v2_v.down

        # Both should have hypot (v2 also has magnitude as alias)
        assert abs(legacy_v.hypot() - v2_v.hypot()) < 0.0001

        # Both should have norm
        legacy_norm = legacy_v.norm()
        v2_norm = v2_v.norm()
        assert abs(legacy_norm.hypot() - v2_norm.magnitude()) < 0.0001

    def test_v2_vector_has_additional_features(self):
        """Test that v2 VectorNED has additional features."""
        from aerpawlib.v2.types import VectorNED

        v = VectorNED(3, 4, 0)

        # v2 should have magnitude as alias for hypot
        assert hasattr(v, 'magnitude')
        assert v.magnitude() == v.hypot()

        # v2 should have normalize as alias for norm
        assert hasattr(v, 'normalize')

        # v2 should have heading
        assert hasattr(v, 'heading')

        # v2 should have dot_product
        assert hasattr(v, 'dot_product')


class TestCoordinateCompatibility:
    """Tests for Coordinate compatibility across API versions."""

    def test_v1_coordinate_same_behavior_as_legacy(self):
        """Test that v1 Coordinate behaves the same as legacy."""
        from aerpawlib.legacy.util import Coordinate as LegacyCoordinate
        from aerpawlib.v1.util import Coordinate as V1Coordinate

        legacy_c = LegacyCoordinate(35.7275, -78.6960, 100)
        v1_c = V1Coordinate(35.7275, -78.6960, 100)

        # Properties should be the same
        assert legacy_c.lat == v1_c.lat
        assert legacy_c.lon == v1_c.lon
        assert legacy_c.alt == v1_c.alt

        # Ground distance should be the same
        legacy_c2 = LegacyCoordinate(35.7280, -78.6965, 100)
        v1_c2 = V1Coordinate(35.7280, -78.6965, 100)
        legacy_dist = legacy_c.ground_distance(legacy_c2)
        v1_dist = v1_c.ground_distance(v1_c2)
        assert abs(legacy_dist - v1_dist) < 1  # Within 1 meter

    def test_v2_coordinate_compatible_with_legacy(self):
        """Test that v2 Coordinate is API compatible with legacy."""
        from aerpawlib.legacy.util import Coordinate as LegacyCoordinate
        from aerpawlib.v2.types import Coordinate as V2Coordinate

        legacy_c = LegacyCoordinate(35.7275, -78.6960, 100)
        v2_c = V2Coordinate(35.7275, -78.6960, 100)

        # v2 uses latitude/longitude as primary, lat/lon as aliases
        assert legacy_c.lat == v2_c.lat
        assert legacy_c.lon == v2_c.lon
        assert legacy_c.alt == v2_c.alt

        # Both should support vector operations
        from aerpawlib.legacy.util import VectorNED as LegacyVectorNED
        from aerpawlib.v2.types import VectorNED as V2VectorNED

        legacy_result = legacy_c + LegacyVectorNED(100, 50, 0)
        v2_result = v2_c + V2VectorNED(100, 50, 0)

        # Results should be similar (within floating point tolerance)
        assert abs(legacy_result.lat - v2_result.lat) < 0.0001
        assert abs(legacy_result.lon - v2_result.lon) < 0.0001

    def test_v2_coordinate_has_additional_features(self):
        """Test that v2 Coordinate has additional features."""
        from aerpawlib.v2.types import Coordinate

        c = Coordinate(35.7275, -78.6960, 100)

        # v2 should have name field
        assert hasattr(c, 'name')

        # v2 should have to_json/from_json
        assert hasattr(c, 'to_json')
        assert hasattr(Coordinate, 'from_json')

        # v2 should have bearing_to
        assert hasattr(c, 'bearing_to')

        # v2 should have midpoint_to
        assert hasattr(c, 'midpoint_to')

        # v2 should have interpolate_to
        assert hasattr(c, 'interpolate_to')


class TestRunnerCompatibility:
    """Tests for Runner compatibility across API versions."""

    def test_entrypoint_decorator_compatibility(self):
        """Test that @entrypoint works similarly across versions."""
        from aerpawlib.legacy.runner import entrypoint as legacy_entrypoint
        from aerpawlib.v1.runner import entrypoint as v1_entrypoint
        from aerpawlib.v2.runner import entrypoint as v2_entrypoint

        @legacy_entrypoint
        async def legacy_entry():
            pass

        @v1_entrypoint
        async def v1_entry():
            pass

        @v2_entrypoint
        async def v2_entry():
            pass

        # Legacy and v1 should mark with _entrypoint attribute
        assert hasattr(legacy_entry, '_entrypoint')
        assert hasattr(v1_entry, '_entrypoint')

        # v2 uses descriptor pattern
        from aerpawlib.v2.runner import MethodDescriptor
        assert isinstance(v2_entry, MethodDescriptor)

    def test_state_decorator_compatibility(self):
        """Test that @state works similarly across versions."""
        from aerpawlib.legacy.runner import state as legacy_state
        from aerpawlib.v1.runner import state as v1_state
        from aerpawlib.v2.runner import state as v2_state

        @legacy_state("test", first=True)
        async def legacy_state_func():
            pass

        @v1_state("test", first=True)
        async def v1_state_func():
            pass

        @v2_state("test", first=True)
        async def v2_state_func():
            pass

        # Legacy and v1 should mark with _is_state attribute
        assert legacy_state_func._is_state is True
        assert legacy_state_func._state_first is True
        assert v1_state_func._is_state is True
        assert v1_state_func._state_first is True

        # v2 uses descriptor with state_config
        from aerpawlib.v2.runner import MethodDescriptor
        assert isinstance(v2_state_func, MethodDescriptor)
        assert v2_state_func.state_config.is_initial is True


class TestDefaultAPIVersion:
    """Tests for the default API version exposed by aerpawlib."""

    def test_default_import_is_v1(self):
        """Test that default import gives v1 API."""
        import aerpawlib

        # The default should be v1 (according to __init__.py)
        assert aerpawlib.__version__ == "2.0.0"

        # Check that BasicRunner is available
        assert hasattr(aerpawlib, 'BasicRunner')
        assert hasattr(aerpawlib, 'entrypoint')

    def test_explicit_v1_import(self):
        """Test explicit v1 import."""
        from aerpawlib.v1 import BasicRunner, entrypoint, Coordinate, VectorNED

        assert BasicRunner is not None
        assert entrypoint is not None
        assert Coordinate is not None
        assert VectorNED is not None

    def test_explicit_v2_import(self):
        """Test explicit v2 import."""
        from aerpawlib.v2 import (
            Drone,
            BasicRunner,
            entrypoint,
            Coordinate,
            VectorNED,
            SafetyLimits,
            CommandHandle,
        )

        assert Drone is not None
        assert BasicRunner is not None
        assert entrypoint is not None
        assert Coordinate is not None
        assert VectorNED is not None
        assert SafetyLimits is not None
        assert CommandHandle is not None


class TestMigrationPatterns:
    """Tests demonstrating migration patterns between API versions."""

    def test_coordinate_migration_legacy_to_v2(self):
        """Demonstrate migrating Coordinate usage from legacy to v2."""
        from aerpawlib.legacy.util import Coordinate as LegacyCoord, VectorNED as LegacyVec
        from aerpawlib.v2.types import Coordinate as V2Coord, VectorNED as V2Vec

        # Legacy style
        legacy_home = LegacyCoord(35.7275, -78.6960, 0)
        legacy_target = legacy_home + LegacyVec(100, 0, -50)

        # v2 style - same operations work
        v2_home = V2Coord(35.7275, -78.6960, 0, "Home")  # Can add name
        v2_target = v2_home + V2Vec(100, 0, -50)

        # Results should be equivalent
        assert abs(legacy_target.lat - v2_target.lat) < 0.0001
        assert abs(legacy_target.alt - v2_target.alt) < 0.1

    def test_vector_migration_legacy_to_v2(self):
        """Demonstrate migrating VectorNED usage from legacy to v2."""
        from aerpawlib.legacy.util import VectorNED as LegacyVec
        from aerpawlib.v2.types import VectorNED as V2Vec

        # Legacy style
        legacy_v = LegacyVec(3, 4, 0)
        legacy_mag = legacy_v.hypot()
        legacy_norm = legacy_v.norm()

        # v2 style - same operations work, plus aliases
        v2_v = V2Vec(3, 4, 0)
        v2_mag = v2_v.magnitude()  # Or hypot() for compatibility
        v2_norm = v2_v.normalize()  # Or norm() for compatibility

        # Results should be equivalent
        assert legacy_mag == v2_mag
        assert abs(legacy_norm.hypot() - v2_norm.magnitude()) < 0.0001


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

