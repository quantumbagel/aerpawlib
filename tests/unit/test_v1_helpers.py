"""Unit tests for aerpawlib v1 helpers module."""

import asyncio
import threading

import pytest

from aerpawlib.v1.helpers import (
    ThreadSafeValue,
    heading_difference,
    normalize_heading,
    validate_tolerance,
    wait_for_condition,
    wait_for_value_change,
)
from aerpawlib.v1.constants import (
    MIN_POSITION_TOLERANCE_M,
    MAX_POSITION_TOLERANCE_M,
)


class TestWaitForCondition:
    """wait_for_condition and wait_for_value_change."""

    @pytest.mark.asyncio
    async def test_condition_met_immediately(self):
        result = await wait_for_condition(lambda: True, timeout=1.0)
        assert result is True

    @pytest.mark.asyncio
    async def test_condition_met_after_delay(self):
        state = {"count": 0}

        def cond():
            state["count"] += 1
            return state["count"] >= 3

        await wait_for_condition(cond, timeout=5.0, poll_interval=0.01)
        assert state["count"] >= 3

    @pytest.mark.asyncio
    async def test_timeout_raises(self):
        with pytest.raises(TimeoutError):
            await wait_for_condition(
                lambda: False,
                timeout=0.1,
                poll_interval=0.01,
                timeout_message="Operation timed out",
            )

    @pytest.mark.asyncio
    async def test_wait_for_value_change(self):
        state = {"val": 0}

        async def setter():
            await asyncio.sleep(0.05)
            state["val"] = 42

        asyncio.create_task(setter())
        result = await wait_for_value_change(
            lambda: state["val"], 42, timeout=1.0, poll_interval=0.01
        )
        assert result == 42


class TestValidateTolerance:
    """validate_tolerance."""

    def test_valid(self):
        assert validate_tolerance(1.0) == 1.0
        assert validate_tolerance(MIN_POSITION_TOLERANCE_M) == MIN_POSITION_TOLERANCE_M
        assert validate_tolerance(MAX_POSITION_TOLERANCE_M) == MAX_POSITION_TOLERANCE_M

    def test_too_small_raises(self):
        with pytest.raises(ValueError, match="at least"):
            validate_tolerance(0.05)

    def test_too_large_raises(self):
        with pytest.raises(ValueError, match="at most"):
            validate_tolerance(150)



class TestNormalizeHeading:
    """normalize_heading."""

    def test_already_normalized(self):
        assert normalize_heading(90) == 90
        assert normalize_heading(0) == 0

    def test_wraps_negative(self):
        assert normalize_heading(-90) == 270

    def test_wraps_over_360(self):
        assert normalize_heading(450) == 90


class TestHeadingDifference:
    """heading_difference."""

    def test_same_heading(self):
        assert heading_difference(90, 90) == 0

    def test_180_apart(self):
        assert heading_difference(0, 180) == 180

    def test_wraps_correctly(self):
        assert heading_difference(350, 10) == 20


class TestThreadSafeValue:
    """ThreadSafeValue — basic and concurrent access."""

    def test_initial_value(self):
        v = ThreadSafeValue(42)
        assert v.get() == 42

    def test_set_and_get(self):
        v = ThreadSafeValue(0)
        v.set(99)
        assert v.get() == 99

    def test_default_none(self):
        v = ThreadSafeValue()
        assert v.get() is None

    def test_compare_and_set_success(self):
        v = ThreadSafeValue("old")
        result = v.compare_and_set("old", "new")
        assert result is True
        assert v.get() == "new"

    def test_compare_and_set_failure(self):
        v = ThreadSafeValue("old")
        result = v.compare_and_set("wrong", "new")
        assert result is False
        assert v.get() == "old"  # unchanged

    def test_compare_and_set_returns_false_on_no_match(self):
        v = ThreadSafeValue(100)
        assert v.compare_and_set(999, 0) is False
        assert v.get() == 100

    def test_thread_safe_concurrent_writes(self):
        """Many threads incrementing should not corrupt state."""
        v = ThreadSafeValue(0)
        iterations = 1000

        def increment():
            for _ in range(iterations):
                current = v.get()
                v.set(current + 1)

        threads = [threading.Thread(target=increment) for _ in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # Value is positive (exact count can vary due to race, but no crash)
        assert v.get() > 0

    def test_stores_none(self):
        v = ThreadSafeValue(42)
        v.set(None)
        assert v.get() is None

    def test_stores_complex_object(self):
        obj = {"key": [1, 2, 3]}
        v = ThreadSafeValue(obj)
        assert v.get() is obj


class TestWaitForConditionExtended:
    """Additional wait_for_condition scenarios."""

    @pytest.mark.asyncio
    async def test_no_timeout_condition_met_quickly(self):
        """Without a timeout, should return True immediately if already true."""
        result = await wait_for_condition(lambda: True, timeout=None)
        assert result is True

    @pytest.mark.asyncio
    async def test_timeout_message_in_exception(self):
        """The TimeoutError should carry the custom message."""
        msg = "custom timeout reason"
        with pytest.raises(TimeoutError, match=msg):
            await wait_for_condition(
                lambda: False,
                timeout=0.05,
                poll_interval=0.01,
                timeout_message=msg,
            )

    @pytest.mark.asyncio
    async def test_condition_checked_multiple_times(self):
        """Condition function is polled repeatedly."""
        calls = [0]

        def cond():
            calls[0] += 1
            return calls[0] >= 5

        await wait_for_condition(cond, timeout=5.0, poll_interval=0.01)
        assert calls[0] >= 5

    @pytest.mark.asyncio
    async def test_returns_true_on_success(self):
        result = await wait_for_condition(lambda: True)
        assert result is True


class TestWaitForValueChangeExtended:
    """Additional wait_for_value_change scenarios."""

    @pytest.mark.asyncio
    async def test_timeout_raises(self):
        with pytest.raises(TimeoutError):
            await wait_for_value_change(
                lambda: 0,
                target_value=1,
                timeout=0.05,
                poll_interval=0.01,
            )

    @pytest.mark.asyncio
    async def test_returns_target_when_met(self):
        val = [0]

        async def setter():
            await asyncio.sleep(0.02)
            val[0] = 7

        asyncio.create_task(setter())
        result = await wait_for_value_change(
            lambda: val[0], 7, timeout=1.0, poll_interval=0.005
        )
        assert result == 7

    @pytest.mark.asyncio
    async def test_works_with_none_target(self):
        val: list = [42]

        async def setter():
            await asyncio.sleep(0.02)
            val[0] = None

        asyncio.create_task(setter())
        result = await wait_for_value_change(
            lambda: val[0], None, timeout=1.0, poll_interval=0.005
        )
        assert result is None


class TestNormalizeHeadingExtended:
    """Edge cases not in the base test."""

    def test_zero(self):
        assert normalize_heading(0) == 0

    def test_exactly_360_wraps_to_0(self):
        assert normalize_heading(360) == 0

    def test_negative_180(self):
        assert normalize_heading(-180) == 180

    def test_large_positive(self):
        assert normalize_heading(720) == 0

    def test_large_negative(self):
        assert normalize_heading(-360) == 0

    def test_fractional(self):
        result = normalize_heading(361.5)
        assert abs(result - 1.5) < 1e-10


class TestHeadingDifferenceExtended:
    """Edge cases for heading_difference."""

    def test_0_and_0(self):
        assert heading_difference(0, 0) == 0

    def test_0_and_360_equivalent(self):
        # 0 and 360 are the same heading
        assert heading_difference(0, 360) == 0

    def test_270_to_90_shortest_180(self):
        # Shortest path is 180 degrees either way
        assert heading_difference(270, 90) == 180

    def test_350_to_10_is_20(self):
        assert heading_difference(350, 10) == 20

    def test_10_to_350_is_20(self):
        assert heading_difference(10, 350) == 20

    def test_max_difference_is_180(self):
        assert heading_difference(0, 180) == 180
        assert heading_difference(90, 270) == 180

    def test_negative_heading_input(self):
        # -90 normalized is 270; difference to 90 is 180
        assert heading_difference(-90, 90) == 180


class TestValidateToleranceExtended:
    """Param name appears in ValueError message."""

    def test_param_name_in_error_too_small(self):
        with pytest.raises(ValueError) as exc_info:
            validate_tolerance(0.0, param_name="my_tolerance")
        assert "my_tolerance" in str(exc_info.value)

    def test_param_name_in_error_too_large(self):
        with pytest.raises(ValueError) as exc_info:
            validate_tolerance(200.0, param_name="my_tolerance")
        assert "my_tolerance" in str(exc_info.value)

    def test_boundary_exactly_min(self):
        assert validate_tolerance(MIN_POSITION_TOLERANCE_M) == MIN_POSITION_TOLERANCE_M

    def test_boundary_exactly_max(self):
        assert validate_tolerance(MAX_POSITION_TOLERANCE_M) == MAX_POSITION_TOLERANCE_M

