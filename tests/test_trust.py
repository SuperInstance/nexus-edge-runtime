"""Tests for src/trust/engine.py"""
import math
import pytest
from trust.engine import (
    TrustEngine, TrustWeights, TrustDimensions,
    AutonomyLevel, InteractionRecord
)


class TestTrustScoring:
    def test_initial_relationship(self):
        engine = TrustEngine()
        rel = engine.record_interaction("a", "b", success=True)
        assert rel.agent_a == "a"
        assert rel.agent_b == "b"
        assert rel.interaction_count == 1
        assert rel.dimensions.history == 1.0

    def test_history_ema(self):
        engine = TrustEngine()
        # first success -> history=1.0
        engine.record_interaction("a", "b", success=True)
        # second failure: history = 0.3*0 + 0.7*1.0 = 0.7
        rel = engine.record_interaction("a", "b", success=False)
        assert rel.dimensions.history == pytest.approx(0.7)

    def test_history_ema_multiple(self):
        engine = TrustEngine()
        for _ in range(10):
            engine.record_interaction("a", "b", success=True)
        rel = engine.record_interaction("a", "b", success=False)
        # history should be high but drop
        assert rel.dimensions.history > 0.5
        assert rel.dimensions.history < 1.0

    def test_capability_update(self):
        engine = TrustEngine()
        engine.record_interaction("a", "b", success=True, capability_score=0.9)
        rel = engine.record_interaction("a", "b", success=True, capability_score=0.5)
        # cap = 0.2*0.5 + 0.8*0.9 = 0.82
        assert rel.dimensions.capability == pytest.approx(0.82)

    def test_latency_score(self):
        engine = TrustEngine()
        rel = engine.record_interaction("a", "b", success=True, latency_ms=50.0)
        # latency_score = max(0, 1 - 50/1000) = 0.95
        # latency = 0.15*0.95 + 0.85*0.5 = 0.5675
        assert rel.dimensions.latency == pytest.approx(0.5675)

    def test_high_latency_low_score(self):
        engine = TrustEngine()
        rel = engine.record_interaction("a", "b", success=True, latency_ms=2000.0)
        # latency_score = max(0, 1 - 2000/1000) = 0
        # latency = 0.15*0 + 0.85*0.5 = 0.425
        assert rel.dimensions.latency == pytest.approx(0.425)


class TestCompositeAndWeights:
    def test_custom_weights(self):
        weights = TrustWeights(alpha=0.0, beta=1.0, gamma=0.0, delta=0.0)
        engine = TrustEngine(weights=weights)
        rel = engine.record_interaction("a", "b", success=True, capability_score=0.8)
        # composite should equal capability dimension
        assert rel.composite_score == pytest.approx(rel.dimensions.capability)

    def test_composite_math(self):
        weights = TrustWeights(alpha=0.5, beta=0.2, gamma=0.2, delta=0.1)
        engine = TrustEngine(weights=weights)
        rel = engine.record_interaction(
            "a", "b", success=True, capability_score=1.0, latency_ms=1.0)
        d = rel.dimensions
        expected = (0.5 * d.history + 0.2 * d.capability +
                    0.2 * d.latency + 0.1 * d.consistency)
        assert rel.composite_score == pytest.approx(expected)


class TestAutonomy:
    def test_autonomy_levels(self):
        assert AutonomyLevel.from_trust(0.1) == AutonomyLevel.MANUAL
        assert AutonomyLevel.from_trust(0.3) == AutonomyLevel.ASSISTED
        assert AutonomyLevel.from_trust(0.5) == AutonomyLevel.SUPERVISED
        assert AutonomyLevel.from_trust(0.7) == AutonomyLevel.CONDITIONAL
        assert AutonomyLevel.from_trust(0.9) == AutonomyLevel.HIGH
        assert AutonomyLevel.from_trust(0.99) == AutonomyLevel.FULL

    def test_record_updates_autonomy(self):
        engine = TrustEngine()
        rel = engine.record_interaction(
            "a", "b", success=True, capability_score=1.0, latency_ms=1.0)
        assert rel.autonomy_level >= AutonomyLevel.CONDITIONAL

    def test_get_autonomy_no_history(self):
        engine = TrustEngine()
        assert engine.get_autonomy_level("a", "b") == AutonomyLevel.MANUAL


class TestDelegation:
    def test_can_delegate_trusted(self):
        engine = TrustEngine()
        for _ in range(5):
            engine.record_interaction(
                "a", "b", success=True, capability_score=0.95, latency_ms=10.0)
        ok, reason = engine.can_delegate("a", "b", min_level=3)
        assert ok is True
        assert reason == "OK"

    def test_can_delegate_no_history(self):
        engine = TrustEngine()
        ok, reason = engine.can_delegate("a", "b", min_level=3)
        assert ok is False
        assert "No trust history" in reason

    def test_can_delegate_low_autonomy(self):
        engine = TrustEngine()
        engine.record_interaction("a", "b", success=False)
        ok, reason = engine.can_delegate("a", "b", min_level=3)
        assert ok is False
        assert "Autonomy L" in reason


class TestPropagation:
    def test_direct_trust(self):
        engine = TrustEngine()
        engine.record_interaction("a", "b", success=True, capability_score=1.0)
        mapping = engine.propagate_trust("a", "b", depth=2)
        assert "b" in mapping
        assert mapping["b"] == pytest.approx(engine.get_trust("a", "b").composite_score)

    def test_transitive_trust(self):
        engine = TrustEngine()
        engine.record_interaction("a", "b", success=True, capability_score=1.0)
        engine.record_interaction("b", "c", success=True, capability_score=1.0)
        mapping = engine.propagate_trust("a", "b", depth=2)
        assert "c" in mapping
        # transitive score = source_score * target_score * 0.5
        a_to_b = engine.get_trust("a", "b").composite_score
        b_to_c = engine.get_trust("b", "c").composite_score
        assert mapping["c"] == pytest.approx(a_to_b * b_to_c * 0.5)


class TestFleetSummary:
    def test_empty_fleet(self):
        engine = TrustEngine()
        summary = engine.fleet_summary()
        assert summary["agents"] == 0
        assert summary["relationships"] == 0
        assert summary["avg_trust"] == 0

    def test_summary_metrics(self):
        engine = TrustEngine()
        engine.record_interaction("a", "b", success=True, capability_score=0.8)
        engine.record_interaction("b", "c", success=False)
        summary = engine.fleet_summary()
        assert summary["agents"] == 3
        assert summary["relationships"] == 2
        assert 0 < summary["avg_trust"] < 1
        assert summary["min_trust"] < summary["max_trust"]


class TestConsistency:
    def test_consistency_after_five_records(self):
        engine = TrustEngine()
        for i in range(6):
            engine.record_interaction("a", "b", success=(i % 2 == 0))
        rel = engine.get_trust("a", "b")
        # 6 records, alternating successes -> nonzero std -> consistency < 1
        assert rel.dimensions.consistency < 1.0
        assert rel.dimensions.consistency >= 0.0
