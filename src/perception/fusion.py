"""Nexus Edge Cooperative Perception — multi-agent sensor fusion.

Bayesian fusion, weighted consensus, quality scoring, and
data sharing for fleet-level situational awareness.
"""
import math, random
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional


@dataclass
class SensorReading:
    agent_id: str
    sensor_type: str   # lidar, camera, sonar, radar, imu, gps, depth
    value: float       # measured value
    uncertainty: float  # standard deviation
    timestamp: float = 0.0
    position: Tuple[float, float, float] = (0, 0, 0)  # x, y, z
    quality: float = 1.0  # sensor quality score (0-1)


@dataclass
class FusedEstimate:
    value: float
    uncertainty: float
    contributors: int
    weights: Dict[str, float] = field(default_factory=dict)
    quality_score: float = 0.0


class BayesianFusion:
    """Bayesian sensor fusion with uncertainty propagation."""

    def fuse_two(self, a: SensorReading, b: SensorReading) -> FusedEstimate:
        """Fuse two readings using inverse-variance weighting."""
        w_a = 1.0 / (a.uncertainty ** 2) if a.uncertainty > 0 else 0
        w_b = 1.0 / (b.uncertainty ** 2) if b.uncertainty > 0 else 0
        total_w = w_a + w_b
        if total_w == 0:
            return FusedEstimate((a.value + b.value) / 2, float('inf'), 2)

        fused_val = (w_a * a.value + w_b * b.value) / total_w
        fused_unc = 1.0 / math.sqrt(total_w)

        return FusedEstimate(fused_val, fused_unc, 2,
                            {a.agent_id: w_a / total_w, b.agent_id: w_b / total_w},
                            min(a.quality, b.quality))

    def fuse_multi(self, readings: List[SensorReading]) -> FusedEstimate:
        """Fuse N readings using optimal linear combination."""
        if not readings:
            return FusedEstimate(0, float('inf'), 0)
        if len(readings) == 1:
            return FusedEstimate(readings[0].value, readings[0].uncertainty,
                                1, {readings[0].agent_id: 1.0}, readings[0].quality)

        weighted_sum = 0.0
        weight_sum = 0.0
        weights = {}
        min_quality = 1.0

        for r in readings:
            w = r.quality / (r.uncertainty ** 2) if r.uncertainty > 0 else 0
            weighted_sum += w * r.value
            weight_sum += w
            weights[r.agent_id] = w
            min_quality = min(min_quality, r.quality)

        if weight_sum == 0:
            avg = sum(r.value for r in readings) / len(readings)
            return FusedEstimate(avg, max(r.uncertainty for r in readings),
                                len(readings), {}, 0.0)

        fused_val = weighted_sum / weight_sum
        fused_unc = 1.0 / math.sqrt(weight_sum)
        weights = {k: v / weight_sum for k, v in weights.items()}

        return FusedEstimate(fused_val, fused_unc, len(readings),
                            weights, min_quality)


class ConsensusEngine:
    """Weighted consensus across fleet agents."""

    def __init__(self, trust_weights: Optional[Dict[str, float]] = None):
        self.trust_weights = trust_weights or {}
        self.history: List[FusedEstimate] = []

    def weighted_consensus(self, readings: List[SensorReading]) -> FusedEstimate:
        """Consensus estimate using trust-weighted fusion."""
        if not readings:
            return FusedEstimate(0, float('inf'), 0)

        total_weight = 0.0
        weighted_val = 0.0
        weights = {}

        for r in readings:
            trust = self.trust_weights.get(r.agent_id, 0.5)
            sensor_w = r.quality / (r.uncertainty ** 2) if r.uncertainty > 0 else 0
            combined_w = trust * sensor_w

            weighted_val += combined_w * r.value
            total_weight += combined_w
            weights[r.agent_id] = combined_w

        if total_weight == 0:
            return FusedEstimate(
                sum(r.value for r in readings) / len(readings),
                max(r.uncertainty for r in readings), len(readings), {}, 0.0)

        fused_val = weighted_val / total_weight
        fused_unc = 1.0 / math.sqrt(total_weight) if total_weight > 0 else float('inf')
        weights = {k: v / total_weight for k, v in weights.items()}

        result = FusedEstimate(fused_val, fused_unc, len(readings), weights)
        self.history.append(result)
        return result

    def detect_outliers(self, readings: List[SensorReading],
                       sigma_threshold: float = 2.0) -> List[SensorReading]:
        """Flag readings that deviate more than sigma_threshold from consensus."""
        if len(readings) < 3:
            return []
        consensus = self.weighted_consensus(readings)
        outliers = []
        for r in readings:
            if consensus.uncertainty > 0:
                z_score = abs(r.value - consensus.value) / consensus.uncertainty
                if z_score > sigma_threshold:
                    outliers.append(r)
        return outliers


class QualityScorer:
    """Score sensor reading quality based on multiple factors."""

    def score(self, reading: SensorReading, reference: float = None,
              max_staleness_ms: float = 1000) -> float:
        score = 1.0

        # Uncertainty penalty
        if reading.uncertainty > 0:
            score *= max(0, 1.0 - reading.uncertainty * 0.1)

        # Staleness penalty (using import time)
        import time as _t
        age = (_t.time() - reading.timestamp) * 1000 if reading.timestamp > 0 else 0
        if age > max_staleness_ms:
            score *= max(0, 1.0 - (age - max_staleness_ms) / max_staleness_ms)

        # Reference consistency
        if reference is not None and reading.uncertainty > 0:
            deviation = abs(reading.value - reference) / reading.uncertainty
            if deviation > 3:
                score *= 0.1

        return max(0, min(1, score))


class PerceptionShare:
    """Manage data sharing between agents."""

    def __init__(self):
        self.shared_data: Dict[str, List[SensorReading]] = {}

    def publish(self, agent_id: str, readings: List[SensorReading]) -> None:
        if agent_id not in self.shared_data:
            self.shared_data[agent_id] = []
        self.shared_data[agent_id].extend(readings)
        # Keep last 100 per agent
        if len(self.shared_data[agent_id]) > 100:
            self.shared_data[agent_id] = self.shared_data[agent_id][-100:]

    def query(self, sensor_type: str = None,
              max_age_ms: float = 5000) -> List[SensorReading]:
        import time as _t
        now = _t.time()
        results = []
        for agent_id, readings in self.shared_data.items():
            for r in readings:
                if sensor_type and r.sensor_type != sensor_type:
                    continue
                if r.timestamp > 0 and (now - r.timestamp) * 1000 > max_age_ms:
                    continue
                results.append(r)
        return results

    def get_contributors(self) -> List[str]:
        return list(self.shared_data.keys())


def demo():
    print("=== Cooperative Perception ===\n")
    random.seed(42)

    fusion = BayesianFusion()
    consensus = ConsensusEngine(trust_weights={
        "auv_1": 0.9, "auv_2": 0.7, "auv_3": 0.8, "surface": 0.95
    })

    # Simulate depth readings from 4 agents
    true_depth = 5.0
    readings = [
        SensorReading("auv_1", "depth", true_depth + random.gauss(0, 0.1), 0.1, quality=0.95),
        SensorReading("auv_2", "depth", true_depth + random.gauss(0, 0.3), 0.3, quality=0.7),
        SensorReading("auv_3", "depth", true_depth + random.gauss(0, 0.15), 0.15, quality=0.85),
        SensorReading("surface", "depth", true_depth + random.gauss(0, 0.05), 0.05, quality=0.99),
    ]

    print("--- Raw Readings ---")
    for r in readings:
        print(f"  {r.agent_id}: {r.value:.3f}m (unc={r.uncertainty}, q={r.quality})")

    # Bayesian fusion
    fused = fusion.fuse_multi(readings)
    print(f"\n--- Bayesian Fusion ---")
    print(f"  Fused: {fused.value:.3f}m (unc={fused.uncertainty:.3f})")
    print(f"  Contributors: {fused.contributors}")
    print(f"  Weights: {', '.join(f'{k}:{v:.2f}' for k,v in fused.weights.items())}")

    # Trust-weighted consensus
    cons = consensus.weighted_consensus(readings)
    print(f"\n--- Trust-Weighted Consensus ---")
    print(f"  Consensus: {cons.value:.3f}m (unc={cons.uncertainty:.3f})")
    print(f"  Error from true: {abs(cons.value - true_depth):.4f}m")

    # Outlier detection
    outlier_readings = readings + [
        SensorReading("rogue", "depth", 12.0, 1.0, quality=0.3)  # way off
    ]
    outliers = consensus.detect_outliers(outlier_readings)
    print(f"\n--- Outlier Detection ---")
    for o in outliers:
        print(f"  OUTLIER: {o.agent_id} = {o.value:.1f}m")

    # Quality scoring
    scorer = QualityScorer()
    print("\n--- Quality Scores ---")
    for r in readings:
        q = scorer.score(r, reference=true_depth)
        print(f"  {r.agent_id}: quality={q:.3f}")

    # Data sharing
    share = PerceptionShare()
    share.publish("auv_1", readings[:2])
    share.publish("surface", readings[3:4])
    available = share.query(sensor_type="depth")
    print(f"\n--- Data Sharing ---")
    print(f"  Contributors: {share.get_contributors()}")
    print(f"  Available depth readings: {len(available)}")

    # Comparison
    print("\n--- Fusion Comparison ---")
    for n_agents in [2, 3, 4]:
        subset = readings[:n_agents]
        f = fusion.fuse_multi(subset)
        c = consensus.weighted_consensus(subset)
        print(f"  {n_agents} agents: bayesian={abs(f.value-true_depth):.4f}, "
              f"consensus={abs(c.value-true_depth):.4f} error")


if __name__ == "__main__":
    demo()
