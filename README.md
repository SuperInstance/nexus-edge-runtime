# nexus-edge-runtime

Edge runtime for autonomous agents in the Cocapn fleet — generalized beyond maritime robotics for IoT, industrial, aerial, and marine domains.

## Core Modules

### Bytecode VM (`src/core/vm.py`) — 15.0K chars
33-opcode stack-based VM: 8-byte instructions, 32 registers (16 GP + 16 IO-mapped), 64KB memory, 1024-deep stack. Assembler, disassembler, bytecode validator. ESP32-S3 deployment + Jetson supervision.

**Opcode table:**

| Range | Group | Opcodes |
|-------|-------|---------|
| 0x00–0x07 | Stack | NOP, PUSH_I8, PUSH_I16, PUSH_F32, POP, DUP, SWAP, ROT |
| 0x08–0x10 | Arithmetic | ADD_F, SUB_F, MUL_F, DIV_F, NEG_F, ABS_F, MIN_F, MAX_F, CLAMP_F |
| 0x11–0x15 | Compare | EQ_F, LT_F, GT_F, LTE_F, GTE_F |
| 0x16–0x19 | Logic | AND_B, OR_B, XOR_B, NOT_B |
| 0x1A–0x1C | I/O | READ_PIN, WRITE_PIN, READ_TIMER_MS |
| 0x1D–0x20 | Control | JUMP, JUMP_IF_FALSE, JUMP_IF_TRUE, HALT |

`HALT` (0x20) cleanly terminates execution by setting `state.halted = True` — no `VMError` raised. Programs without an explicit `HALT` still terminate safely via the PC-out-of-bounds check (`max_cycles`-bounded).

### INCREMENTS Trust Engine (`src/trust/engine.py`) — 10.4K chars
Multi-dimensional trust: history (EMA), capability, latency, consistency. Composite scoring with configurable weights. Autonomy levels L0-L5. Transitive trust propagation. Trust decay.

### Wire Protocol (`src/wire/protocol.py`) — 5.8K chars
Length-prefixed framed protocol: `[PREAMBLE:2B][SRC:1B][DST:1B][TYPE:1B][SEQ:2B][LEN:2B][PAYLOAD:N][CRC16:2B]`. CRC-16/CCITT. Stream parser with partial buffering. 14 message types.

### Safety System (`src/safety/system.py`) — 11.4K chars
4-tier defense-in-depth:
- **Tier 1 HARDWARE**: Kill switch, watchdog IC (<1us)
- **Tier 2 FIRMWARE**: Stack canary, safe-state outputs (<1ms)
- **Tier 3 SUPERVISORY**: Heartbeat monitoring, state machine (<100ms)
- **Tier 4 APPLICATION**: Bytecode validation, trust-gated autonomy (<1s)

### Self-Healing (`src/safety/self_healing.py`) — 11.4K chars
Component health monitoring with heartbeat timeouts and error-count tracking, fault classification into WARNING/MINOR/MAJOR/CRITICAL, and recovery-strategy selection (restart, backup switch, reduce load, safe mode, escalate). Includes graceful degradation assessment that maps fleet-wide health to capability levels.

### Intent Compiler (`src/reflex/compiler.py`) — 14.8K chars
NL intent → IR → bytecode pipeline. Parses action + target + value + condition. Supports: maintain, navigate, monitor (conditional), alert.

### Fleet Coordination (`src/fleet/coordination.py`) — 11.4K chars
Task assignment with trust-aware scoring, capability matching, distance-weighted selection. Delegation with trust gates. Rendezvous planning (centroid). Formation planning (line, V, circle).

### Cooperative Perception (`src/perception/fusion.py`) — 9.7K chars
Bayesian sensor fusion with inverse-variance weighting. Trust-weighted consensus. Outlier detection (sigma threshold). Quality scoring. Data sharing with staleness filtering.

### Digital Twin (`src/digital_twin/twin.py`) — 8.8K chars
Real-time state mirroring with configurable history buffer. Forward predictive simulation (kinematics + power model). Anomaly detection with baseline learning (z-score). Battery life estimation.

### Navigation (`src/navigation/path.py`) — 9.1K chars
Pose and Waypoint dataclasses with dead-reckoning position estimation, sequential waypoint following with per-waypoint arrival tolerance, desired heading/speed computation, and simple potential-field obstacle avoidance.

## Architecture

```
NL Intent → Intent Parser → IR → Bytecode → VM Execution
                                           ↓
         Wire Protocol ←→ Agent Comms ←→ Trust Engine
                                           ↓
              Safety System (4 tiers) ↔ Autonomy Levels (L0-L5)
                                           ↓
        Fleet Coordination ←→ Cooperative Perception ←→ Digital Twin
```

## vs SuperInstance/nexus-runtime

Covers the same architectural patterns as [nexus-runtime](https://github.com/SuperInstance/nexus-runtime) but generalized:
- Domain-agnostic (not just maritime)
- Integrated with Cocapn fleet protocol
- Designed for mask-locked inference chip offload
- Trust-aware autonomy gating throughout

## Next: Additional Modules

- `src/mission/` — mission planning, execution monitoring, contingency
- `src/energy/` — power management, solar/recharge, budget allocation
- `src/maintenance/` — predictive maintenance, diagnostic scheduling
- `src/security/` — byzantine fault detection, encryption, auth
- `src/learning/` — reinforcement learning, skill acquisition
- `src/explainability/` — XAI decision logging, audit trail
- `src/sensor/` — sensor health monitoring, calibration, fusion
- `src/comms/` — MQTT bridge, mesh networking, relay routing
- `src/data_pipeline/` — telemetry ingestion, compression, storage
- `src/config/` — runtime configuration, hot-reload, schema validation
- `src/simulation/` — physics simulation, Monte Carlo scenarios
- `src/swarm/` — swarm behaviors, emergence detection, consensus
- `src/autonomy/` — adaptive autonomy, self-healing, reflex override
- `src/hardware/` — 11+ platform profiles (ESP32, Jetson, Pi, etc.)

## Related repos

Part of the Cocapn fleet. Siblings with a real conceptual overlap:

- **[Edge-Native](https://github.com/SuperInstance/Edge-Native)** — the
  specification repository; this runtime implements its opcodes, wire frames,
  and trust/safety model on the Jetson side.
- **[nexus-runtime](https://github.com/SuperInstance/nexus-runtime)** — the
  companion runtime (compared above); shares the same architectural patterns.
- **[edge-relay-agent](https://github.com/SuperInstance/edge-relay-agent)** —
  cloud↔edge relay; models the asymmetric message flow this runtime consumes.
- **[marine-gpu-edge](https://github.com/SuperInstance/marine-gpu-edge)** —
  GPU-accelerated counterpart to this runtime's `perception/fusion.py`
  sensor-fusion module.
- **[edge-equipment-catalog](https://github.com/SuperInstance/edge-equipment-catalog)**
  — profiles the ESP32 / Jetson hardware this runtime deploys to.

## License

MIT — DiGennaro et al. (SuperInstance & Lucineer)
