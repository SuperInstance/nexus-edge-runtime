# nexus-edge-runtime

Edge runtime for autonomous agents in the Cocapn fleet.

## Core Modules

### Bytecode VM (`src/core/vm.py`)
- 32-opcode stack-based VM for deterministic edge control
- 8-byte instruction format: `[opcode:u8][arg8:u8][arg16:u16][imm32:i32]`
- 32 registers (16 GP + 16 IO-mapped), 64KB memory, 1024-deep stack
- Assembler, disassembler, and bytecode validator
- Designed for ESP32-S3 deployment and Jetson supervision

### INCREMENTS Trust Engine (`src/trust/engine.py`)
- Multi-dimensional trust: history (EMA), capability, latency, consistency
- Composite score with configurable weights
- Autonomy levels L0-L5 (Manual → Full) based on trust score
- Transitive trust propagation through fleet graph
- Trust decay without interaction

### Wire Protocol (`src/wire/protocol.py`)
- Length-prefixed framed communication with CRC-16/CCITT
- Frame: `[PREAMBLE:2B][SRC:1B][DST:1B][TYPE:1B][SEQ:2B][LEN:2B][PAYLOAD:N][CRC16:2B]`
- Stream-level parser with partial frame buffering
- 14 message types: heartbeat, telemetry, command, bytecode, trust, tasks, sync

### Safety System (`src/safety/system.py`)
- 4-tier defense-in-depth architecture
  - Tier 1 HARDWARE: Kill switch, watchdog IC (<1us response)
  - Tier 2 FIRMWARE: Stack canary, safe-state outputs (<1ms)
  - Tier 3 SUPERVISORY: Heartbeat monitoring, state machine (<100ms)
  - Tier 4 APPLICATION: Bytecode validation, trust-gated autonomy (<1s)
- Emergency stop with safe-state output mapping
- Autonomy level gating by trust score

### Intent Compiler (`src/reflex/compiler.py`)
- Natural language intent → bytecode compilation pipeline
- Intent parser: action + target + value + unit + condition
- IR emitter: sensor read → compare → actuate patterns
- Supports: maintain, navigate, monitor (conditional), alert
- Example: "maintain depth at 2m" → control loop bytecode

## Architecture

```
NL Intent → Intent Parser → IR → Bytecode Emitter → VM Execution
                                                        ↓
                    Wire Protocol ←→ Agent Communication ←→ Trust Engine
                                                        ↓
                    Safety System (4 tiers) ←→ Autonomy Levels
```

## Compared to nexus-runtime

This repo covers the same architectural patterns as [SuperInstance/nexus-runtime](https://github.com/SuperInstance/nexus-runtime)
but generalized beyond maritime robotics for the Cocapn fleet:
- Domain-agnostic (robotics, IoT, industrial, aerial, marine)
- Integrated with Cocapn fleet protocol
- Designed for mask-locked inference chip offload
- Trust-aware autonomy gating

## License

MIT
