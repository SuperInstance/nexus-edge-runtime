"""Tests for src/core/vm.py"""
import struct
import pytest
from core.vm import BytecodeVM, Assembler, Validator, Opcode, VMError


def encode_instr(opcode, arg8=0, arg16=0, imm32=0):
    return struct.pack('<BBHi', opcode, arg8, arg16, imm32 & 0xFFFFFFFF)


class TestAssembler:
    def test_assemble_push_f32(self):
        asm = Assembler()
        bc = asm.assemble("PUSH_F32 3.14")
        assert len(bc) == 8
        op, arg8, arg16, imm32 = struct.unpack('<BBHi', bc)
        assert op == Opcode.PUSH_F32
        assert pytest.approx(struct.unpack('<f', struct.pack('<i', imm32))[0]) == 3.14

    def test_assemble_label_jump(self):
        asm = Assembler()
        source = """
        PUSH_F32 1.0
        start:
        PUSH_F32 2.0
        ADD_F
        JUMP start
        """
        bc = asm.assemble(source)
        # 4 instructions: push, push, add, jump
        assert len(bc) == 32
        lines = asm.disassemble(bc).splitlines()
        assert "JUMP @1" in lines[3]


class TestStackOps:
    def test_push_pop(self):
        asm = Assembler()
        vm = BytecodeVM()
        vm.load(asm.assemble("PUSH_F32 7.0\nPOP"))
        vm.run(max_cycles=2)
        assert len(vm.state.stack) == 0
        assert vm.state.cycles == 2

    def test_dup(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble("PUSH_F32 5.0\nDUP"))
        vm.run(max_cycles=2)
        assert vm.state.stack == [5.0, 5.0]

    def test_swap(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble("PUSH_F32 1.0\nPUSH_F32 2.0\nSWAP"))
        vm.run(max_cycles=3)
        assert vm.state.stack == [2.0, 1.0]

    def test_rot(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble(
            "PUSH_F32 1.0\nPUSH_F32 2.0\nPUSH_F32 3.0\nROT"))
        vm.run(max_cycles=4)
        assert vm.state.stack == [2.0, 3.0, 1.0]

    def test_rot_underflow(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble("PUSH_F32 1.0\nPUSH_F32 2.0\nROT"))
        with pytest.raises(VMError, match="ROT needs 3 items"):
            vm.run(max_cycles=3)


class TestArithmetic:
    def test_add(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble(
            "PUSH_F32 2.0\nPUSH_F32 3.0\nADD_F"))
        vm.run(max_cycles=3)
        assert vm.state.stack == [5.0]

    def test_sub_mul(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble(
            "PUSH_F32 4.0\nPUSH_F32 3.0\nSUB_F\nPUSH_F32 2.0\nMUL_F"))
        vm.run(max_cycles=5)
        # (4 - 3) * 2 = 2
        assert vm.state.stack == [2.0]

    def test_div(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble(
            "PUSH_F32 5.0\nPUSH_F32 2.0\nDIV_F"))
        vm.run(max_cycles=3)
        assert vm.state.stack == [pytest.approx(2.5)]

    def test_div_by_zero(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble(
            "PUSH_F32 5.0\nPUSH_F32 0.0\nDIV_F"))
        vm.run(max_cycles=3)
        assert vm.state.stack == [0.0]

    def test_neg_abs(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble(
            "PUSH_F32 -4.5\nNEG_F\nPUSH_F32 -7.0\nABS_F"))
        vm.run(max_cycles=4)
        assert vm.state.stack == [pytest.approx(4.5), pytest.approx(7.0)]

    def test_min_max(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble(
            "PUSH_F32 5.0\nPUSH_F32 3.0\nMIN_F\n"
            "PUSH_F32 5.0\nPUSH_F32 3.0\nMAX_F"))
        vm.run(max_cycles=6)
        assert vm.state.stack == [3.0, 5.0]

    def test_clamp(self):
        vm = BytecodeVM()
        # CLAMP_F uses imm32 as lo and arg8 as hi; assembler only sets imm32,
        # so build manually: clamp(v, 10.0, 20.0)
        vm.load(
            encode_instr(Opcode.PUSH_F32, imm32=struct.unpack('<i', struct.pack('<f', 25.0))[0]) +
            encode_instr(Opcode.CLAMP_F, arg8=20,
                         imm32=struct.unpack('<i', struct.pack('<f', 10.0))[0])
        )
        vm.run(max_cycles=2)
        assert vm.state.stack == [pytest.approx(20.0)]


class TestCompare:
    def test_eq_lt_gt(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble(
            "PUSH_F32 2.0\nPUSH_F32 2.0\nEQ_F\n"
            "PUSH_F32 1.0\nPUSH_F32 2.0\nLT_F\n"
            "PUSH_F32 3.0\nPUSH_F32 2.0\nGT_F"))
        vm.run(max_cycles=9)
        assert vm.state.stack == [1.0, 1.0, 1.0]

    def test_lte_gte(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble(
            "PUSH_F32 2.0\nPUSH_F32 2.0\nLTE_F\n"
            "PUSH_F32 2.0\nPUSH_F32 2.0\nGTE_F\n"
            "PUSH_F32 3.0\nPUSH_F32 2.0\nLTE_F"))
        vm.run(max_cycles=9)
        assert vm.state.stack == [1.0, 1.0, 0.0]


class TestLogic:
    def test_and_or_xor(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble(
            "PUSH_F32 6.0\nPUSH_F32 3.0\nAND_B\n"   # 6 & 3 = 2
            "PUSH_F32 5.0\nPUSH_F32 2.0\nOR_B\n"    # 5 | 2 = 7
            "PUSH_F32 7.0\nPUSH_F32 2.0\nXOR_B"))   # 7 ^ 2 = 5
        vm.run(max_cycles=9)
        assert vm.state.stack == [2.0, 7.0, 5.0]

    def test_not(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble("PUSH_F32 0.0\nNOT_B"))
        vm.run(max_cycles=2)
        # ~0 & 0xFF = 255
        assert vm.state.stack == [255.0]


class TestIO:
    def test_read_pin(self):
        vm = BytecodeVM()
        vm.pin_state[3] = 42.0
        vm.load(Assembler().assemble("READ_PIN 3"))
        vm.run(max_cycles=1)
        assert vm.state.stack == [42.0]
        assert vm.state.regs[16 + 3] == 42.0

    def test_write_pin(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble("PUSH_F32 99.0\nWRITE_PIN 4"))
        vm.run(max_cycles=2)
        assert vm.pin_state[4] == 99.0
        assert ("WRITE_PIN", (4, 99.0)) in vm.state.output_log


class TestControlFlow:
    def test_jump(self):
        vm = BytecodeVM()
        # Instruction 0: JUMP to instruction 2
        # Instruction 1: PUSH_F32 1.0 (skipped)
        # Instruction 2: PUSH_F32 2.0
        vm.load(Assembler().assemble("JUMP 2\nPUSH_F32 1.0\nPUSH_F32 2.0"))
        vm.step()  # execute JUMP
        assert vm.state.pc == 2 * 8

    def test_jump_if_false_taken(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble(
            "PUSH_F32 0.0\nJUMP_IF_FALSE 2\nPUSH_F32 1.0\nPUSH_F32 2.0"))
        vm.run(max_cycles=2)
        assert vm.state.pc == 2 * 8

    def test_jump_if_false_not_taken(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble(
            "PUSH_F32 1.0\nJUMP_IF_FALSE 2\nPUSH_F32 1.0\nPUSH_F32 2.0"))
        vm.run(max_cycles=2)
        assert vm.state.pc == 2 * 8  # fell through

    def test_jump_if_true_taken(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble(
            "PUSH_F32 1.0\nJUMP_IF_TRUE 2\nPUSH_F32 1.0\nPUSH_F32 2.0"))
        vm.run(max_cycles=2)
        assert vm.state.pc == 2 * 8


class TestVMState:
    def test_reset(self):
        vm = BytecodeVM()
        vm.load(Assembler().assemble("PUSH_F32 5.0"))
        vm.run(max_cycles=1)
        vm.reset()
        assert vm.state.stack == []
        assert vm.state.cycles == 0
        assert vm.pin_state == {}

    def test_stack_overflow(self):
        vm = BytecodeVM()
        source = "\n".join(["PUSH_F32 1.0"] * (BytecodeVM.MAX_STACK + 1))
        vm.load(Assembler().assemble(source))
        with pytest.raises(VMError, match="Stack overflow"):
            vm.run()


class TestValidator:
    def test_valid(self):
        bc = Assembler().assemble("PUSH_F32 1.0\nADD_F")
        ok, errs = Validator().validate(bc)
        assert ok is True
        assert errs == []

    def test_empty(self):
        ok, errs = Validator().validate(b"")
        assert ok is False
        assert "Empty bytecode" in errs

    def test_misaligned(self):
        ok, errs = Validator().validate(b"\x00\x00")
        assert ok is False
        assert any("multiple of 8" in e for e in errs)

    def test_invalid_opcode(self):
        bc = b"\xFF\x00\x00\x00\x00\x00\x00\x00"
        ok, errs = Validator().validate(bc)
        assert ok is False
        assert any("Invalid opcode" in e for e in errs)
