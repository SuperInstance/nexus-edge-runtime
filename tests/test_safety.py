"""Tests for src/safety/system.py — Tier-4 bytecode validation.

These pin down the agreement between the safety validator and the VM's
actual opcode set: the validator must reject every opcode the VM cannot
execute (it previously accepted 0x21, which the VM rejects at runtime).
"""
import pytest

from core.vm import BytecodeVM, Opcode, VMError
from safety.system import SafetySystem, SafetyConfig, SafetyTier


def _instr(op):
    """One 8-byte instruction with the given opcode byte."""
    return bytes([op, 0, 0, 0, 0, 0, 0, 0])


class TestBytecodeValidation:
    def _validator(self):
        return SafetySystem(SafetyConfig()).tier4

    def test_accepts_every_real_vm_opcode(self):
        """Every value in the VM's Opcode enum must pass validation."""
        v = self._validator()
        for op in Opcode:
            ok, errs = v.validate_bytecode(_instr(op.value))
            assert ok, f"0x{op.value:02X} ({op.name}) wrongly rejected: {errs}"

    def test_rejects_opcode_above_range(self):
        """0x21 is not a real opcode: validator must reject it.

        Regression test: the validator previously whitelisted 0x21 even
        though the VM raises VMError on it.
        """
        v = self._validator()
        ok, errs = v.validate_bytecode(_instr(0x21))
        assert not ok
        assert errs and errs[0].tier == SafetyTier.APPLICATION

    def test_validator_and_vm_agree_on_0x21(self):
        """The validator's verdict must match the VM's runtime behaviour."""
        v = self._validator()
        ok, _ = v.validate_bytecode(_instr(0x21))

        vm = BytecodeVM()
        vm.load(_instr(0x21))
        with pytest.raises(VMError):
            vm.run(max_cycles=5)

        # If the validator passes bytecode the VM rejects, a program can be
        # deployed as "safe" and then crash the agent at runtime.
        assert not ok, "validator accepted bytecode the VM cannot execute"

    def test_rejects_clearly_invalid_high_opcode(self):
        v = self._validator()
        ok, errs = v.validate_bytecode(_instr(0xFF))
        assert not ok
        assert errs and errs[0].kind == "invalid_opcode"

    def test_rejects_empty_bytecode(self):
        ok, errs = self._validator().validate_bytecode(b"")
        assert not ok

    def test_rejects_misaligned_bytecode(self):
        ok, errs = self._validator().validate_bytecode(b"\x00" * 7)
        assert not ok

    def test_rejects_oversized_bytecode(self):
        cfg = SafetyConfig(max_bytecode_size=16)
        ok, errs = SafetySystem(cfg).tier4.validate_bytecode(b"\x00" * 64)
        assert not ok
