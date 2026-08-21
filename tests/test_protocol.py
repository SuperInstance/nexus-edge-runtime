"""Tests for src/wire/protocol.py"""
import struct
import pytest
from wire.protocol import (
    Message, MessageType, CRC16, WireProtocol, FrameParser,
    PREAMBLE, HEADER_SIZE, CRC_SIZE, MAX_PAYLOAD,
)


class TestCRC16:
    def test_compute_known(self):
        """CRC-16/CCITT of a known string is deterministic."""
        data = b"Hello, Nexus!"
        crc = CRC16.compute(data)
        assert crc == 0x5CAF
        assert CRC16.verify(data, crc) is True

    def test_verify_wrong_fails(self):
        data = b"Hello, Nexus!"
        assert CRC16.verify(data, 0x0000) is False

    def test_empty(self):
        assert CRC16.compute(b"") == CRC16.INIT


class TestWireProtocol:
    def test_round_trip(self):
        wp = WireProtocol(node_id=1)
        msg = Message(MessageType.TELEMETRY, 1, 2, 5, b"temp=25.3")
        frame = wp.encode_frame(msg)
        decoded = wp.decode_frame(frame)
        assert decoded is not None
        assert decoded.msg_type == MessageType.TELEMETRY
        assert decoded.source == 1
        assert decoded.destination == 2
        assert decoded.sequence == 5
        assert decoded.payload == b"temp=25.3"

    def test_sequence_counter_auto_increments(self):
        wp = WireProtocol(node_id=1)
        m1 = wp.decode_frame(wp.encode_frame(Message(MessageType.HEARTBEAT, 1, 0)))
        m2 = wp.decode_frame(wp.encode_frame(Message(MessageType.HEARTBEAT, 1, 0)))
        assert m2.sequence == (m1.sequence + 1) & 0xFFFF

    def test_build_heartbeat(self):
        wp = WireProtocol(node_id=7)
        frame = wp.build_heartbeat(dst=9)
        msg = wp.decode_frame(frame)
        assert msg.msg_type == MessageType.HEARTBEAT
        assert msg.source == 7
        assert msg.destination == 9

    def test_bad_preamble_rejected(self):
        wp = WireProtocol()
        frame = wp.encode_frame(Message(MessageType.STATUS, 1, 2, 0, b"ok"))
        bad = b"\x00\x00" + frame[2:]
        assert wp.decode_frame(bad) is None

    def test_bad_crc_rejected(self):
        wp = WireProtocol()
        frame = bytearray(wp.encode_frame(Message(MessageType.STATUS, 1, 2, 0, b"ok")))
        # Corrupt a payload byte
        frame[-3] ^= 0xFF
        assert wp.decode_frame(bytes(frame)) is None

    def test_truncated_rejected(self):
        wp = WireProtocol()
        frame = wp.encode_frame(Message(MessageType.STATUS, 1, 2, 0, b"ok"))
        assert wp.decode_frame(frame[:HEADER_SIZE]) is None

    def test_max_payload_truncated(self):
        wp = WireProtocol()
        payload = b"x" * (MAX_PAYLOAD + 50)
        frame = wp.encode_frame(Message(MessageType.DATA, 1, 2, 0, payload))
        msg = wp.decode_frame(frame)
        assert len(msg.payload) == MAX_PAYLOAD

    def test_empty_payload_round_trip(self):
        wp = WireProtocol(node_id=3)
        frame = wp.encode_frame(Message(MessageType.HEARTBEAT, 3, 0, 7))
        msg = wp.decode_frame(frame)
        assert msg is not None
        assert msg.payload == b""
        assert msg.sequence == 7

    def test_message_type_values(self):
        assert MessageType.HEARTBEAT == 0x01
        assert MessageType.TELEMETRY == 0x02
        assert MessageType.COMMAND == 0x03
        assert MessageType.BYTECODE == 0x04


class TestFrameParser:
    def test_single_complete_frame(self):
        wp = WireProtocol(node_id=1)
        parser = FrameParser(wp)
        frame = wp.encode_frame(Message(MessageType.COMMAND, 1, 2, 10, b"go"))
        msgs = parser.feed(frame)
        assert len(msgs) == 1
        assert msgs[0].payload == b"go"

    def test_multiple_frames(self):
        wp = WireProtocol(node_id=1)
        parser = FrameParser(wp)
        stream = bytearray()
        for i in range(3):
            m = Message(MessageType(i + 1), 1, i + 1, i, f"m{i}".encode())
            stream.extend(wp.encode_frame(m))
        msgs = parser.feed(bytes(stream))
        assert len(msgs) == 3
        assert [m.payload for m in msgs] == [b"m0", b"m1", b"m2"]

    def test_partial_frame_reassembled(self):
        wp = WireProtocol(node_id=1)
        parser = FrameParser(wp)
        frame = wp.encode_frame(Message(MessageType.TELEMETRY, 1, 2, 0, b"partial"))
        mid = len(frame) // 2
        msgs1 = parser.feed(frame[:mid])
        msgs2 = parser.feed(frame[mid:])
        assert msgs1 == []
        assert len(msgs2) == 1
        assert msgs2[0].payload == b"partial"

    def test_partial_frame_in_three_chunks(self):
        wp = WireProtocol(node_id=1)
        parser = FrameParser(wp)
        frame = wp.encode_frame(Message(MessageType.TELEMETRY, 1, 2, 0, b"chunks"))
        a = frame[:5]
        b = frame[5:12]
        c = frame[12:]
        assert parser.feed(a) == []
        assert parser.feed(b) == []
        msgs = parser.feed(c)
        assert len(msgs) == 1
        assert msgs[0].payload == b"chunks"

    def test_leading_garbage_skipped(self):
        wp = WireProtocol(node_id=1)
        parser = FrameParser(wp)
        frame = wp.encode_frame(Message(MessageType.STATUS, 1, 2, 0, b"ok"))
        garbage = b"\xDE\xAD\xBE\xEF" + frame
        msgs = parser.feed(garbage)
        assert len(msgs) == 1
        assert msgs[0].payload == b"ok"

    def test_partial_and_complete_together(self):
        wp = WireProtocol(node_id=1)
        parser = FrameParser(wp)
        f1 = wp.encode_frame(Message(MessageType.STATUS, 1, 2, 0, b"first"))
        f2 = wp.encode_frame(Message(MessageType.STATUS, 1, 2, 0, b"second"))
        # feed first full frame + first half of second
        mid = len(f2) // 2
        msgs = parser.feed(f1 + f2[:mid])
        assert len(msgs) == 1
        assert msgs[0].payload == b"first"
        # feed remainder of second
        msgs2 = parser.feed(f2[mid:])
        assert len(msgs2) == 1
        assert msgs2[0].payload == b"second"
