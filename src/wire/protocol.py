"""Nexus Edge Wire Protocol — length-prefixed framed communication.

Frame format:
  [0xAA][0x55][SRC:1B][DST:1B][TYPE:1B][SEQ:2B][LEN:2B][PAYLOAD:N][CRC16:2B]

Simple length-prefixed framing with CRC-16/CCITT integrity.
No COBS needed — length field handles delineation.
"""
import struct, enum
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


class MessageType(enum.IntEnum):
    HEARTBEAT = 0x01
    TELEMETRY = 0x02
    COMMAND = 0x03
    BYTECODE = 0x04
    TRUST_UPDATE = 0x05
    STATUS = 0x06
    ERROR = 0x07
    DATA = 0x10
    TRUST_QUERY = 0x20
    TASK_ASSIGN = 0x30
    TASK_RESULT = 0x31
    CAP_EXCHANGE = 0x40
    SYNC_REQ = 0x50
    SYNC_RESP = 0x51


PREAMBLE = bytes([0xAA, 0x55])
HEADER_SIZE = 11
CRC_SIZE = 2
MAX_PAYLOAD = 512


@dataclass
class Message:
    msg_type: MessageType = MessageType.HEARTBEAT
    source: int = 0
    destination: int = 0
    sequence: int = 0
    payload: bytes = b""
    timestamp: float = 0.0


class CRC16:
    POLY = 0x1021
    INIT = 0xFFFF

    @staticmethod
    def compute(data: bytes) -> int:
        crc = CRC16.INIT
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ CRC16.POLY
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    @staticmethod
    def verify(data: bytes, expected: int) -> bool:
        return CRC16.compute(data) == expected


class WireProtocol:
    """Length-prefixed wire protocol with CRC-16."""

    def __init__(self, node_id: int = 0):
        self.node_id = node_id
        self.seq_counter = 0

    def encode_frame(self, msg: Message) -> bytes:
        payload = msg.payload[:MAX_PAYLOAD]
        body = struct.pack('<BBBH H',
            msg.source or self.node_id,
            msg.destination,
            msg.msg_type.value,
            msg.sequence or self.seq_counter,
            len(payload))
        body += payload
        crc = CRC16.compute(body)
        self.seq_counter = (self.seq_counter + 1) & 0xFFFF
        return PREAMBLE + body + struct.pack('<H', crc)

    def decode_frame(self, data: bytes) -> Optional[Message]:
        if len(data) < HEADER_SIZE + CRC_SIZE:
            return None
        if data[:2] != PREAMBLE:
            return None
        body = data[2:-CRC_SIZE]
        crc_received = struct.unpack_from('<H', data, len(data) - CRC_SIZE)[0]
        if not CRC16.verify(body, crc_received):
            return None
        if len(body) < 9:
            return None
        src = body[0]
        dst = body[1]
        msg_type = MessageType(body[2])
        seq = struct.unpack_from('<H', body, 3)[0]
        payload_len = struct.unpack_from('<H', body, 5)[0]
        payload = body[9:9 + payload_len]
        return Message(msg_type, src, dst, seq, payload)

    def build_heartbeat(self, dst: int = 0) -> bytes:
        return self.encode_frame(Message(MessageType.HEARTBEAT, self.node_id, dst))


class FrameParser:
    """Stream-level frame parser with buffering."""

    def __init__(self, protocol: WireProtocol):
        self.protocol = protocol
        self.buffer = bytearray()

    def feed(self, data: bytes) -> List[Message]:
        self.buffer.extend(data)
        messages = []
        while True:
            idx = self.buffer.find(PREAMBLE)
            if idx < 0:
                if len(self.buffer) > 100:
                    self.buffer.clear()
                break
            if idx > 0:
                del self.buffer[:idx]

            if len(self.buffer) < HEADER_SIZE + CRC_SIZE:
                break

            payload_len = struct.unpack_from('<H', self.buffer, 7)[0]
            frame_len = 2 + 7 + payload_len + 2
            if len(self.buffer) < frame_len:
                break

            frame = bytes(self.buffer[:frame_len])
            msg = self.protocol.decode_frame(frame)
            if msg:
                messages.append(msg)
            del self.buffer[:frame_len]

        return messages


def demo():
    print("=== Nexus Edge Wire Protocol ===\n")

    # CRC
    print("--- CRC-16/CCITT ---")
    test = b"Hello, Nexus!"
    crc = CRC16.compute(test)
    print(f"  CRC16 = 0x{crc:04X}, verify={CRC16.verify(test, crc)}")

    # Single frame
    print("\n--- Frame Encode/Decode ---")
    wp = WireProtocol(node_id=1)
    msg = Message(MessageType.TELEMETRY, 1, 2, 0, b"temp=25.3,depth=2.1")
    frame = wp.encode_frame(msg)
    print(f"  Frame: {len(frame)} bytes")
    decoded = wp.decode_frame(frame)
    print(f"  Decoded: {decoded.msg_type.name} src={decoded.source} dst={decoded.destination}")
    print(f"  Payload: {decoded.payload.decode()}")

    # Stream parsing
    print("\n--- Stream Parser ---")
    parser = FrameParser(wp)
    stream = bytearray()
    for i in range(5):
        m = Message(MessageType(i % 4 + 1), 1, i + 1, payload=f"msg_{i}".encode())
        stream.extend(wp.encode_frame(m))
    msgs = parser.feed(bytes(stream))
    print(f"  Sent 5, received {len(msgs)}")
    for m in msgs:
        print(f"  {m.msg_type.name} #{m.source}→#{m.destination}: {m.payload.decode()}")

    # Partial frames
    print("\n--- Partial Frame Handling ---")
    parser2 = FrameParser(wp)
    half = stream[:len(stream)//2]
    rest = stream[len(stream)//2:]
    msgs1 = parser2.feed(half)
    msgs2 = parser2.feed(rest)
    print(f"  First half: {len(msgs1)} msgs, second half: {len(msgs2)} msgs, total: {len(msgs1)+len(msgs2)}")

    # Throughput
    print(f"\n--- Efficiency ---")
    overhead = HEADER_SIZE + CRC_SIZE - 2  # minus preamble (shared)
    for pl in [10, 50, 100, 256, 512]:
        eff = pl / (pl + overhead) * 100
        print(f"  {pl:3d}B payload: {eff:.1f}% efficiency ({pl+overhead}B frame)")


if __name__ == "__main__":
    demo()
