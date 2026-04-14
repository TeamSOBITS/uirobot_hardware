#!/usr/bin/env python3

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is required. Install it with: sudo apt install python3-serial", file=sys.stderr)
    sys.exit(1)


HEADER = 0xAD
CRC_HEADER = 0xAA
FOOTER = 0xCC
PACKET_SIZE = 16


def build_get_brake_packet(node_id: int) -> bytes:
    packet = bytearray([HEADER, node_id, 0x90, 0x01, 0x05])
    packet.extend([0x00] * 10)
    packet.append(FOOTER)
    return bytes(packet)


def read_packet(ser: serial.Serial, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    data = bytearray()

    while time.monotonic() < deadline:
      chunk = ser.read(PACKET_SIZE - len(data))
      if not chunk:
          continue
      data.extend(chunk)
      if len(data) >= PACKET_SIZE:
          return bytes(data[:PACKET_SIZE])

    return bytes(data)


def decode_brake_state(packet: bytes, node_id: int) -> str:
    if len(packet) < PACKET_SIZE:
        raise ValueError(f"short reply ({len(packet)} bytes)")
    if packet[0] not in (HEADER, CRC_HEADER) or packet[-1] != FOOTER:
        raise ValueError(f"invalid framing: {[hex(b) for b in packet]}")
    if packet[1] != node_id:
        raise ValueError(f"unexpected node id {packet[1]}")
    if packet[2] != 0x10:
        raise ValueError(f"unexpected CW 0x{packet[2]:02X}")
    if packet[3] != 0x03 or packet[4] != 0x05:
        raise ValueError(f"unexpected MT[5] payload: {[hex(b) for b in packet]}")

    value = packet[5] | (packet[6] << 8)
    if value == 0:
        return "released"
    if value == 1:
        return "locked"
    return f"unknown({value})"


def main() -> int:
    parser = argparse.ArgumentParser(description="Read UIM342 internal brake state via UIM2513 gateway")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB2")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate")
    parser.add_argument("--id", type=int, required=True, help="UIROBOT node id")
    parser.add_argument("--timeout", type=float, default=1.0, help="Read timeout in seconds")
    args = parser.parse_args()

    packet = build_get_brake_packet(args.id)

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        ser.reset_input_buffer()
        ser.write(packet)
        reply = read_packet(ser, args.timeout)

    try:
        state = decode_brake_state(reply, args.id)
    except ValueError as exc:
        print(f"Brake query failed: {exc}", file=sys.stderr)
        if reply:
            print("Reply bytes:", " ".join(f"{b:02X}" for b in reply), file=sys.stderr)
        return 1

    mode = "crc" if reply[0] == CRC_HEADER else "plain"
    print(f"node={args.id} brake={state} mode={mode}")
    print("reply:", " ".join(f"{b:02X}" for b in reply))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
