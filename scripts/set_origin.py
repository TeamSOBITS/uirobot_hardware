#!/usr/bin/env python3

import argparse
import os
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is required. Install it with: sudo apt install python3-serial", file=sys.stderr)
    sys.exit(1)


HEADER = 0xAA
FOOTER = 0xCC
PACKET_SIZE = 16


def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_origin_packet(node_id: int) -> bytes:
    payload = bytes([
        node_id, 0xA1, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ])
    crc = modbus_crc16(payload)
    return bytes([HEADER]) + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF, FOOTER])


def build_expected_ack(node_id: int) -> bytes:
    payload = bytes([
        node_id, 0x21, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ])
    crc = modbus_crc16(payload)
    return bytes([HEADER]) + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF, FOOTER])


def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def main() -> int:
    parser = argparse.ArgumentParser(description="Set the current UIM342 position as origin via UIM2513 gateway")
    parser.add_argument("--settle", type=float, default=0.1, help="Seconds to wait before reading ACK")
    args = parser.parse_args()

    port = str(os.environ.get('UM_PORT'))
    baud = 57600
    device_id = 5

    tx = build_origin_packet(device_id)
    expected = build_expected_ack(device_id)

    print("Sending:", hexdump(tx))
    print("Expect :", hexdump(expected))

    with serial.Serial(port, baud, timeout=1) as ser:
        ser.reset_input_buffer()
        ser.write(tx)
        time.sleep(args.settle)
        rx = ser.read(PACKET_SIZE)

    print("Received:", hexdump(rx))

    if rx != expected:
        print("Set origin failed: ACK mismatch", file=sys.stderr)
        return 1

    print("Origin set successfully")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
