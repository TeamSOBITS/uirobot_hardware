#!/usr/bin/env python3
import serial
import time
import os
import argparse


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


def build_packet(device_id: int) -> bytes:
    # MT[5] = 0 → release brake
    payload = bytes([
        device_id, 0x90, 0x03,
        0x05, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ])

    crc = modbus_crc16(payload)
    crc_lo = crc & 0xFF
    crc_hi = (crc >> 8) & 0xFF

    packet = bytes([0xAA]) + payload + bytes([crc_lo, crc_hi, 0xCC])
    return packet


def hexdump(data: bytes):
    return " ".join(f"{b:02X}" for b in data)


def main():
    # Configured to accept arguments with minimal changes.
    parser = argparse.ArgumentParser(description="Release Uirobot Brake")
    parser.add_argument('--port', type=str, default=os.environ.get('UM_PORT'), help="Serial port")
    parser.add_argument('--baud', type=int, default=115200, help="Baud rate")
    parser.add_argument('--id', type=int, default=5, help="Device (Node) ID")
    args = parser.parse_known_args()[0]  # tolerate unknown extra arguments

    # Assign the parsed value (matching the type).
    port = str(args.port) if args.port else None
    baud = args.baud
    device_id = args.id

    # Safety when no port is specified
    if not port or port == "None":
        print("❌ Error: Serial port is not specified. Please set UM_PORT or pass --port.")
        return

    expected_reply = bytes([
        0xAA,
        device_id, 0x10, 0x03,  # use device_id instead of a hard-coded 0x05
        0x05, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xF3, 0x91,             # CRC changes with the ID; see the reply check below
        0xCC
    ])

    tx = build_packet(device_id)

    print("Sending:", hexdump(tx))
    # expected_reply keeps the fixed CRC [0xF3, 0x91], so it is only printed
    # for reference; success is judged by length, header, ID and command below.
    print("Expect (Static):", hexdump(expected_reply))

    with serial.Serial(port, baud, timeout=1) as ser:
        ser.reset_input_buffer()
        ser.write(tx)

        time.sleep(0.1)

        rx = ser.read(16)
        print("Received:", hexdump(rx))

        # Judge by length (16 bytes), header, ID and command instead of an
        # exact match so the check works for any device ID.
        if len(rx) == 16 and rx[0] == 0xAA and rx[1] == device_id and rx[2] == 0x10:
            print(f"✅ Brake released successfully (ACK matched for ID {device_id})")
        else:
            print("⚠️ Reply mismatch")


if __name__ == "__main__":
    main()