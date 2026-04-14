#!/usr/bin/env python3
import serial
import time
import os


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
    port = str(os.environ.get('UM_PORT'))
    baud = 57600
    device_id = 5

    expected_reply = bytes([
        0xAA,
        0x05, 0x10, 0x03,
        0x05, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xF3, 0x91,
        0xCC
    ])

    tx = build_packet(device_id)

    print("Sending:", hexdump(tx))
    print("Expect :", hexdump(expected_reply))

    with serial.Serial(port, baud, timeout=1) as ser:
        ser.reset_input_buffer()
        ser.write(tx)

        time.sleep(0.1)

        rx = ser.read(16)
        print("Received:", hexdump(rx))

        if rx == expected_reply:
            print("✅ Brake released successfully (ACK matched)")
        else:
            print("⚠️ Reply mismatch")


if __name__ == "__main__":
    main()
