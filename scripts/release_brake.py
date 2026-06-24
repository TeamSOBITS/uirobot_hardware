#!/usr/bin/env python3
import serial
import time
import os
import argparse  # 💡 引数を処理するために追加


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
    # 💡 最小限の変更で引数を受け取れるように設定
    parser = argparse.ArgumentParser(description="Release Uirobot Brake")
    parser.add_argument('--port', type=str, default=os.environ.get('UM_PORT'), help="Serial port")
    parser.add_argument('--baud', type=int, default=115200, help="Baud rate")
    parser.add_argument('--id', type=int, default=5, help="Device (Node) ID")
    args = parser.parse_known_args()[0] # 他の未知の引数があってもエラーにしない

    # 💡 パースした値を代入（型を一致させています）
    port = str(args.port) if args.port else None
    baud = args.baud
    device_id = args.id

    # 🚨 ポートが指定されていない場合のセーフティ
    if not port or port == "None":
        print("❌ Error: Serial port is not specified. Please set UM_PORT or pass --port.")
        return

    expected_reply = bytes([
        0xAA,
        device_id, 0x10, 0x03,  # 💡 デバイスIDが変わっても動くようにマジックナンバー0x05をdevice_idに変更
        0x05, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xF3, 0x91,             # ⚠️ 注意: IDが変わるとCRCも変わるため、後述のチェック方法を修正
        0xCC
    ])

    tx = build_packet(device_id)

    print("Sending:", hexdump(tx))
    # 💡 expected_replyのCRCは固定値 [0xF3, 0x91] のままなので、期待値の表示はtxのパケットを元に構築する、
    # あるいは返答の成否判定を「返ってきたデータの長さとコマンド(0x10)」で行うのがより正確です。
    # ここではコードの形を崩さないよう、そのまま出力します。
    print("Expect (Static):", hexdump(expected_reply))

    with serial.Serial(port, baud, timeout=1) as ser:
        ser.reset_input_buffer()
        ser.write(tx)

        time.sleep(0.1)

        rx = ser.read(16)
        print("Received:", hexdump(rx))

        # 💡 ID変更に対応するため、完全一致(==)ではなく「長さが16バイト」かつ「ヘッダーとID、コマンドの一致」で判定
        if len(rx) == 16 and rx[0] == 0xAA and rx[1] == device_id and rx[2] == 0x10:
            print(f"✅ Brake released successfully (ACK matched for ID {device_id})")
        else:
            print("⚠️ Reply mismatch")


if __name__ == "__main__":
    main()