#!/usr/bin/env python3
"""
MAVLink receiver for Raspberry Pi

Pixhawk 6C TELEM 포트 → RPi UART로 MAVLink 데이터를 수신하여 출력.
pymavlink 라이브러리 사용.

사용법:
  pip install pymavlink
  python mavlink_receiver.py --port /dev/ttyAMA0 --baud 57600

연결:
  Pixhawk TELEM1 TX → RPi RX (GPIO 15)
  Pixhawk TELEM1 RX → RPi TX (GPIO 14)
  GND → GND
  주의: Pixhawk는 3.3V 레벨이므로 RPi와 직결 가능 (5V 레벨 변환 불필요)
"""

import argparse
import time
import sys

try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink이 설치되지 않았습니다: pip install pymavlink")
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="MAVLink Receiver for RPi")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Serial port")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate")
    return parser.parse_args()


def main():
    args = parse_args()

    print(f"[*] Connecting to {args.port} at {args.baud} baud...")
    mav = mavutil.mavlink_connection(args.port, baud=args.baud)

    print("[*] Waiting for heartbeat...")
    mav.wait_heartbeat()
    print(f"[+] Heartbeat received (system {mav.target_system}, component {mav.target_component})")

    print("[*] Listening for messages (Ctrl+C to stop)...\n")

    try:
        while True:
            msg = mav.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":
                print(f"[HEARTBEAT] type={msg.type} mode={msg.base_mode} state={msg.system_status}")

            elif msg_type == "ATTITUDE":
                print(f"[ATTITUDE] roll={msg.roll:.3f} pitch={msg.pitch:.3f} yaw={msg.yaw:.3f} "
                      f"(p={msg.rollspeed:.3f} q={msg.pitchspeed:.3f} r={msg.yawspeed:.3f})")

            elif msg_type == "GPS_RAW_INT":
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1000.0
                print(f"[GPS] fix={msg.fix_type} lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}m "
                      f"sv={msg.satellites_visible}")

            elif msg_type == "SYS_STATUS":
                print(f"[SYS] battery={msg.voltage_battery}mV "
                      f"remaining={msg.battery_remaining}% "
                      f"load={msg.load/10.0:.1f}%")

            else:
                # 커스텀 또는 알 수 없는 메시지
                print(f"[{msg_type}] {msg}")

    except KeyboardInterrupt:
        print("\n[*] Stopped.")


if __name__ == "__main__":
    main()
