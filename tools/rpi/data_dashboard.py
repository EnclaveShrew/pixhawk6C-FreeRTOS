#!/usr/bin/env python3
"""
Real-time data dashboard for Raspberry Pi

MAVLink 데이터를 수신하여 터미널에 실시간 대시보드로 표시.
pymavlink + curses 기반 (GUI 불필요).

사용법:
  python data_dashboard.py --port /dev/ttyAMA0 --baud 57600
"""

import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink이 설치되지 않았습니다: pip install pymavlink")
    sys.exit(1)

import math


class DashboardData:
    """MAVLink에서 수신한 최신 데이터"""
    def __init__(self):
        # Attitude
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0

        # GPS
        self.gps_fix = 0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.num_sv = 0

        # System
        self.battery_mv = 0
        self.battery_pct = -1
        self.cpu_load = 0.0

        # Controller
        self.ctrl_type = "UNKNOWN"
        self.ctrl_roll = 0.0
        self.ctrl_pitch = 0.0
        self.ctrl_yaw = 0.0

        # Status
        self.heartbeat_count = 0
        self.last_heartbeat = 0.0


def parse_args():
    parser = argparse.ArgumentParser(description="MAVLink Data Dashboard")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Serial port")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate")
    parser.add_argument("--refresh", type=float, default=0.1, help="Refresh interval (s)")
    return parser.parse_args()


def rad_to_deg(rad):
    return rad * 180.0 / math.pi


CTRL_NAMES = {0: "PID", 1: "LQR", 2: "INDI"}


def update_data(data, msg):
    """MAVLink 메시지에서 대시보드 데이터 업데이트"""
    msg_type = msg.get_type()

    if msg_type == "HEARTBEAT":
        data.heartbeat_count += 1
        data.last_heartbeat = time.time()

    elif msg_type == "ATTITUDE":
        data.roll = msg.roll
        data.pitch = msg.pitch
        data.yaw = msg.yaw
        data.roll_rate = msg.rollspeed
        data.pitch_rate = msg.pitchspeed
        data.yaw_rate = msg.yawspeed

    elif msg_type == "GPS_RAW_INT":
        data.gps_fix = msg.fix_type
        data.lat = msg.lat / 1e7
        data.lon = msg.lon / 1e7
        data.alt = msg.alt / 1000.0
        data.num_sv = msg.satellites_visible

    elif msg_type == "SYS_STATUS":
        data.battery_mv = msg.voltage_battery
        data.battery_pct = msg.battery_remaining
        data.cpu_load = msg.load / 10.0


def print_dashboard(data):
    """터미널 대시보드 출력 (ANSI escape로 화면 갱신)"""
    # 커서를 홈으로 이동
    print("\033[H\033[J", end="")

    hb_age = time.time() - data.last_heartbeat if data.last_heartbeat > 0 else -1

    print("=" * 60)
    print("  Pixhawk 6C — Real-time Dashboard")
    print("=" * 60)

    # Attitude
    print(f"\n  ATTITUDE")
    print(f"    Roll:  {rad_to_deg(data.roll):+7.2f} deg   Rate: {rad_to_deg(data.roll_rate):+7.2f} deg/s")
    print(f"    Pitch: {rad_to_deg(data.pitch):+7.2f} deg   Rate: {rad_to_deg(data.pitch_rate):+7.2f} deg/s")
    print(f"    Yaw:   {rad_to_deg(data.yaw):+7.2f} deg   Rate: {rad_to_deg(data.yaw_rate):+7.2f} deg/s")

    # GPS
    fix_str = ["No Fix", "Dead Reck", "2D", "3D", "3D+DR", "Time"][min(data.gps_fix, 5)]
    print(f"\n  GPS ({fix_str}, {data.num_sv} satellites)")
    print(f"    Lat: {data.lat:.7f}   Lon: {data.lon:.7f}")
    print(f"    Alt: {data.alt:.1f} m")

    # System
    batt_str = f"{data.battery_pct}%" if data.battery_pct >= 0 else "N/A"
    print(f"\n  SYSTEM")
    print(f"    Battery: {data.battery_mv}mV ({batt_str})")
    print(f"    CPU:     {data.cpu_load:.1f}%")

    # Controller
    print(f"\n  CONTROLLER: {data.ctrl_type}")
    print(f"    Roll:  {data.ctrl_roll:+.3f}")
    print(f"    Pitch: {data.ctrl_pitch:+.3f}")
    print(f"    Yaw:   {data.ctrl_yaw:+.3f}")

    # Connection
    hb_str = f"{hb_age:.1f}s ago" if hb_age >= 0 else "never"
    print(f"\n  LINK: heartbeats={data.heartbeat_count}, last={hb_str}")
    print(f"\n  Press Ctrl+C to exit")


def main():
    args = parse_args()

    print(f"[*] Connecting to {args.port} at {args.baud} baud...")
    mav = mavutil.mavlink_connection(args.port, baud=args.baud)

    print("[*] Waiting for heartbeat...")
    mav.wait_heartbeat()
    print("[+] Connected!")

    data = DashboardData()

    try:
        while True:
            # 비블로킹으로 모든 대기 메시지 처리
            while True:
                msg = mav.recv_match(blocking=False)
                if msg is None:
                    break
                update_data(data, msg)

            print_dashboard(data)
            time.sleep(args.refresh)

    except KeyboardInterrupt:
        print("\n[*] Dashboard stopped.")


if __name__ == "__main__":
    main()
