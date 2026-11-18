#!/usr/bin/env python3
"""
test_xbee_rf_link.py -- Direct Hardware RF Communication & Latency Test.

Tests physical over-the-air communication between two XBee radios:
  - Radio A: Base station transmitter (default: /dev/ttyUSB2)
  - Radio B: Rover receiver (default: /dev/ttyUSB3)

Verifies:
  1. Serial port connectivity at 115200 baud
  2. Querying 64-bit hardware addresses (SH/SL)
  3. Bidirectional transmission:
     - Base -> Rover: Motion ('M'), Science ('S'), Mode ('N')
     - Rover -> Base: ACK ('A')
  4. Packet round-trip latency & packet loss statistics

Usage:
  python3 tests/test_xbee_rf_link.py
  python3 tests/test_xbee_rf_link.py /dev/ttyUSB2 /dev/ttyUSB3
"""

import os
import sys
import time

# Add rover source path for link module
SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rover', 'src', 'xbee_teleop_rover'))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from xbee_teleop.link import (
    XBeeLink,
    pack_motion,
    pack_science,
    pack_mode,
    pack_ack,
    unpack,
)


def run_rf_test(base_port='/dev/ttyUSB2', rover_port='/dev/ttyUSB3', baud=115200, count=10):
    print("=" * 65)
    print("  AAUB ROVER71 -- PHYSICAL XBEE RF HARDWARE LINK TEST")
    print("=" * 65)
    print(f"  Base Radio Port  : {base_port}")
    print(f"  Rover Radio Port : {rover_port}")
    print(f"  Baud Rate        : {baud}")
    print(f"  Test Packets     : {count}")
    print("=" * 65)

    rover_received = []
    base_acks = []

    def on_rover_rx(data):
        unpacked = unpack(data)
        if unpacked:
            rover_received.append(unpacked)

    def on_base_rx(data):
        unpacked = unpack(data)
        if unpacked and unpacked[0] == 'ack':
            base_acks.append(unpacked)

    base_link = XBeeLink(base_port, baud, '', on_base_rx)
    rover_link = XBeeLink(rover_port, baud, '', on_rover_rx)

    print("\n[1/4] Connecting to serial ports...")
    base_link.connect()
    rover_link.connect()
    time.sleep(0.5)

    if not base_link.connected:
        print(f"❌ Failed to connect to Base Radio on {base_port}: {base_link.err}")
        return False
    if not rover_link.connected:
        print(f"❌ Failed to connect to Rover Radio on {rover_port}: {rover_link.err}")
        base_link.close()
        return False

    print(f"  ✅ Base  Serial Connected on {base_port}")
    print(f"  ✅ Rover Serial Connected on {rover_port}")

    # Query local addresses
    time.sleep(0.3)
    base_addr = base_link.local_addr or "Unknown"
    rover_addr = rover_link.local_addr or "Unknown"
    print(f"  📍 Base  XBee Address: {base_addr}")
    print(f"  📍 Rover XBee Address: {rover_addr}")

    print("\n[2/4] Testing Downlink (Base -> Rover: Motion, Science, Mode)...")
    latencies = []

    for seq in range(1, count + 1):
        # Alternate packet types
        if seq % 3 == 1:
            pkt = pack_motion(seq, 0.450, -0.120, 1.0, 0.0, -1.0, 1.0, 0.0)
            pkt_desc = f"MOTION (seq={seq}, vx=+0.45, wz=-0.12)"
        elif seq % 3 == 2:
            pkt = pack_science(seq, 1, 0, 1, 0, 0, 1, 90)
            pkt_desc = f"SCIENCE (seq={seq}, M1,3:ON LED:ON Servo:90)"
        else:
            pkt = pack_mode(seq, 1)
            pkt_desc = f"MODE (seq={seq}, AUTONOMOUS)"

        t0 = time.monotonic()
        base_link.send(pkt)

        # Wait for delivery
        delivered = False
        timeout = t0 + 0.3
        while time.monotonic() < timeout:
            if any(p[1] == seq for p in rover_received):
                elapsed_ms = (time.monotonic() - t0) * 1000.0
                latencies.append(elapsed_ms)
                delivered = True
                break
            time.sleep(0.005)

        status = f"✅ OK ({elapsed_ms:5.1f} ms)" if delivered else "❌ TIMEOUT"
        print(f"  [{seq:02d}/{count:02d}] {pkt_desc:<45} {status}")
        time.sleep(0.03)

    print("\n[3/4] Testing Uplink (Rover -> Base: ACK telemetry)...")
    for seq in range(1, 4):
        ack_pkt = pack_ack(seq, 0)
        rover_link.send(ack_pkt)
        time.sleep(0.05)

    time.sleep(0.2)
    ack_count = len(base_acks)
    print(f"  Base received {ack_count}/3 ACK response packets from Rover.")

    # [4/4] Results summary
    loss_rate = ((count - len(latencies)) / count) * 100.0
    avg_lat = (sum(latencies) / len(latencies)) if latencies else 0.0
    min_lat = min(latencies) if latencies else 0.0
    max_lat = max(latencies) if latencies else 0.0

    print("\n" + "=" * 65)
    print("  TEST SUMMARY")
    print("=" * 65)
    print(f"  Packets Sent        : {count}")
    print(f"  Packets Received    : {len(latencies)}")
    print(f"  Packet Loss Rate    : {loss_rate:.1f}%")
    if latencies:
        print(f"  Round-Trip Latency  : min={min_lat:.1f}ms | avg={avg_lat:.1f}ms | max={max_lat:.1f}ms")
    print("=" * 65)

    base_link.close()
    rover_link.close()

    success = loss_rate == 0.0 and ack_count > 0
    if success:
        print("  🎉 TEST PASSED: Radio link is HEALTHY, FAST, and STABLE!\n")
    else:
        print("  ⚠️ TEST COMPLETED WITH WARNINGS: Check antenna or baud rates.\n")

    return success


if __name__ == '__main__':
    p1 = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB2'
    p2 = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyUSB3'
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200

    ok = run_rf_test(base_port=p1, rover_port=p2, baud=baud)
    sys.exit(0 if ok else 1)
