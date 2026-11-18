#!/usr/bin/env python3
"""
send_science_cmd.py -- CLI tool to send Science commands over XBee and/or Network UDP.

Usage Examples:
  # 1. Send via Network UDP (port 8889):
  python3 send_science_cmd.py --ip 127.0.0.1 --port 8889 --m1 1 --led 1 --servo 90

  # 2. Send via XBee Radio:
  python3 send_science_cmd.py --xbee /dev/ttyUSB0 --baud 115200 --m1 1 --led 1 --servo 90

  # 3. Dual-Redundant (Send via BOTH XBee and Network UDP simultaneously):
  python3 send_science_cmd.py --xbee /dev/ttyUSB0 --ip 192.168.1.100 --port 8889 --m1 1 --servo 45

  # 4. Emergency Stop all motors:
  python3 send_science_cmd.py --ip 127.0.0.1 --port 8889 --all-off

  # 5. Raw ASCII command string (matches socat):
  python3 send_science_cmd.py --ip 127.0.0.1 --port 8889 --cmd "S:1,0,1,0,0,1,45"

  # Equivalent socat one-liners:
  echo "S:1,0,0,0,0,1,90" | socat - UDP-DATAGRAM:127.0.0.1:8889
  echo '{"m1": 1, "led": 1, "servo": 90}' | socat - UDP-DATAGRAM:127.0.0.1:8889
"""
import argparse
import socket
import struct
import sys
import time

_SCIENCE_STRUCT = struct.Struct('<cBBBBBBBB')


def wrap_xbee_api_tx(raw_payload: bytes, dest_addr_hex: str = '') -> bytes:
    """Build an XBee API 0x10 frame (Frame ID 0x00, Option 0x01 Disable RF ACK)."""
    clean_hex = dest_addr_hex.strip().upper().replace('0X', '').zfill(16) if dest_addr_hex else ''
    if clean_hex and len(clean_hex) == 16 and clean_hex != '000000000000FFFF':
        dest_bytes = bytes.fromhex(clean_hex)
    else:
        dest_bytes = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF])
    header = bytes([
        0x10, 0x00,
        dest_bytes[0], dest_bytes[1], dest_bytes[2], dest_bytes[3],
        dest_bytes[4], dest_bytes[5], dest_bytes[6], dest_bytes[7],
        0xFF, 0xFE,
        0x00, 0x01
    ])
    frame_data = header + raw_payload
    length = len(frame_data)
    checksum = 0xFF - (sum(frame_data) & 0xFF)
    return struct.pack('>BH', 0x7E, length) + frame_data + bytes([checksum])


def main():
    parser = argparse.ArgumentParser(description="Send Science commands over XBee and/or Network UDP.")
    # Network UDP options
    parser.add_argument('--ip', default='127.0.0.1', help="Target Rover IP for UDP (default: 127.0.0.1)")
    parser.add_argument('--port', type=int, default=8889, help="Target Science UDP Port (default: 8889)")
    parser.add_argument('--no-udp', action='store_true', help="Disable sending via UDP")

    # XBee Serial options
    parser.add_argument('--xbee', type=str, default='', help="XBee serial port (e.g. /dev/ttyUSB0, COM3)")
    parser.add_argument('--baud', type=int, default=115200, help="XBee baud rate (default: 115200)")
    parser.add_argument('--remote-addr', type=str, default='', help="Rover 64-bit radio hex address (default: broadcast)")

    # Actuator states
    parser.add_argument('--m1', type=int, choices=[0, 1], help="Motor 1 (Sample 1) [0/1]")
    parser.add_argument('--m2', type=int, choices=[0, 1], help="Motor 2 (Sample 2) [0/1]")
    parser.add_argument('--m3', type=int, choices=[0, 1], help="Motor 3 (Reagent 1) [0/1]")
    parser.add_argument('--m4', type=int, choices=[0, 1], help="Motor 4 (Reagent 2) [0/1]")
    parser.add_argument('--m5', type=int, choices=[0, 1], help="Motor 5 (Flush) [0/1]")
    parser.add_argument('--led', type=int, choices=[0, 1], help="Illumination LED [0/1]")
    parser.add_argument('--servo', type=int, help="Carousel Servo angle [0..180 deg]")
    parser.add_argument('--all-off', action='store_true', help="Turn off all 5 motors and LED")
    parser.add_argument('--all-on', action='store_true', help="Turn on all 5 motors and LED")
    parser.add_argument('--cmd', type=str, help="Raw ASCII command string, e.g. 'S:1,0,1,0,0,1,90'")
    parser.add_argument('--timeout', type=float, default=1.5, help="Seconds to wait for UDP reply (default: 1.5s)")

    args = parser.parse_args()

    # Determine command payload
    if args.cmd:
        ascii_payload = (args.cmd.strip() + '\n').encode('utf-8')
        desc = f"Raw string: {args.cmd.strip()}"
        # Parse numbers for binary struct if possible
        try:
            parts = args.cmd.strip()[2:].replace(',', ' ').split()
            m1, m2, m3, m4, m5, led = (int(parts[i]) for i in range(6))
            servo = int(parts[6]) if len(parts) > 6 else 0
        except Exception:
            m1 = m2 = m3 = m4 = m5 = led = servo = 0
    else:
        m1 = args.m1 or 0
        m2 = args.m2 or 0
        m3 = args.m3 or 0
        m4 = args.m4 or 0
        m5 = args.m5 or 0
        led = args.led or 0
        servo = args.servo if args.servo is not None else 0

        if args.all_off:
            m1 = m2 = m3 = m4 = m5 = led = 0
        elif args.all_on:
            m1 = m2 = m3 = m4 = m5 = led = 1

        servo = max(0, min(180, int(servo)))
        ascii_payload = f"S:{m1},{m2},{m3},{m4},{m5},{led},{servo}\n".encode('ascii')
        desc = f"M1:{m1} M2:{m2} M3:{m3} M4:{m4} M5:{m5} | LED:{led} | SERVO:{servo}°"

    seq = int(time.time()) & 0xFF
    bin_payload = _SCIENCE_STRUCT.pack(b'S', seq, m1, m2, m3, m4, m5, led, servo)

    print(f"📦 Command: {desc}")

    # 1. Transmit over XBee (if requested)
    if args.xbee:
        print(f"📡 Sending via XBee Radio ({args.xbee} @ {args.baud})...")
        try:
            import serial
            ser = serial.Serial(args.xbee, args.baud, timeout=0.5, write_timeout=0.5)
            frame = wrap_xbee_api_tx(bin_payload, args.remote_addr)
            # Send 2-packet burst for high RF reliability
            ser.write(frame)
            time.sleep(0.01)
            ser.write(frame)
            ser.close()
            print("✅ Transmitted over XBee RF successfully (2-frame burst).")
        except Exception as e:
            print(f"❌ XBee transmission error: {e}")

    # 2. Transmit over Network UDP (unless --no-udp)
    if not args.no_udp:
        print(f"🌐 Sending via Network UDP ({args.ip}:{args.port})...")
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(args.timeout)
            sock.sendto(ascii_payload, (args.ip, args.port))
            sock.sendto(bin_payload, (args.ip, args.port))
            print("✅ Transmitted over Network UDP successfully.")

            # Listen for reply
            try:
                reply_data, sender = sock.recvfrom(2048)
                reply_str = reply_data.decode('utf-8', errors='ignore').strip()
                print(f"📥 Response from {sender[0]}:{sender[1]}: {reply_str}")
            except socket.timeout:
                print("⏱️ No UDP reply received within timeout.")
            finally:
                sock.close()
        except Exception as e:
            print(f"❌ Network UDP transmission error: {e}")


if __name__ == '__main__':
    main()
