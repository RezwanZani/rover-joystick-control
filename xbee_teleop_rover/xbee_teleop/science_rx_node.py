#!/usr/bin/env python3
"""
science_rx_node.py -- Network UDP Receiver for Rover Science Module (ROS 2).

Listens on a UDP socket (default 0.0.0.0:8889) for science commands sent over
Wi-Fi / Ethernet from ground station GUIs, Python scripts, or socat.

Supports 4 command formats:
  1. Structured ASCII string:
     "S:1,0,0,0,0,1,90\n"  (m1, m2, m3, m4, m5, led, servo)
     Example with socat:
       echo "S:1,0,0,0,0,1,90" | socat - UDP-DATAGRAM:127.0.0.1:8889

  2. Binary struct (9 bytes, '<cBBBBBBBB'):
     Byte 0: b'S'
     Byte 1: seq (0..255)
     Bytes 2-6: m1, m2, m3, m4, m5 (0 or 1)
     Byte 7: led (0 or 1)
     Byte 8: servo (0..180 deg)

  3. JSON string:
     '{"m1": 1, "m2": 0, "m3": 0, "m4": 0, "m5": 0, "led": 1, "servo": 90}'

  4. Legacy single-character shortcuts:
     'x' = Motor 1 ON, 'c' = M1+M2 ON, 'v' = M1..M3 ON, 'b' = M1..M4 ON,
     'n' = All 5 Motors ON, 'z' = All Motors OFF,
     '1' = LED ON, '2' = LED OFF,
     '3' = Servo 0°, '4' = Servo 180°

Publishes:
  /science/actuator_cmd    (std_msgs/UInt8MultiArray) -- [m1, m2, m3, m4, m5, led, servo]
  /science/actuator_status (std_msgs/String)          -- JSON feedback

Optional direct serial link to Arduino Mega:
  If direct_serial=True and an Arduino is detected at arduino_port, sends
  "S:m1,m2,m3,m4,m5,led,servo\n" straight to serial as well.
"""
import json
import os
import socket
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray

# Binary 9-byte packet struct layout
_SCIENCE_STRUCT = struct.Struct('<cBBBBBBBB')


class ScienceRxNode(Node):
    def __init__(self):
        super().__init__('science_rx_node')

        # Parameters
        self.declare_parameter('udp_ip', '0.0.0.0')
        self.declare_parameter('udp_port', 8889)
        self.declare_parameter('cmd_topic', '/science/actuator_cmd')
        self.declare_parameter('status_topic', '/science/actuator_status')
        self.declare_parameter('send_reply', True)
        self.declare_parameter('direct_serial', False)
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('arduino_baud', 9600)

        p = lambda n: self.get_parameter(n).value

        self.udp_ip = str(p('udp_ip'))
        self.udp_port = int(p('udp_port'))
        self.cmd_topic = str(p('cmd_topic'))
        self.status_topic = str(p('status_topic'))
        self.send_reply = bool(p('send_reply'))
        self.direct_serial = bool(p('direct_serial'))
        self.arduino_port = str(p('arduino_port'))
        self.arduino_baud = int(p('arduino_baud'))

        # State storage (m1..m5: 0/1, led: 0/1, servo: 0..180)
        self.lock = threading.Lock()
        self.state = {
            'm1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'm5': 0,
            'led': 0, 'servo': 0
        }
        self.seq = 0
        self.last_rx = 0.0
        self.last_sender = None

        # ROS 2 Publishers
        self.cmd_pub = self.create_publisher(UInt8MultiArray, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        # Optional Arduino Mega direct serial connection
        self.ser = None
        if self.direct_serial:
            self._connect_serial()

        # UDP Socket & Receiver Thread
        self.udp_running = True
        self.sock = None
        self._init_udp()

        # Periodic status publisher timer (1 Hz)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f"🔬 [ScienceRxNode] Started:\n"
            f"   Listening on UDP: {self.udp_ip}:{self.udp_port}\n"
            f"   Publishing cmd : {self.cmd_topic} (UInt8MultiArray)\n"
            f"   Publishing stat: {self.status_topic} (String/JSON)\n"
            f"   Direct serial  : {self.direct_serial} ({self.arduino_port} @ {self.arduino_baud})"
        )

    def _init_udp(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.udp_ip, self.udp_port))
            self.sock.settimeout(0.5)
            self.rx_thread = threading.Thread(target=self._udp_loop, daemon=True)
            self.rx_thread.start()
            self.get_logger().info(f"✅ UDP socket successfully bound to {self.udp_ip}:{self.udp_port}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to bind UDP socket on {self.udp_port}: {e}")

    def _connect_serial(self):
        try:
            import serial
            self.ser = serial.Serial(self.arduino_port, self.arduino_baud, timeout=0.1)
            self.get_logger().info(f"✅ Connected directly to Science Arduino on {self.arduino_port} @ {self.arduino_baud}")
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f"⚠️ Direct serial to Arduino unavailable ({e}). Continuing in ROS-only mode.")

    def _udp_loop(self):
        """Continuously receive UDP packets and dispatch commands."""
        while self.udp_running and rclpy.ok():
            if not self.sock:
                time.sleep(0.5)
                continue
            try:
                data, addr = self.sock.recvfrom(4096)
                if not data:
                    continue
                self.process_payload(data, addr)
            except socket.timeout:
                continue
            except Exception as e:
                if self.udp_running:
                    self.get_logger().debug(f"UDP rx error: {e}")
                time.sleep(0.1)

    def parse_command(self, raw_data: bytes):
        """
        Parses raw bytes into a dictionary with keys:
          'm1', 'm2', 'm3', 'm4', 'm5', 'led', 'servo'
        Returns parsed dict or None if invalid.
        """
        if not raw_data:
            return None

        # 1. Binary struct: 9 bytes starting with 'S'
        if len(raw_data) == _SCIENCE_STRUCT.size and raw_data[:1] == b'S':
            try:
                _, seq, m1, m2, m3, m4, m5, led, servo = _SCIENCE_STRUCT.unpack(raw_data)
                return {
                    'm1': int(bool(m1)), 'm2': int(bool(m2)), 'm3': int(bool(m3)),
                    'm4': int(bool(m4)), 'm5': int(bool(m5)), 'led': int(bool(led)),
                    'servo': max(0, min(180, int(servo))),
                    '_seq': seq
                }
            except Exception:
                pass

        # Decode to text
        text = raw_data.decode('utf-8', errors='ignore').strip()
        if not text:
            return None

        # 2. JSON string: '{"m1": 1, ...}'
        if text.startswith('{') and text.endswith('}'):
            try:
                obj = json.loads(text)
                if isinstance(obj, dict):
                    res = {}
                    for k in ('m1', 'm2', 'm3', 'm4', 'm5', 'led'):
                        if k in obj:
                            res[k] = int(bool(obj[k]))
                    if 'servo' in obj:
                        res['servo'] = max(0, min(180, int(obj['servo'])))
                    return res if res else None
            except Exception:
                pass

        # 3. Structured ASCII: "S:m1,m2,m3,m4,m5,led,servo"
        if text.upper().startswith('S:') or text.upper().startswith('S '):
            parts = text[2:].replace(',', ' ').split()
            if len(parts) >= 6:
                try:
                    m1 = int(bool(int(parts[0])))
                    m2 = int(bool(int(parts[1])))
                    m3 = int(bool(int(parts[2])))
                    m4 = int(bool(int(parts[3])))
                    m5 = int(bool(int(parts[4])))
                    led = int(bool(int(parts[5])))
                    servo = max(0, min(180, int(parts[6]))) if len(parts) > 6 else self.state['servo']
                    return {'m1': m1, 'm2': m2, 'm3': m3, 'm4': m4, 'm5': m5, 'led': led, 'servo': servo}
                except ValueError:
                    pass

        # 4. Key-Value format: "m1=1,led=1,servo=90"
        if '=' in text:
            res = {}
            for item in text.replace(';', ',').split(','):
                if '=' in item:
                    k, v = item.split('=', 1)
                    k = k.strip().lower()
                    try:
                        v_num = int(v.strip())
                        if k in ('m1', 'm2', 'm3', 'm4', 'm5', 'led'):
                            res[k] = int(bool(v_num))
                        elif k == 'servo':
                            res[k] = max(0, min(180, v_num))
                    except ValueError:
                        pass
            if res:
                return res

        # 5. Legacy single-character commands (compat with Arduino sketch):
        ch = text.strip()
        if len(ch) == 1:
            res = {}
            if ch == 'x': res.update(m1=1)
            elif ch == 'c': res.update(m1=1, m2=1)
            elif ch == 'v': res.update(m1=1, m2=1, m3=1)
            elif ch == 'b': res.update(m1=1, m2=1, m3=1, m4=1)
            elif ch == 'n': res.update(m1=1, m2=1, m3=1, m4=1, m5=1)
            elif ch == 'z': res.update(m1=0, m2=0, m3=0, m4=0, m5=0)
            elif ch == '1': res.update(led=1)
            elif ch == '2': res.update(led=0)
            elif ch == '3': res.update(servo=0)
            elif ch == '4': res.update(servo=180)
            if res:
                return res

        return None

    def process_payload(self, raw_data: bytes, sender_addr):
        parsed = self.parse_command(raw_data)
        if not parsed:
            self.get_logger().debug(f"Unrecognized payload from {sender_addr}: {raw_data[:32]!r}")
            return

        with self.lock:
            # Update state with incoming parameters
            for k in ('m1', 'm2', 'm3', 'm4', 'm5', 'led', 'servo'):
                if k in parsed:
                    self.state[k] = parsed[k]
            self.seq = parsed.get('_seq', (self.seq + 1) & 0xFF)
            self.last_rx = time.monotonic()
            self.last_sender = sender_addr

            m1, m2, m3, m4, m5 = (self.state[f'm{i}'] for i in range(1, 6))
            led = self.state['led']
            servo = self.state['servo']

        # 1. Publish to ROS 2 topic: /science/actuator_cmd (UInt8MultiArray)
        cmd_msg = UInt8MultiArray()
        cmd_msg.data = [m1, m2, m3, m4, m5, led, servo]
        self.cmd_pub.publish(cmd_msg)

        # 2. Forward to direct serial if enabled
        if self.direct_serial and self.ser and self.ser.is_open:
            try:
                line = f"S:{m1},{m2},{m3},{m4},{m5},{led},{servo}\n".encode('ascii')
                self.ser.write(line)
            except Exception as e:
                self.get_logger().warn(f"Direct serial write failed: {e}")

        # 3. Log to console
        self.get_logger().info(
            f"🔬 [SCIENCE UDP RX from {sender_addr[0]}:{sender_addr[1]}] "
            f"M1:{m1} M2:{m2} M3:{m3} M4:{m4} M5:{m5} | LED:{led} | SERVO:{servo}° "
            f"-> Published {self.cmd_topic}"
        )

        # 4. Send acknowledgment back over UDP (useful for socat / clients)
        if self.send_reply and self.sock:
            try:
                reply = f"STATUS:m1={m1},m2={m2},m3={m3},m4={m4},m5={m5},led={led},servo={servo}\n".encode('ascii')
                self.sock.sendto(reply, sender_addr)
            except Exception as e:
                self.get_logger().debug(f"Failed to send UDP reply: {e}")

        # 5. Immediate status publish
        self.publish_status()

    def publish_status(self):
        """Publish JSON status string to /science/actuator_status."""
        with self.lock:
            st = dict(self.state)
            st['seq'] = self.seq
            st['last_rx_age'] = round(time.monotonic() - self.last_rx, 2) if self.last_rx > 0 else None
            st['sender'] = f"{self.last_sender[0]}:{self.last_sender[1]}" if self.last_sender else None

        msg = String()
        msg.data = json.dumps(st)
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.udp_running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScienceRxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
