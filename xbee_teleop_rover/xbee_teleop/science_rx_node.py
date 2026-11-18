#!/usr/bin/env python3
"""
science_rx_node.py -- Dual-Redundant Receiver for Rover Science Module (ROS 2).

Reads Science commands over:
  1. Primary:   XBee Radio (RF API frame 0x10 / 0x90 via serial)
  2. Secondary: Network UDP Socket (Wi-Fi / Ethernet / socat on port 8889)

Automatic Failover & Failback:
  - While XBee is healthy, commands are executed strictly from XBee (redundant UDP ignored).
  - If XBee communication fails/times out (>failover_timeout seconds) or is disconnected,
    seamlessly fails over to Network UDP.
  - As soon as XBee signal resumes, automatically fails back to XBee as primary.

Supported Payload Formats:
  1. Structured ASCII string (socat / echo):
     "S:1,0,0,0,0,1,90\n"  (m1, m2, m3, m4, m5, led, servo)
     Example: echo "S:1,0,0,0,0,1,90" | socat - UDP-DATAGRAM:127.0.0.1:8889

  2. Binary Struct (9 bytes, '<cBBBBBBBB'):
     Byte 0: b'S', Byte 1: seq, Bytes 2-6: m1..m5, Byte 7: led, Byte 8: servo

  3. JSON String:
     '{"m1": 1, "led": 1, "servo": 90}'

  4. Single-char Shortcuts (Arduino Mega sketch compatibility):
     'x' = M1 ON, 'z' = All OFF, '1' = LED ON, '2' = LED OFF, etc.

Publishes:
  /science/actuator_cmd    (std_msgs/UInt8MultiArray) -- [m1, m2, m3, m4, m5, led, servo]
  /science/actuator_status (std_msgs/String)          -- JSON status with active_source & link health
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

from xbee_teleop.link import XBeeLink, unpack

_SCIENCE_STRUCT = struct.Struct('<cBBBBBBBB')


class ScienceRxNode(Node):
    def __init__(self):
        super().__init__('science_rx_node')

        # ------------------------------------------------------------------ #
        # ROS 2 Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter('xbee_port', '/dev/ttyUSB0')
        self.declare_parameter('xbee_baud', 115200)
        self.declare_parameter('remote_addr', '')
        self.declare_parameter('failover_timeout', 1.0)
        self.declare_parameter('udp_ip', '0.0.0.0')
        self.declare_parameter('udp_port', 8889)
        self.declare_parameter('cmd_topic', '/science/actuator_cmd')
        self.declare_parameter('status_topic', '/science/actuator_status')
        self.declare_parameter('send_reply', True)
        self.declare_parameter('direct_serial', False)
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('arduino_baud', 9600)

        p = lambda n: self.get_parameter(n).value

        self.xbee_port = str(p('xbee_port')).strip()
        self.xbee_baud = int(p('xbee_baud'))
        self.remote_addr = str(p('remote_addr')).strip()
        self.failover_timeout = float(p('failover_timeout'))
        self.udp_ip = str(p('udp_ip'))
        self.udp_port = int(p('udp_port'))
        self.cmd_topic = str(p('cmd_topic'))
        self.status_topic = str(p('status_topic'))
        self.send_reply = bool(p('send_reply'))
        self.direct_serial = bool(p('direct_serial'))
        self.arduino_port = str(p('arduino_port'))
        self.arduino_baud = int(p('arduino_baud'))

        # Shared state storage
        self.lock = threading.Lock()
        self.state = {
            'm1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'm5': 0,
            'led': 0, 'servo': 0
        }
        self.seq = 0
        self.last_rx = 0.0
        self.last_xbee_rx = None
        self.last_udp_rx = None
        self.active_source = 'DISCONNECTED'   # 'XBEE' | 'UDP' | 'DISCONNECTED'
        self.failover_count = 0
        self.last_sender = None

        # ROS 2 Publishers
        self.cmd_pub = self.create_publisher(UInt8MultiArray, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        # Direct Arduino Serial (optional)
        self.arduino_ser = None
        if self.direct_serial:
            self._connect_arduino_serial()

        # Primary Link: XBee Radio
        self.xbee_link = None
        if self.xbee_port and self.xbee_port.lower() not in ('none', 'false', 'disabled', ''):
            self.xbee_link = XBeeLink(self.xbee_port, self.xbee_baud, self.remote_addr, self.on_xbee_data)
            self.xbee_link.connect()

        # Secondary Link: UDP Network Socket
        self.udp_running = True
        self.sock = None
        self._init_udp()

        # Watchdog & Status Timers
        self.create_timer(1.0, self.publish_status)
        self.create_timer(2.0, self._watchdog)

        self.get_logger().info(
            f"🔬 [ScienceRxNode] Started:\n"
            f"   Primary Link   : XBee on {self.xbee_port or 'DISABLED'} @ {self.xbee_baud}\n"
            f"   Secondary Link : UDP on {self.udp_ip}:{self.udp_port} (Wi-Fi/Ethernet/socat)\n"
            f"   Failover Window: {self.failover_timeout:.1f} s (XBee Priority -> UDP Failover -> XBee Failback)\n"
            f"   Publishing Cmd : {self.cmd_topic} (UInt8MultiArray)\n"
            f"   Publishing Stat: {self.status_topic} (String/JSON)"
        )

    # ---------------------------------------------------------------------- #
    # Hardware Link Initializers & Watchdogs
    # ---------------------------------------------------------------------- #
    def _init_udp(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.udp_ip, self.udp_port))
            self.sock.settimeout(0.5)
            self.rx_thread = threading.Thread(target=self._udp_loop, daemon=True)
            self.rx_thread.start()
            self.get_logger().info(f"✅ Secondary UDP receiver bound to {self.udp_ip}:{self.udp_port}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to bind UDP socket on {self.udp_port}: {e}")

    def _connect_arduino_serial(self):
        try:
            import serial
            self.arduino_ser = serial.Serial(self.arduino_port, self.arduino_baud, timeout=0.1)
            self.get_logger().info(f"✅ Connected to Science Arduino on {self.arduino_port} @ {self.arduino_baud}")
        except Exception as e:
            self.arduino_ser = None
            self.get_logger().warn(f"⚠️ Direct Arduino serial unavailable ({e}). Continuing in ROS-only mode.")

    def _watchdog(self):
        """Auto-reconnect XBee radio if disconnected or port plugged in late."""
        if self.xbee_link and not self.xbee_link.connected:
            self.xbee_link.connect()

    # ---------------------------------------------------------------------- #
    # Receiver Callbacks
    # ---------------------------------------------------------------------- #
    def on_xbee_data(self, data: bytes):
        """Callback from XBee serial thread when an RF packet arrives."""
        self.process_packet(data, source='XBEE', sender_info='XBEE_RF')

    def _udp_loop(self):
        """Continuous background thread receiving UDP network datagrams."""
        while self.udp_running and rclpy.ok():
            if not self.sock:
                time.sleep(0.5)
                continue
            try:
                data, addr = self.sock.recvfrom(4096)
                if not data:
                    continue
                sender_str = f"{addr[0]}:{addr[1]}"
                self.process_packet(data, source='UDP', sender_info=sender_str, sender_addr=addr)
            except socket.timeout:
                continue
            except Exception as e:
                if self.udp_running:
                    self.get_logger().debug(f"UDP rx error: {e}")
                time.sleep(0.1)

    # ---------------------------------------------------------------------- #
    # Packet Parsers
    # ---------------------------------------------------------------------- #
    def parse_command(self, raw_data: bytes):
        """Parses raw bytes into dict of {'m1'..'m5', 'led', 'servo', '_seq'} or None."""
        if not raw_data:
            return None

        # 1. Standard unpack helper ('science' packet from link.py or 9-byte struct)
        m = unpack(raw_data)
        if m and m[0] == 'science':
            _, seq, m1, m2, m3, m4, m5, led, servo = m
            return {
                'm1': int(bool(m1)), 'm2': int(bool(m2)), 'm3': int(bool(m3)),
                'm4': int(bool(m4)), 'm5': int(bool(m5)), 'led': int(bool(led)),
                'servo': max(0, min(180, int(servo))),
                '_seq': seq
            }

        # Raw binary struct fallback
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

        # Text decoders
        text = raw_data.decode('utf-8', errors='ignore').strip()
        if not text:
            return None

        # 2. JSON string
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

        # 5. Single-character shortcuts (from Arduino Mega sketch)
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

    # ---------------------------------------------------------------------- #
    # Core Failover & Execution Engine
    # ---------------------------------------------------------------------- #
    def process_packet(self, raw_data: bytes, source: str, sender_info: str, sender_addr=None):
        """Unified packet handler implementing:
          1. XBee priority
          2. Seamless failover to Network UDP on XBee timeout / silence (>failover_timeout)
          3. Immediate failback to XBee upon XBee signal restoration
        """
        parsed = self.parse_command(raw_data)
        if not parsed:
            return

        now = time.monotonic()
        with self.lock:
            if source == 'XBEE':
                self.last_xbee_rx = now
            elif source == 'UDP':
                self.last_udp_rx = now

            xbee_alive = (
                (self.last_xbee_rx is not None)
                and ((now - self.last_xbee_rx) < self.failover_timeout)
            )

            # FAILOVER SELECTION:
            # If XBee is active and this command arrived from UDP: ignore UDP, let XBee rule!
            if source == 'UDP' and xbee_alive:
                self.get_logger().debug(f"[UDP Dropped] XBee is healthy and primary ({sender_info}).")
                # Still send UDP confirmation back if requested
                if self.send_reply and sender_addr and self.sock:
                    try:
                        m1, m2, m3, m4, m5 = (self.state[f'm{i}'] for i in range(1, 6))
                        led, servo = self.state['led'], self.state['servo']
                        reply = f"STATUS:m1={m1},m2={m2},m3={m3},m4={m4},m5={m5},led={led},servo={servo},src=XBEE_PRIMARY\n".encode('ascii')
                        self.sock.sendto(reply, sender_addr)
                    except Exception:
                        pass
                return

            # FAILOVER LOGGING:
            # Check state transitions
            if source == 'UDP' and self.active_source == 'XBEE':
                self.failover_count += 1
                self.get_logger().warn(
                    f"⚠️ [FAILOVER -> UDP] XBee silent for >{self.failover_timeout:.1f} s! "
                    f"Seamlessly switched to Network UDP ({sender_info})."
                )
            elif source == 'XBEE' and self.active_source == 'UDP':
                self.get_logger().info(
                    f"🔄 [FAILBACK -> XBEE] XBee signal restored! "
                    f"Immediately switched back to Primary XBee."
                )

            self.active_source = source
            self.last_rx = now
            self.last_sender = sender_info

            # Apply parameters
            for k in ('m1', 'm2', 'm3', 'm4', 'm5', 'led', 'servo'):
                if k in parsed:
                    self.state[k] = parsed[k]
            self.seq = parsed.get('_seq', (self.seq + 1) & 0xFF)

            m1, m2, m3, m4, m5 = (self.state[f'm{i}'] for i in range(1, 6))
            led = self.state['led']
            servo = self.state['servo']

        # 1. Publish to ROS 2 topic: /science/actuator_cmd (std_msgs/UInt8MultiArray)
        cmd_msg = UInt8MultiArray()
        cmd_msg.data = [m1, m2, m3, m4, m5, led, servo]
        self.cmd_pub.publish(cmd_msg)

        # 2. Forward to direct Arduino serial if enabled
        if self.direct_serial and self.arduino_ser and self.arduino_ser.is_open:
            try:
                line = f"S:{m1},{m2},{m3},{m4},{m5},{led},{servo}\n".encode('ascii')
                self.arduino_ser.write(line)
            except Exception as e:
                self.get_logger().warn(f"Direct serial write failed: {e}")

        # 3. Log to console
        self.get_logger().info(
            f"🔬 [SCIENCE CMD via {source} from {sender_info}] "
            f"M1:{m1} M2:{m2} M3:{m3} M4:{m4} M5:{m5} | LED:{led} | SERVO:{servo}° "
            f"-> Published {self.cmd_topic}"
        )

        # 4. If UDP, send acknowledgment back over network
        if source == 'UDP' and self.send_reply and sender_addr and self.sock:
            try:
                reply = f"STATUS:m1={m1},m2={m2},m3={m3},m4={m4},m5={m5},led={led},servo={servo}\n".encode('ascii')
                self.sock.sendto(reply, sender_addr)
            except Exception as e:
                self.get_logger().debug(f"Failed to send UDP reply: {e}")

        # 5. Immediate status publish
        self.publish_status()

    def publish_status(self):
        """Publish JSON status string to /science/actuator_status."""
        now = time.monotonic()
        with self.lock:
            st = dict(self.state)
            st['seq'] = self.seq
            st['active_source'] = self.active_source
            st['failover_count'] = self.failover_count
            st['last_rx_age'] = round(now - self.last_rx, 2) if self.last_rx > 0 else None
            st['xbee_connected'] = (self.xbee_link is not None and self.xbee_link.connected)
            st['xbee_alive'] = (
                (self.last_xbee_rx is not None)
                and ((now - self.last_xbee_rx) < self.failover_timeout)
                and st['xbee_connected']
            )
            st['udp_alive'] = (
                (self.last_udp_rx is not None)
                and ((now - self.last_udp_rx) < self.failover_timeout)
            )
            st['sender'] = self.last_sender

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
        if self.xbee_link:
            try:
                self.xbee_link.close()
            except Exception:
                pass
        if self.arduino_ser and self.arduino_ser.is_open:
            try:
                self.arduino_ser.close()
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
