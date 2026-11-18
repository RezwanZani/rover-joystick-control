#!/usr/bin/env python3
"""
rx_node.py – XBee receiver for the rover (ROS 2).

Receives combined MOTION packets from the base station over XBee and publishes:
  /cmd_vel   (geometry_msgs/Twist)            – drive commands (vx, wz)
  /arm_vel   (std_msgs/Float32MultiArray)     – arm commands [act1, act2, act3, wrist, gripper]

Safety: publishes zeros on both topics if no packet arrives within cmd_timeout seconds.
Also publishes diagnostic JSON on xbee_rx/status.

Arm channel order in /arm_vel:
  index 0 = act1    (linear actuator 1)
  index 1 = act2    (linear actuator 2)
  index 2 = act3    (linear actuator 3)
  index 3 = wrist   (stepper motor)
  index 4 = gripper (gear motor)
Values: -1.0 (contract)  |  0.0 (still)  |  +1.0 (expand)
"""
import json
import os
import socket
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String, UInt8MultiArray
try:
    from rtcm_msgs.msg import Message as RTCMMessage
except ImportError:
    RTCMMessage = None

from xbee_teleop.link import XBeeLink, pack_ack, unpack

# Default network parameters (override via params.yaml or ROS launch)
ROVER_UDP_LISTEN_IP = '0.0.0.0'
ROVER_UDP_LISTEN_PORT = 8888
XBEE_FAILOVER_TIMEOUT = 0.35


class RxNode(Node):
    def __init__(self):
        super().__init__('rx_node')

        # ------------------------------------------------------------------ #
        # ROS parameters (override in config/params.yaml)
        # ------------------------------------------------------------------ #
        for k, v in dict(
            port='/dev/ttyUSB0',
            baud=115200,
            remote_addr='',
            cmd_timeout=0.5,
            cmd_vel_topic='cmd_vel',
            arm_vel_topic='arm_vel',   # topic for arm commands
            udp_ip=ROVER_UDP_LISTEN_IP,
            udp_port=ROVER_UDP_LISTEN_PORT,
            xbee_failover_timeout=XBEE_FAILOVER_TIMEOUT,
        ).items():
            self.declare_parameter(k, v)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        # ------------------------------------------------------------------ #
        # XBee link
        # ------------------------------------------------------------------ #
        self.link = XBeeLink(p('port'), p('baud'), p('remote_addr'), self.on_radio)

        # ------------------------------------------------------------------ #
        # Shared state (written by XBee thread, read by ROS timer thread)
        # ------------------------------------------------------------------ #
        self.lock = threading.Lock()

        # Drive values
        self.vx = 0.0
        self.wz = 0.0

        # Arm values: act1, act2, act3, wrist, gripper
        self.act1    = 0.0
        self.act2    = 0.0
        self.act3    = 0.0
        self.wrist   = 0.0
        self.gripper = 0.0

        # Packet tracking
        self.seq     = None      # sequence number of last valid packet
        self.lost    = 0         # cumulative lost packet count
        self.last_rx = None      # monotonic time of the last valid command
        self._last   = (time.monotonic(), 0)  # for rate calculation

        # ------------------------------------------------------------------ #
        # Publishers & Subscribers (Rover Comms Master)
        # ------------------------------------------------------------------ #
        self.drive_pub = self.create_publisher(Twist, p('cmd_vel_topic'), 10)
        self.arm_pub   = self.create_publisher(Float32MultiArray, p('arm_vel_topic'), 10)
        self.status_pub = self.create_publisher(String, 'xbee_rx/status', 10)

        # Science Mission Actuator Publisher & Rover Mode Publisher
        self.science_pub = self.create_publisher(UInt8MultiArray, '/science/actuator_cmd', 10)
        self.mode_pub    = self.create_publisher(String, '/rover/mode', 10)
        self.current_mode = "TELEOP"
        self.last_science_seq = None
        self.last_mode_seq = None

        # GPS RTCM Injection Publisher
        if RTCMMessage:
            self.rtcm_pub = self.create_publisher(RTCMMessage, '/rtcm', 10)
        else:
            self.rtcm_pub = self.create_publisher(UInt8MultiArray, '/rtcm', 10)
        self.rtcm_count = 0
        self._last_rtcm_log = 0.0

        # Outgoing Telemetry Subscriber (from rover_telemetry node)
        self.create_subscription(String, 'telemetry_tx', self.on_telemetry_tx, 10)

        # ------------------------------------------------------------------ #
        # Dual-Redundancy & Failover State
        # ------------------------------------------------------------------ #
        self.last_xbee_rx = None
        self.last_udp_rx = None
        self.last_executed_seq = None
        self.active_source = 'DISCONNECTED'   # 'XBEE' | 'UDP' | 'DISCONNECTED'
        self.stale_rejected = 0
        self.duplicates_rejected = 0
        self.xbee_failover_timeout = float(p('xbee_failover_timeout'))
        self.udp_ip = str(p('udp_ip'))
        self.udp_port = int(p('udp_port'))
        self._recent_rtcm_hashes = []

        # Wi-Fi UDP Receiver Thread
        self.udp_running = True
        self.udp_sock = None
        self._init_udp_receiver()

        # ------------------------------------------------------------------ #
        # Timers
        # ------------------------------------------------------------------ #
        self.create_timer(0.05, self.publish_cmd)      # 20 Hz: publish drive + arm
        self.create_timer(0.5,  self.send_ack)         # 2 Hz:  send ACK to base station
        self.create_timer(0.2,  self.publish_status)   # 5 Hz:  publish diagnostic JSON
        self.create_timer(2.0,  self._reconnect)       # 0.5 Hz: auto-reconnect XBee

        # ------------------------------------------------------------------ #
        # Connect XBee
        # ------------------------------------------------------------------ #
        self.link.connect()
        self.get_logger().info(
            f'rx_node started — XBee port: {p("port")} @ {p("baud")} baud\n'
            f'  Publishing drive commands on /{p("cmd_vel_topic")}\n'
            f'  Publishing arm  commands on /{p("arm_vel_topic")}\n'
            f'  Waiting for MOTION packets from base station...'
        )

    # ---------------------------------------------------------------------- #
    # Dual-Redundant UDP Receiver
    # ---------------------------------------------------------------------- #
    def _init_udp_receiver(self):
        """Bind UDP socket and launch receiver thread. Never raises — retried by watchdog."""
        try:
            if self.udp_sock:
                try:
                    self.udp_sock.close()
                except Exception:
                    pass
                self.udp_sock = None
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_sock.bind((self.udp_ip, self.udp_port))
            self.udp_sock.settimeout(0.5)
            self.udp_thread = threading.Thread(target=self._udp_rx_loop, daemon=True)
            self.udp_thread.start()
            self.get_logger().info(f"✅ Dual-Redundancy UDP receiver bound to {self.udp_ip}:{self.udp_port}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Could not bind UDP receiver on {self.udp_port}: {e} — will retry in 5 s")

    def _udp_rx_loop(self):
        """Background UDP receiver thread. Auto-restarts on any socket error."""
        while self.udp_running:
            # If socket is gone, wait for watchdog to rebind
            if not self.udp_sock:
                time.sleep(0.5)
                continue
            try:
                data, _ = self.udp_sock.recvfrom(4096)
                if data:
                    try:
                        self.process_packet(data, source='UDP')
                    except Exception as e:
                        self.get_logger().debug(f'[UDP] process_packet error: {e}')
            except socket.timeout:
                continue
            except OSError as e:
                # Socket was closed externally (rebind in progress) — wait and retry
                self.get_logger().debug(f'[UDP] Socket error in rx loop: {e}')
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().debug(f'[UDP] Unexpected rx loop error: {e}')
                time.sleep(0.5)

    # ---------------------------------------------------------------------- #
    # Unified Dual-Redundant Packet Processor
    # ---------------------------------------------------------------------- #
    def on_radio(self, data):
        """Callback from XBeeLink background thread."""
        try:
            self.process_packet(data, source='XBEE')
        except Exception as e:
            self.get_logger().debug(f'[on_radio] Error: {e}')

    def process_packet(self, data: bytes, source: str):
        """Unified packet processor with XBee Priority & Out-of-Order / Stale Rejection.
        source: 'XBEE' or 'UDP'
        """
        if not data:
            return

        now = time.monotonic()
        with self.lock:
            if source == 'XBEE':
                self.last_xbee_rx = now
            elif source == 'UDP':
                self.last_udp_rx = now

            xbee_alive = (self.last_xbee_rx is not None) and ((now - self.last_xbee_rx) < self.xbee_failover_timeout)

            # -------------------------------------------------------------
            # 1. RTCM3 GPS Corrections (starts with 0xD3)
            # -------------------------------------------------------------
            if data[0] == 0xD3 or data.startswith(b'\xd3'):
                # XBee is priority; if XBee is dead, switch to UDP RTCM
                if source == 'XBEE' or not xbee_alive:
                    self._handle_rtcm(data, source=source)
                return

            # -------------------------------------------------------------
            # 2. Drive & Arm Motion Packets ('M' or 'C')
            # -------------------------------------------------------------
            m = unpack(data)
            if not m:
                return

            seq = m[1]

            # LATENCY & STALE PACKET REJECTION (Circular Sequence Arithmetic):
            # 0 < diff < 128 means packet is strictly NEWER.
            if self.last_executed_seq is not None:
                diff = (seq - self.last_executed_seq) & 0xFF
                if diff == 0:
                    # Duplicate packet received via the other link -> discard
                    self.duplicates_rejected += 1
                    return
                if diff >= 128:
                    # Stale / delayed packet that arrived out of order -> discard
                    self.stale_rejected += 1
                    return

            # CHANNEL PRIORITY SELECTION:
            # - If XBee is healthy and this packet is from UDP: ignore UDP drive, let XBee drive!
            # - If XBee has timed out / silent (>0.35s): execute UDP drive seamlessly!
            # - Science and Mode packets over UDP are ALWAYS accepted!
            if source == 'UDP' and xbee_alive and m[0] in ('motion', 'cmd'):
                return

            # Accept and execute packet
            self.last_executed_seq = seq
            self.last_rx = now
            self.active_source = source

            if m[0] == 'motion':
                _, seq, vx, wz, a1, a2, a3, wr, gr = m
                if self.seq is not None:
                    self.lost += (seq - self.seq - 1) & 0xFF

                self.seq = seq
                self.vx, self.wz = vx, wz
                self.act1, self.act2 = a1, a2
                self.act3, self.wrist = a3, wr
                self.gripper = gr
                self.get_logger().info(
                    f"[RX] seq={seq:3d} | drive: vx={vx:+.3f}  wz={wz:+.3f} | "
                    f"arm: act1={int(a1):+d}  act2={int(a2):+d}  act3={int(a3):+d}  "
                    f"wrist={int(wr):+d}  gripper={int(gr):+d}"
                )

            elif m[0] == 'cmd':
                _, seq, vx, wz = m
                if self.seq is not None:
                    self.lost += (seq - self.seq - 1) & 0xFF

                self.seq = seq
                self.vx, self.wz = vx, wz

            elif m[0] == 'science':
                _, seq, m1, m2, m3, m4, m5, led, servo = m
                msg = UInt8MultiArray()
                msg.data = [int(m1), int(m2), int(m3), int(m4), int(m5), int(led), int(servo)]
                self.science_pub.publish(msg)
                self.get_logger().info(
                    f"🔬 [SCIENCE CMD via {source}] M1:{m1} M2:{m2} M3:{m3} M4:{m4} M5:{m5} LED:{led} SERVO:{servo}°"
                )

            elif m[0] == 'mode':
                _, seq, mode_val = m
                mode_str = "AUTONOMOUS" if int(mode_val) == 1 else "TELEOP"
                self.current_mode = mode_str
                self.mode_pub.publish(String(data=mode_str))
                self.get_logger().info(f"⚙️ [ROVER MODE via {source}] Switched to {mode_str}")

    def _handle_rtcm(self, data: bytes, source: str = 'XBEE'):
        """Publish incoming RTCM3 binary bytes to /rtcm topic for gps_node with deduplication."""
        if hasattr(self, 'rtcm_pub') and self.rtcm_pub:
            try:
                # Deduplication check for RTCM packets arriving from both channels
                content_hash = hash(data[:32])
                now = time.monotonic()
                if content_hash in self._recent_rtcm_hashes:
                    return
                self._recent_rtcm_hashes.append(content_hash)
                if len(self._recent_rtcm_hashes) > 64:
                    self._recent_rtcm_hashes.pop(0)

                if RTCMMessage:
                    msg = RTCMMessage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.message = list(data)
                    self.rtcm_pub.publish(msg)
                else:
                    msg = UInt8MultiArray()
                    msg.data = list(data)
                    self.rtcm_pub.publish(msg)
                self.rtcm_count += 1
                if now - self._last_rtcm_log > 2.0:
                    self.get_logger().info(f"🛰️ [RTCM RX via {source}] Injected {len(data)} bytes -> published to /rtcm (Total: {self.rtcm_count})")
                    self._last_rtcm_log = now
            except Exception as e:
                self.get_logger().debug(f"RTCM publish error: {e}")

    def on_telemetry_tx(self, msg):
        """Receive telemetry from rover_telemetry node and transmit over XBee radio."""
        try:
            payload = msg.data.encode('utf-8') if isinstance(msg.data, str) else bytes(msg.data)
            if not payload.startswith(b'T'):
                payload = b'T' + payload
            self.link.send(payload)
        except Exception:
            pass

    # ---------------------------------------------------------------------- #
    # Helpers
    # ---------------------------------------------------------------------- #
    def _age(self):
        """Seconds since the last valid packet, or None if never received."""
        return None if self.last_rx is None else time.monotonic() - self.last_rx

    def _reconnect(self):
        """Auto-reconnect the XBee link and rebind UDP socket if either has dropped."""
        try:
            if not self.link.connected:
                err_str = f" ({self.link.err})" if self.link.err else ""
                self.get_logger().warn(f'[rx_node] XBee not connected on {self.link.port}{err_str} — retrying...')
                self.link.connect()
        except Exception as e:
            self.get_logger().debug(f'[rx_node] XBee reconnect error: {e}')

        # Restart UDP receiver thread if it has died
        try:
            if self.udp_running:
                thread_dead = not hasattr(self, 'udp_thread') or not self.udp_thread.is_alive()
                sock_dead   = self.udp_sock is None
                if thread_dead or sock_dead:
                    self.get_logger().warn('[rx_node] UDP receiver not running — rebinding...')
                    self._init_udp_receiver()
        except Exception as e:
            self.get_logger().debug(f'[rx_node] UDP watchdog error: {e}')

    # ---------------------------------------------------------------------- #
    # Publish drive + arm commands (20 Hz)
    # ---------------------------------------------------------------------- #
    def publish_cmd(self):
        """Publish /cmd_vel and /arm_vel. Publishes zeros on both if link is stale."""
        try:
            with self.lock:
                age    = self._age()
                fresh  = age is not None and age < self.get_parameter('cmd_timeout').value
                vx     = self.vx     if fresh else 0.0
                wz     = self.wz     if fresh else 0.0
                act1   = self.act1   if fresh else 0.0
                act2   = self.act2   if fresh else 0.0
                act3   = self.act3   if fresh else 0.0
                wrist  = self.wrist  if fresh else 0.0
                gripper= self.gripper if fresh else 0.0

            # Drive command
            drive_msg = Twist()
            drive_msg.linear.x  = vx
            drive_msg.angular.z = wz
            self.drive_pub.publish(drive_msg)

            # Arm command: [act1, act2, act3, wrist, gripper]
            arm_msg = Float32MultiArray()
            arm_msg.data = [act1, act2, act3, wrist, gripper]
            self.arm_pub.publish(arm_msg)
        except Exception as e:
            self.get_logger().debug(f'[publish_cmd] Error: {e}')

    # ---------------------------------------------------------------------- #
    # Send ACK back to the base station (2 Hz)
    # ---------------------------------------------------------------------- #
    def send_ack(self):
        """Send an ACK packet to the base station so it can track packet loss (non-blocking)."""
        try:
            if self.link.connected and self.seq is not None:
                dest = self.link.last_src or self.link.remote_addr
                self.link.send(pack_ack(self.seq, self.lost), dest_addr=dest)
        except Exception as e:
            self.get_logger().debug(f'[send_ack] Error: {e}')

    # ---------------------------------------------------------------------- #
    # Publish diagnostic status as JSON (5 Hz)
    # ---------------------------------------------------------------------- #
    def publish_status(self):
        try:
            now = time.monotonic()
            t0, n0 = self._last
            rate = (self.link.rx - n0) / max(now - t0, 1e-3)
            self._last = (now, self.link.rx)

            with self.lock:
                s = dict(
                    # Link
                    link=self.link.connected,
                    port=self.link.port,
                    baud=self.link.baud,
                    local=self.link.local_addr,
                    remote=self.link.remote_addr,
                    err=self.link.err,
                    # Reception stats
                    rx=self.link.rx,
                    ignored=self.link.rx_ignored,
                    last_src=self.link.last_src,
                    rate=round(rate, 1),
                    age=self._age(),
                    lost=self.lost,
                    seq=self.seq,
                    # Dual-Redundancy & Failover
                    active_source=self.active_source,
                    mode=self.current_mode,
                    xbee_alive=(self.last_xbee_rx is not None and (now - self.last_xbee_rx < self.xbee_failover_timeout)),
                    udp_alive=(self.last_udp_rx is not None and (now - self.last_udp_rx < self.xbee_failover_timeout)),
                    stale_rejected=self.stale_rejected,
                    duplicates_rejected=self.duplicates_rejected,
                    # RTCM GPS corrections
                    rtcm_count=getattr(self, 'rtcm_count', 0),
                    last_rtcm_age=None if getattr(self, '_last_rtcm_log', 0.0) == 0.0 else round(time.monotonic() - self._last_rtcm_log, 1),
                    # Drive
                    vx=self.vx,
                    wz=self.wz,
                    # Arm
                    act1=self.act1,
                    act2=self.act2,
                    act3=self.act3,
                    wrist=self.wrist,
                    gripper=self.gripper,
                )
            self.status_pub.publish(String(data=json.dumps(s)))
        except Exception as e:
            self.get_logger().debug(f'[publish_status] Error: {e}')


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = RxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety: send zeros before shutting down
        node.get_logger().info('[rx_node] Shutting down — publishing zero commands.')
        if rclpy.ok():
            node.drive_pub.publish(Twist())
            arm_zero = Float32MultiArray()
            arm_zero.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            node.arm_pub.publish(arm_zero)
        node.link.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

