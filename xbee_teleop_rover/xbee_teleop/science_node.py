#!/usr/bin/env python3
"""
science_node.py -- ROS 2 to Arduino Mega Serial Bridge for Rover Science Mission.

Subscribes to:
  /science/actuator_cmd   (std_msgs/UInt8MultiArray) -- [m1, m2, m3, m4, m5, led, servo]

Publishes to:
  /science/actuator_status (std_msgs/String) -- JSON feedback from Arduino Mega

Hardware:
  Arduino Mega 2560 controlling 5 peristaltic pumps, 1 LED, and 1 servo.
"""

import json
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray

# Default Arduino Mega parameters (override via params.yaml or ROS launch)
SCIENCE_ARDUINO_PORT = '/dev/ttyACM0'
SCIENCE_ARDUINO_BAUD = 9600
SCIENCE_CMD_TOPIC = '/science/actuator_cmd'
SCIENCE_STATUS_TOPIC = '/science/actuator_status'


class ScienceNode(Node):
    def __init__(self, *args, **kwargs):
        super().__init__('science_node', *args, **kwargs)

        self.declare_parameter('port', SCIENCE_ARDUINO_PORT)
        self.declare_parameter('baud', SCIENCE_ARDUINO_BAUD)
        self.declare_parameter('cmd_topic', SCIENCE_CMD_TOPIC)
        self.declare_parameter('status_topic', SCIENCE_STATUS_TOPIC)

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)

        self.get_logger().info(
            f"🔬 ScienceNode initializing...\n"
            f"   Target Port: {self.port} @ {self.baud} baud\n"
            f"   Subscribing: {self.cmd_topic}\n"
            f"   Publishing:  {self.status_topic}"
        )

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(UInt8MultiArray, self.cmd_topic, self.on_cmd, 10)

        self.lock = threading.Lock()
        self.ser = None
        self.running = True
        self.last_status = {
            "m1": 0, "m2": 0, "m3": 0, "m4": 0, "m5": 0,
            "led": 0, "servo": 0, "connected": False
        }

        # Serial background reader thread
        self.reader_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.reader_thread.start()

        # 2 Hz status publisher timer (ensures UI gets periodic updates)
        self.create_timer(0.5, self._publish_latest_status)

    def _connect(self):
        import serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"✅ Connected to Science Arduino Mega on {self.port} @ {self.baud}")
            with self.lock:
                self.last_status["connected"] = True
            return True
        except Exception as e:
            self.ser = None
            with self.lock:
                self.last_status["connected"] = False
            return False

    def on_cmd(self, msg: UInt8MultiArray):
        """Receive command from rx_node and forward to Arduino over serial.
        Does NOT update status optimistically; actual status is only updated
        when confirmed by the Arduino Mega over serial.
        """
        if not msg.data or len(msg.data) < 6:
            return

        m1 = int(msg.data[0])
        m2 = int(msg.data[1])
        m3 = int(msg.data[2])
        m4 = int(msg.data[3])
        m5 = int(msg.data[4])
        led = int(msg.data[5])
        servo = int(msg.data[6]) if len(msg.data) > 6 else 0

        cmd_str = f"S:{m1},{m2},{m3},{m4},{m5},{led},{servo}\n"

        with self.lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(cmd_str.encode('ascii'))
                    self.get_logger().info(f"Sent to Arduino Mega: {cmd_str.strip()}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to write to Arduino: {e}")
            else:
                self.get_logger().warn("Arduino Mega not connected — command not sent.")

    def _serial_loop(self):
        """Background thread to read serial responses from Arduino Mega."""
        last_conn_try = 0
        buf = ""

        while self.running:
            if not self.ser or not self.ser.is_open:
                now = time.time()
                if now - last_conn_try > 3.0:
                    last_conn_try = now
                    self._connect()
                time.sleep(0.5)
                continue

            try:
                raw = self.ser.read(64)
                if raw:
                    buf += raw.decode('ascii', errors='ignore')
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        line = line.strip()
                        if line:
                            self._parse_arduino_line(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self.ser = None
                with self.lock:
                    self.last_status["connected"] = False
                time.sleep(1.0)

    def _parse_arduino_line(self, line: str):
        """Parses: STATUS:m1=1,m2=0,m3=1,m4=0,m5=0,led=1,servo=90"""
        if line.startswith("STATUS:"):
            content = line[7:].strip()
            parts = content.split(',')
            parsed = {}
            for part in parts:
                if '=' in part:
                    k, v = part.split('=', 1)
                    k = k.strip().lower()
                    try:
                        parsed[k] = int(v.strip())
                    except ValueError:
                        pass

            with self.lock:
                for k in ["m1", "m2", "m3", "m4", "m5", "led", "servo"]:
                    if k in parsed:
                        self.last_status[k] = parsed[k]
                self.last_status["connected"] = True
                self.last_status["timestamp"] = time.time()

            # Publish status immediately upon receipt
            self._publish_latest_status()

    def _publish_latest_status(self):
        with self.lock:
            payload = dict(self.last_status)
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        with self.lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(b"S:0,0,0,0,0,0,0\nz\n")
                    self.ser.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScienceNode()
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
