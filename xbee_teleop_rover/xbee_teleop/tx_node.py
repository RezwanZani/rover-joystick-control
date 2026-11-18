#!/usr/bin/env python3
"""TX side: /joy -> drive + arm commands -> XBee. Publishes JSON status on xbee_tx/status."""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from xbee_teleop.link import XBeeLink, pack_motion, unpack, get_controller_state


class TxNode(Node):
    def __init__(self):
        super().__init__('tx_node')
        for k, v in dict(
            port='/dev/ttyUSB0', baud=115200, remote_addr='', send_rate_hz=20.0,
            cmd_timeout=0.5, axis_linear=1, axis_angular=2, enable_button=4,
            scale_linear=0.5, scale_angular=1.0, require_enable=False
        ).items():
            self.declare_parameter(k, v)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.link = XBeeLink(p('port'), p('baud'), p('remote_addr'), self.on_radio)
        self.vx = self.wz = 0.0
        self.arm = (0.0, 0.0, 0.0, 0.0, 0.0)
        self.enable = False
        self.joy_time = 0.0
        self.seq = 0
        self.ack = None                      # (time, seq, lost) reported by the robot
        self.lock = threading.Lock()
        self._last = (time.monotonic(), 0)

        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.status_pub = self.create_publisher(String, 'xbee_tx/status', 10)
        self.create_timer(1.0 / p('send_rate_hz'), self.send)
        self.create_timer(0.2, self.publish_status)
        self.create_timer(2.0, lambda: None if self.link.connected else self.link.connect())
        self.link.connect()

    def on_joy(self, msg):
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        g = lambda seq, i: seq[i] if 0 <= i < len(seq) else 0  # noqa: E731

        btns, contract, expand, gripper = get_controller_state(msg.buttons, msg.axes)

        axis_lin = p('axis_linear')
        axis_ang = p('axis_angular')
        # Standard Linux xpad Xbox pad has <= 12 buttons and > 5 axes (axis 2 is LT trigger, axis 3 is RX).
        # Generic HID pads (e.g. 19-button Zikway) have axis 2 as RX.
        if axis_ang == 2 and len(msg.buttons) <= 12 and len(msg.axes) > 5:
            axis_ang = 3

        self.enable = bool(g(msg.buttons, p('enable_button'))) or btns['LB']
        if self.enable or not p('require_enable'):
            # Left throttle (Left Stick Y) strictly controls front and back speed (vx); left/right is ignored.
            self.vx = p('scale_linear') * g(msg.axes, axis_lin)
            # Right throttle (Right Stick X) strictly controls left/right turning (wz); front/back is ignored.
            self.wz = p('scale_angular') * g(msg.axes, axis_ang)
        else:
            self.vx = self.wz = 0.0

        def _ch(is_held):
            if not is_held: return 0.0
            if contract and not expand: return -1.0
            if expand and not contract: return 1.0
            return 0.0

        self.arm = (
            _ch(btns['A']),
            _ch(btns['B']),
            _ch(btns['X']),
            _ch(btns['Y']),
            gripper
        )
        self.joy_time = time.monotonic()

    def send(self):
        fresh = time.monotonic() - self.joy_time < self.get_parameter('cmd_timeout').value
        vx, wz = (self.vx, self.wz) if fresh else (0.0, 0.0)
        act1, act2, act3, wrist, gripper = self.arm if fresh else (0.0, 0.0, 0.0, 0.0, 0.0)
        if self.link.send(pack_motion(self.seq, vx, wz, act1, act2, act3, wrist, gripper)):
            self.seq = (self.seq + 1) & 0xFF

    def on_radio(self, data):
        m = unpack(data)
        if m and m[0] == 'ack':
            with self.lock:
                self.ack = (time.monotonic(), m[1], m[2])

    def publish_status(self):
        now = time.monotonic()
        t0, n0 = self._last
        rate = (self.link.sent - n0) / max(now - t0, 1e-3)
        self._last = (now, self.link.sent)
        with self.lock:
            ack = self.ack
        act1, act2, act3, wrist, gripper = self.arm
        s = dict(
            link=self.link.connected, port=self.link.port, baud=self.link.baud,
            local=self.link.local_addr, remote=self.link.remote_addr, err=self.link.err,
            sent=self.link.sent, rate=round(rate, 1), tx_ok=self.link.tx_ok,
            tx_fail=self.link.tx_fail, vx=self.vx, wz=self.wz, enable=self.enable,
            act1=act1, act2=act2, act3=act3, wrist=wrist, gripper=gripper,
            ack=None if ack is None else dict(age=now - ack[0], seq=ack[1], lost=ack[2])
        )
        self.status_pub.publish(String(data=json.dumps(s)))


def main(args=None):
    rclpy.init(args=args)
    node = TxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.link.send(pack_motion(node.seq, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        node.link.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()