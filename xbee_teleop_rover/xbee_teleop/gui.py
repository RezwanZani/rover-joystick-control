#!/usr/bin/env python3
"""PyQt5 status window: joystick, command, XBee TX, XBee RX.
Reads /joy, xbee_tx/status, xbee_rx/status (run it next to tx_node and/or rx_node)."""
import json
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QPainter
from PyQt5.QtWidgets import QApplication, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QWidget

GREEN, RED, GREY, AMBER = '#2ecc71', '#e74c3c', '#7f8c8d', '#f1c40f'


class State(Node):
    """Latest message of each topic + arrival time (ROS thread writes, Qt reads)."""

    def __init__(self):
        super().__init__('xbee_gui')
        self.data = dict(joy=None, tx=None, rx=None)
        self.t = dict(joy=0.0, tx=0.0, rx=0.0)
        self.create_subscription(Joy, 'joy', lambda m: self._set('joy', m), 10)
        self.create_subscription(String, 'xbee_tx/status', lambda m: self._set('tx', json.loads(m.data)), 10)
        self.create_subscription(String, 'xbee_rx/status', lambda m: self._set('rx', json.loads(m.data)), 10)

    def _set(self, k, v):
        self.data[k] = v
        self.t[k] = time.monotonic()

    def get(self, k, within=1.5):
        """Latest message, or None if it is missing / stale."""
        return self.data[k] if time.monotonic() - self.t[k] < within else None


class Bar(QWidget):
    """Horizontal bar, zero in the middle, value in [-1, 1]."""

    def __init__(self):
        super().__init__()
        self.v = 0.0
        self.setMinimumSize(150, 16)

    def set(self, v):
        self.v = max(-1.0, min(1.0, v))
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor('#2c3e50'))
        p.fillRect(w // 2, 0, int(self.v * w / 2), h, QColor('#3498db'))
        p.fillRect(w // 2 - 1, 0, 2, h, QColor('#ecf0f1'))


class Panel(QGroupBox):
    """Group box with a status LED + headline, then 'name: value' rows."""

    def __init__(self, title, rows):
        super().__init__(title)
        form = QFormLayout(self)
        self.led, self.head = QLabel(), QLabel()
        self.led.setFixedSize(16, 16)
        form.addRow(self.led, self.head)
        self.v = {}
        for key, name in rows:
            self.v[key] = QLabel('-')
            self.v[key].setTextInteractionFlags(Qt.TextSelectableByMouse)
            form.addRow(name + ':', self.v[key])
        self.state(GREY, '')

    def state(self, color, text):
        self.led.setStyleSheet(f'background:{color}; border-radius:8px; border:1px solid #222;')
        self.head.setText(text)

    def clear(self):
        for lb in self.v.values():
            lb.setText('-')

    def put(self, **kw):
        for k, t in kw.items():
            self.v[k].setText(str(t))


class Window(QWidget):
    def __init__(self, st):
        super().__init__()
        self.st = st
        self.setWindowTitle('XBee teleop')
        grid = QGridLayout(self)

        self.joy = Panel('Joystick', [('deadman', 'Deadman (hold to drive)'), ('btn', 'Buttons pressed')])
        for name in ('stick_x', 'stick_y'):          # raw left stick, handy to check axis numbers
            setattr(self, name, Bar())
        self.joy.layout().addRow('Axis 0 (x):', self.stick_x)
        self.joy.layout().addRow('Axis 1 (y):', self.stick_y)

        self.cmd = Panel('Command', [('vx', 'Linear vx (m/s)'), ('wz', 'Angular wz (rad/s)')])
        self.vx_bar, self.wz_bar = Bar(), Bar()
        self.cmd.layout().addRow('vx:', self.vx_bar)
        self.cmd.layout().addRow('wz:', self.wz_bar)

        self.tx = Panel('XBee TX', [('port', 'Port'), ('local', 'This radio'), ('remote', 'Sending to'),
                                    ('sent', 'Frames sent'), ('rate', 'Rate'),
                                    ('ok', 'Delivered / failed'), ('err', 'Error')])
        self.rx = Panel('XBee RX', [('src', 'Last source'), ('n', 'Frames received'), ('rate', 'Rate'),
                                    ('age', 'Last cmd age'), ('lost', 'Lost frames'),
                                    ('cmd', 'Command received'), ('err', 'Info')])
        for i, w in enumerate((self.joy, self.cmd, self.tx, self.rx)):
            grid.addWidget(w, i // 2, i % 2)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def refresh(self):
        joy, tx, rx = self.st.get('joy'), self.st.get('tx'), self.st.get('rx')

        # --- Joystick ---
        if joy is not None:
            self.joy.state(GREEN, 'connected (/joy)')
            pressed = ', '.join(str(i) for i, b in enumerate(joy.buttons) if b) or '-'
            self.joy.put(btn=pressed)
            ax = list(joy.axes) + [0.0, 0.0]
            self.stick_x.set(ax[0])
            self.stick_y.set(ax[1])
        else:
            self.joy.state(RED, 'no /joy data')
            self.joy.put(btn='-')
            self.stick_x.set(0)
            self.stick_y.set(0)
        self.joy.put(deadman='-' if tx is None else ('HELD' if tx['enable'] else 'released'))

        # --- Command + XBee TX ---
        if tx is None:
            self.cmd.state(GREY, 'tx_node not running')
            self.tx.state(GREY, 'tx_node not running')
            self.cmd.clear()
            self.tx.clear()
            self.vx_bar.set(0)
            self.wz_bar.set(0)
        else:
            moving = abs(tx['vx']) > 1e-3 or abs(tx['wz']) > 1e-3
            self.cmd.state(GREEN if moving else AMBER, 'driving' if moving else 'zero / stopped')
            self.cmd.put(vx=f"{tx['vx']:+.2f}", wz=f"{tx['wz']:+.2f}")
            self.vx_bar.set(tx['vx'])
            self.wz_bar.set(tx['wz'])
            sending = tx['link'] and tx['rate'] > 0.5
            self.tx.state(GREEN if sending else (AMBER if tx['link'] else RED),
                          'sending' if sending else ('radio open, idle' if tx['link'] else 'radio not open'))
            self.tx.put(port=f"{tx['port']} @ {tx['baud']}", local=tx['local'] or '-', remote=tx['remote'],
                        sent=tx['sent'], rate=f"{tx['rate']:.1f} Hz", ok=f"{tx['tx_ok']} / {tx['tx_fail']}",
                        err=tx['err'] or '-')

        # --- XBee RX: local rx_node if running here, else the robot's ACK reported to tx_node ---
        ack = tx['ack'] if tx and tx['ack'] and tx['ack']['age'] < 2.0 else None
        if rx is not None:
            alive = rx['age'] is not None and rx['age'] < 0.5
            self.rx.state(GREEN if alive else (AMBER if rx['link'] else RED),
                          'receiving' if alive else ('radio open, no data' if rx['link'] else 'radio not open'))
            self.rx.put(src=rx['last_src'] or '-', n=f"{rx['rx']}  (ignored: {rx['ignored']})",
                        rate=f"{rx['rate']:.1f} Hz", age='-' if rx['age'] is None else f"{rx['age'] * 1000:.0f} ms",
                        lost=rx['lost'], cmd=f"vx {rx['vx']:+.2f}  wz {rx['wz']:+.2f}",
                        err=rx['err'] or ('wrong source address!' if rx['ignored'] else '-'))
        elif ack is not None:
            self.rx.state(GREEN, 'robot is receiving (via ACK)')
            self.rx.clear()
            self.rx.put(n=f"last seq {ack['seq']}", age=f"{ack['age'] * 1000:.0f} ms (ACK)", lost=ack['lost'])
        else:
            waiting = tx is not None and tx['link']
            self.rx.state(RED if waiting else GREY, 'no ACK from robot' if waiting else 'no rx_node data')
            self.rx.clear()


def main(args=None):
    rclpy.init(args=args)
    st = State()
    threading.Thread(target=rclpy.spin, args=(st,), daemon=True).start()
    app = QApplication(sys.argv)
    win = Window(st)
    win.show()
    code = app.exec_()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
