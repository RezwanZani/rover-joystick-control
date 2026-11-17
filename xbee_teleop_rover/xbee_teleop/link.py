"""XBee link (radio must be in API mode, AP=1) + packet helpers.

Packet types
------------
  CMD    TX -> RX (legacy, 'C') : seq u8, vx i16, wz i16
  MOTION TX -> RX (new,    'M') : seq u8, vx i16, wz i16,
                                   act1 i16, act2 i16, act3 i16, wrist i16, gripper i16
  ACK    RX -> TX ('A')         : last seq u8, lost u16

All i16 values are stored as (real_value * 1000), giving ±32.767 m/s / rad/s / units.
Arm channels use discrete steps: -1000 (contract), 0 (still), +1000 (expand).
"""
import struct

from digi.xbee.devices import RemoteXBeeDevice, XBeeDevice
from digi.xbee.models.address import XBee64BitAddress
from digi.xbee.models.status import TransmitStatus
from digi.xbee.packets.common import TransmitStatusPacket

# ---------------------------------------------------------------------------
# Struct layouts (little-endian)
# ---------------------------------------------------------------------------

# Legacy drive-only packet: 'C' seq vx wz  (6 bytes)
_CMD = struct.Struct('<cBhh')

# New combined packet: 'M' seq vx wz act1 act2 act3 wrist gripper  (16 bytes)
#   act1/act2/act3 = three linear actuators
#   wrist          = wrist stepper motor
#   gripper        = gripper gear motor
_MOTION = struct.Struct('<cBhhhhhhh')

# ACK from rover to base station: 'A' last_seq lost  (5 bytes)
_ACK = struct.Struct('<cBH')


# ---------------------------------------------------------------------------
# Helper: clamp a float to a signed 16-bit integer (×1000 fixed-point)
# ---------------------------------------------------------------------------
def _i16(v):
    """Scale a float by 1000 and clamp to int16 range."""
    return max(-32768, min(32767, int(round(v * 1000))))


# ---------------------------------------------------------------------------
# Pack helpers
# ---------------------------------------------------------------------------
def pack_cmd(seq, vx, wz):
    """Pack a legacy drive-only CMD packet (kept for backward compatibility)."""
    return _CMD.pack(b'C', seq & 0xFF, _i16(vx), _i16(wz))


def pack_motion(seq, vx, wz, act1, act2, act3, wrist, gripper):
    """Pack the combined drive + arm MOTION packet.

    Drive:  vx (m/s), wz (rad/s)
    Arm:    act1, act2, act3  = linear actuators  (-1 contract / 0 still / +1 expand)
            wrist             = wrist stepper      (-1 / 0 / +1)
            gripper           = gripper gear motor (-1 / 0 / +1)
    """
    return _MOTION.pack(
        b'M', seq & 0xFF,
        _i16(vx), _i16(wz),
        _i16(act1), _i16(act2), _i16(act3),
        _i16(wrist), _i16(gripper),
    )


def pack_ack(seq, lost):
    """Pack an ACK from the rover back to the base station."""
    return _ACK.pack(b'A', seq & 0xFF, lost & 0xFFFF)


# ---------------------------------------------------------------------------
# Unpack (handles all packet types)
# ---------------------------------------------------------------------------
def unpack(data):
    """Decode an incoming XBee payload.

    Returns one of:
      ('motion', seq, vx, wz, act1, act2, act3, wrist, gripper)  -- new combined packet
      ('cmd',    seq, vx, wz)                                     -- legacy drive-only
      ('ack',    seq, lost)                                        -- rover ACK
      None                                                         -- unknown / corrupt
    """
    data = bytes(data)

    # New combined MOTION packet
    if data[:1] == b'M' and len(data) == _MOTION.size:
        _, seq, vx, wz, a1, a2, a3, wr, gr = _MOTION.unpack(data)
        return ('motion', seq,
                vx / 1000.0, wz / 1000.0,
                a1 / 1000.0, a2 / 1000.0, a3 / 1000.0,
                wr / 1000.0, gr / 1000.0)

    # Legacy drive-only CMD packet (backward compatibility)
    if data[:1] == b'C' and len(data) == _CMD.size:
        _, seq, vx, wz = _CMD.unpack(data)
        return ('cmd', seq, vx / 1000.0, wz / 1000.0)

    # ACK from rover
    if data[:1] == b'A' and len(data) == _ACK.size:
        _, seq, lost = _ACK.unpack(data)
        return ('ack', seq, lost)

    return None


def norm_addr(a):
    return str(a).strip().upper().replace('0X', '').zfill(16)


def get_controller_state(buttons, raw_axes, hats=None):
    """
    Parses buttons, triggers, and D-Pad hats across all controller types:
      - Linux generic HID pads (e.g. 19 buttons: Zikway HID gamepad, BTN_WEST for Y, etc.)
      - Linux standard xpad / xboxdrv (11-15 buttons)
      - Windows direct XInput (10 buttons)

    Returns:
      btns: dict with booleans 'A', 'B', 'X', 'Y', 'LB', 'RB', 'LT', 'RT', 'UP', 'DOWN'
      contract: bool (LT pulled > -0.4 OR LB pressed OR LT digital button)
      expand:   bool (RT pulled > -0.4 OR RB pressed OR RT digital button)
      gripper:  float (-1.0 for UP/contract, +1.0 for DOWN/expand, 0.0 for still)
    """
    n = len(buttons)
    g = lambda i: bool(buttons[i]) if 0 <= i < n else False

    if n > 12:
        btn_A  = g(0)
        btn_B  = g(1)
        btn_X  = g(3) or g(2)
        btn_Y  = g(4)
        btn_LB = g(6)
        btn_RB = g(7)
        btn_LT = g(8)
        btn_RT = g(9)
    else:
        btn_A  = g(0)
        btn_B  = g(1)
        btn_X  = g(2)
        btn_Y  = g(3)
        btn_LB = g(4)
        btn_RB = g(5)
        btn_LT = g(8) if n > 10 else False
        btn_RT = g(9) if n > 10 else False

    lt_axis = raw_axes[4] if len(raw_axes) > 4 else -1.0
    rt_axis = raw_axes[5] if len(raw_axes) > 5 else -1.0
    lt_pulled = (lt_axis > -0.4)
    rt_pulled = (rt_axis > -0.4)

    contract = lt_pulled or btn_LB or btn_LT
    expand   = rt_pulled or btn_RB or btn_RT

    gripper = 0.0
    dpad_up = False
    dpad_down = False

    if hats and len(hats) > 0:
        hy = hats[0][1]
        if hy > 0:
            dpad_up = True
            gripper = -1.0
        elif hy < 0:
            dpad_down = True
            gripper = 1.0
    elif len(raw_axes) > 7:
        dpad_y = raw_axes[7]
        if dpad_y < -0.5:
            dpad_up = True
            gripper = -1.0
        elif dpad_y > 0.5:
            dpad_down = True
            gripper = 1.0

    if not dpad_up and not dpad_down and n > 14:
        if g(11) or g(13):
            dpad_up = True
            gripper = -1.0
        elif g(12) or g(14):
            dpad_down = True
            gripper = 1.0

    btns = dict(
        A=btn_A, B=btn_B, X=btn_X, Y=btn_Y,
        LB=btn_LB, RB=btn_RB,
        LT=btn_LT or lt_pulled, RT=btn_RT or rt_pulled,
        UP=dpad_up, DOWN=dpad_down
    )

    return btns, contract, expand, gripper


class XBeeLink:
    def __init__(self, port, baud, remote_addr, on_message):
        self.port, self.baud = port, int(baud)
        self.remote_addr = norm_addr(remote_addr)
        self.on_message = on_message            # on_message(bytes), called from the XBee thread
        self.dev = self.remote = None
        self.local_addr = ''
        self.err = ''
        self.sent = self.tx_ok = self.tx_fail = 0   # frames sent / delivered / failed
        self.rx = self.rx_ignored = 0               # frames accepted / from a wrong address
        self.last_src = ''

    @property
    def connected(self):
        return self.dev is not None

    def connect(self):
        try:
            dev = XBeeDevice(self.port, self.baud)
            dev.open()
            dev.set_sync_ops_timeout(1.0)
            dev.add_data_received_callback(self._on_data)
            dev.add_packet_received_callback(self._on_packet)
            self.remote = RemoteXBeeDevice(dev, XBee64BitAddress.from_hex_string(self.remote_addr))
            self.local_addr = str(dev.get_64bit_addr())
            self.dev, self.err = dev, ''
        except Exception as e:  # noqa: BLE001 - show any reason in the GUI
            msg = str(e) or type(e).__name__
            if 'operating mode' in msg.lower():
                msg += '  -> radio not in API mode: run  ros2 run xbee_teleop xbee_check <port> <baud> --fix'
            self.err, self.dev = msg, None

    def send(self, payload):
        if not self.dev:
            return False
        try:
            self.dev.send_data_async(self.remote, payload)
            self.sent += 1
            return True
        except Exception as e:  # noqa: BLE001
            self.err = f'send failed: {e}'
            self.close()
            return False

    def _on_data(self, msg):
        src = str(msg.remote_device.get_64bit_addr())
        self.last_src = src
        if src != self.remote_addr:
            self.rx_ignored += 1
            return
        self.rx += 1
        self.on_message(bytes(msg.data))

    def _on_packet(self, pkt):
        if isinstance(pkt, TransmitStatusPacket):
            if pkt.transmit_status == TransmitStatus.SUCCESS:
                self.tx_ok += 1
            else:
                self.tx_fail += 1

    def close(self):
        dev, self.dev = self.dev, None
        try:
            if dev and dev.is_open():
                dev.close()
        except Exception:  # noqa: BLE001
            pass
