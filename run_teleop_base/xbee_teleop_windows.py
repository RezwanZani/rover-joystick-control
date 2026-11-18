#!/usr/bin/env python3
"""
xbee_teleop_windows.py -- Windows TRANSMITTER for the XBee teleop system (no ROS needed).

  gamepad -> drive + arm commands -> XBee (API mode, AP=1) -> rover's rx_node (ROS 2)
  GUI: Joystick | Command | Arm Control | XBee TX | Robot ACK panels, plus a Radio Check tab.

Drive commands  : left stick Y = forward/back (vx), right stick X = turn (wz)  -- UNCHANGED
Arm commands    : published together with drive in a single MOTION packet ('M') at 20 Hz

Arm control mapping (Xbox pad):
  A button + LT       -> Actuator 1 contract  (-1)
  A button + RT       -> Actuator 1 expand    (+1)
  B button + LT       -> Actuator 2 contract  (-1)
  B button + RT       -> Actuator 2 expand    (+1)
  X button + LT       -> Actuator 3 contract  (-1)
  X button + RT       -> Actuator 3 expand    (+1)
  Y button + LT       -> Wrist contract       (-1)
  Y button + RT       -> Wrist expand         (+1)
  D-Pad Up            -> Gripper contract     (-1)
  D-Pad Down          -> Gripper expand       (+1)
  Releasing a button  -> that channel stays still (0)

Packet format: 'M', seq u8, vx i16, wz i16, act1 i16, act2 i16, act3 i16, wrist i16, gripper i16
               All i16 values are real_value * 1000  (e.g. 1.0 -> 1000)

Run:  pip install pygame PyQt5 digi-xbee pyserial   then   python xbee_teleop_windows.py
"""
import ctypes
import json
import os
import sys
import struct
import threading
import time

os.environ.setdefault('SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS', '1')   # keep reading the pad when the window is unfocused
os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')

# --------------------------------------------------------------------------
# CONFIG — Standalone & fully independent configuration (no comms_config required)
# --------------------------------------------------------------------------

# Hardware Serial Ports
TELEOP_XBEE_PORT = '/dev/ttyUSB0' if sys.platform != 'win32' else 'COM3'
XBEE_BAUD_RATE = 115200
AVAILABLE_BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

# Target Rover Radio 64-bit Address
ROBOT_RADIO_ADDR = '0013A200425E94FF'

# Controller Settings
JOY_INDEX = 0
JOY_DEADZONE = 0.05
SEND_RATE_HZ = 20.0
CMD_TIMEOUT = 0.5

# Deadman & Drive Scales
ENABLE_BUTTON = 4          # LB — hold to enable drive (if require_enable=True)
REQUIRE_ENABLE = False     # False = stick alone drives, no need to hold LB
SCALE_LINEAR = 0.5         # multiply left-stick Y by this for vx (m/s)
SCALE_ANGULAR = 1.0        # multiply right-stick X by this for wz (rad/s)
DRIVE_MODE = 'arcade'      # 'arcade' (default) | 'tank'

# Drive axes (0=LX, 1=LY, 2=RX, 3=RY, 4=LT, 5=RT)
AXIS_LINEAR = 1            # Left Stick Y -> front/back vx
AXIS_ANGULAR = 2           # Right Stick X -> left/right wz
AXIS_LEFT = 1              # Tank drive: Left Stick Y
AXIS_RIGHT = 3             # Tank drive: Right Stick Y

# Arm Buttons & Triggers
BTN_ACT1 = 0               # A -> Actuator 1
BTN_ACT2 = 1               # B -> Actuator 2
BTN_ACT3 = 2               # X -> Actuator 3
BTN_WRIST = 3              # Y -> Wrist
AXIS_LT = 4                # LT -> Contract (-1)
AXIS_RT = 5                # RT -> Expand (+1)
TRIGGER_THRESHOLD = 0.3    # Trigger press travel to register action
AXIS_DPAD_Y = 7            # D-Pad Y synthetic axis for gripper

# UDP Hub & Master Ports (optional LAN / local interconnect)
MASTER_IP = '127.0.0.1'
UDP_JOY_PORT = 9001
UDP_STATUS_PORT = 9003

# Science & Mode Packet Protocol
SCIENCE_PACKET_FORMAT = '<cBBBBBBBB'
SCIENCE_PACKET_ID = b'S'
SCIENCE_PACKET_SIZE = 9
MODE_PACKET_FORMAT = '<cBB'
MODE_PACKET_ID = b'N'
MODE_PACKET_SIZE = 3

def scan_available_ports():
    """Scans physical hardware serial ports and USB adapters across Linux and Windows."""
    import glob
    ports = []
    def _add(p):
        if p and p not in ports:
            ports.append(p)
    if sys.platform != 'win32':
        for pattern in ['/dev/ttyUSB*', '/dev/ttyACM*', '/dev/rover_*', '/dev/serial/by-id/*', '/dev/serial/by-path/*']:
            for dev in sorted(glob.glob(pattern)):
                if os.path.exists(dev):
                    _add(dev)
    try:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            dev = p.device
            if sys.platform != 'win32' and dev.startswith('/dev/ttyS'):
                continue
            _add(dev)
    except Exception:
        pass
    if sys.platform == 'win32':
        try:
            import winreg
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
            for i in range(256):
                try:
                    val = winreg.EnumValue(key, i)
                    _add(str(val[1]))
                except OSError:
                    break
            winreg.CloseKey(key)
        except Exception:
            pass
    return ports

CONFIG = dict(
    port=TELEOP_XBEE_PORT,
    baud=XBEE_BAUD_RATE,
    joy_index=JOY_INDEX,
    joy_deadzone=JOY_DEADZONE,
    send_rate_hz=SEND_RATE_HZ,
    cmd_timeout=CMD_TIMEOUT,

    # ---- Drive axes (Xbox pad SDL/pygame axis numbers) ----
    # axes   0=LX  1=LY  2=RX  3=RY  4=LT  5=RT
    # SDL convention: stick up / left = NEGATIVE; we flip below so up/left = positive.
    # Watch raw values in the Joystick panel and adjust if your pad differs.
    enable_button=ENABLE_BUTTON,       # LB — hold to enable drive (if require_enable=True)
    require_enable=REQUIRE_ENABLE,     # False = stick alone drives, no need to hold LB
    scale_linear=SCALE_LINEAR,         # multiply left-stick Y by this for vx (m/s)
    scale_angular=SCALE_ANGULAR,       # multiply right-stick X by this for wz (rad/s)
    drive_mode=DRIVE_MODE,             # 'arcade' (default) | 'tank'
    axis_linear=AXIS_LINEAR,           # arcade: left stick Y  -> vx
    axis_angular=AXIS_ANGULAR,         # arcade: right stick X -> wz
    axis_left=AXIS_LEFT,               # tank:   left stick Y
    axis_right=AXIS_RIGHT,             # tank:   right stick Y

    # ---- Arm buttons (Xbox pad button numbers) ----
    # buttons  0=A  1=B  2=X  3=Y  4=LB  5=RB  6=Back  7=Start  8=LS  9=RS
    btn_act1=BTN_ACT1,                 # A button  -> Actuator 1  (linear actuator)
    btn_act2=BTN_ACT2,                 # B button  -> Actuator 2  (linear actuator)
    btn_act3=BTN_ACT3,                 # X button  -> Actuator 3  (linear actuator)
    btn_wrist=BTN_WRIST,               # Y button  -> Wrist        (stepper motor)

    # ---- Trigger axes for arm direction ----
    # LT/RT rest = -1.0 (SDL), fully pressed = +1.0.
    # We convert to 0..1 range: (raw + 1) / 2
    axis_lt=AXIS_LT,                   # Left Trigger axis  -> arm contract  (-1)
    axis_rt=AXIS_RT,                   # Right Trigger axis -> arm expand    (+1)
    trigger_threshold=TRIGGER_THRESHOLD,  # trigger must be pressed > 30% from rest to count

    # ---- D-Pad (gripper) ----
    # On XInput pads the D-Pad comes in as buttons; we synthesize axes 6 (X) and 7 (Y).
    # D-Pad Up   -> gripper contract  (-1)   (axis_dpad_y < -0.5)
    # D-Pad Down -> gripper expand    (+1)   (axis_dpad_y > +0.5)
    axis_dpad_y=AXIS_DPAD_Y,           # synthetic D-Pad Y axis index (appended by XInputPad.read)
)

# ==========================================================================
# 1. Payloads
# ==========================================================================
# Legacy drive-only CMD packet (kept for unpack backward compat): 'C' seq vx wz  (6 bytes)
_CMD = struct.Struct('<cBhh')

# New MOTION packet: 'M' seq vx wz act1 act2 act3 wrist gripper  (16 bytes)
#   act1 / act2 / act3 = linear actuators
#   wrist              = wrist stepper motor
#   gripper            = gripper gear motor
_MOTION = struct.Struct('<cBhhhhhhh')

# ACK from rover: 'A' seq lost  (5 bytes)
_ACK = struct.Struct('<cBH')

# Science actuator packet: 'S' seq m1 m2 m3 m4 m5 led servo  (9 bytes)
_SCIENCE = struct.Struct('<cBBBBBBBB')

# Operational mode packet: 'N' seq mode  (3 bytes: 0=TELEOP, 1=AUTONOMOUS)
_MODE = struct.Struct('<cBB')


def _i16(v):
    """Scale float by 1000 and clamp to signed 16-bit integer."""
    return max(-32768, min(32767, int(round(v * 1000))))


def pack_motion(seq, vx, wz, act1, act2, act3, wrist, gripper):
    """Pack the combined drive + arm MOTION packet (sent every tick at 20 Hz)."""
    return _MOTION.pack(
        b'M', seq & 0xFF,
        _i16(vx), _i16(wz),
        _i16(act1), _i16(act2), _i16(act3),
        _i16(wrist), _i16(gripper),
    )


def pack_cmd(seq, vx, wz):
    """Pack a legacy drive-only CMD packet (used by Radio Check test-frame sender)."""
    return _CMD.pack(b'C', seq & 0xFF, _i16(vx), _i16(wz))


def pack_science(seq, m1, m2, m3, m4, m5, led, servo):
    """Pack Science actuator command packet ('S', 9 bytes)."""
    return _SCIENCE.pack(
        b'S', seq & 0xFF,
        int(bool(m1)), int(bool(m2)), int(bool(m3)), int(bool(m4)), int(bool(m5)),
        int(bool(led)), max(0, min(180, int(servo)))
    )


def pack_mode(seq, mode):
    """Pack Rover operational mode packet ('N', 3 bytes)."""
    return _MODE.pack(b'N', seq & 0xFF, int(mode) & 0xFF)


def unpack(data):
    """Decode an incoming XBee packet.

    Returns one of:
      ('motion',  seq, vx, wz, act1, act2, act3, wrist, gripper)
      ('cmd',     seq, vx, wz)   -- legacy, backward compat
      ('science', seq, m1, m2, m3, m4, m5, led, servo)
      ('mode',    seq, mode)
      ('ack',     seq, lost)     -- rover ACK
      None
    """
    data = bytes(data)
    if data[:1] == b'M' and len(data) == _MOTION.size:
        _, seq, vx, wz, a1, a2, a3, wr, gr = _MOTION.unpack(data)
        return ('motion', seq,
                vx / 1000.0, wz / 1000.0,
                a1 / 1000.0, a2 / 1000.0, a3 / 1000.0,
                wr / 1000.0, gr / 1000.0)
    if data[:1] == b'C' and len(data) == _CMD.size:
        _, seq, vx, wz = _CMD.unpack(data)
        return ('cmd', seq, vx / 1000.0, wz / 1000.0)
    if data[:1] == b'S' and len(data) == _SCIENCE.size:
        _, seq, m1, m2, m3, m4, m5, led, servo = _SCIENCE.unpack(data)
        return ('science', seq, m1, m2, m3, m4, m5, led, servo)
    if data[:1] == b'N' and len(data) == _MODE.size:
        _, seq, mode = _MODE.unpack(data)
        return ('mode', seq, mode)
    if data[:1] == b'A' and len(data) == _ACK.size:
        _, seq, lost = _ACK.unpack(data)
        return ('ack', seq, lost)
    return None


def norm_addr(a):
    return str(a).strip().upper().replace('0X', '').zfill(16)


# ==========================================================================
# 1b. Direct XInput reader (Xbox 360 / One / Series pads) - no pygame needed
# ==========================================================================
class XInputPad:
    """Reads the pad straight from Windows XInput.

    Output layout:
      axes[0] = LX,  axes[1] = LY (up=+),  axes[2] = RX,  axes[3] = RY (up=+)
      axes[4] = LT   (-1=released .. +1=fully pressed)
      axes[5] = RT   (-1=released .. +1=fully pressed)
      axes[6] = D-Pad X synthetic  (-1=left, 0=none, +1=right)
      axes[7] = D-Pad Y synthetic  (-1=up,   0=none, +1=down)

    buttons: 0=A 1=B 2=X 3=Y 4=LB 5=RB 6=Back 7=Start 8=LS 9=RS
    """
    # XInput button bitmasks -> our button index
    _BTN = [(0x1000, 0), (0x2000, 1), (0x4000, 2), (0x8000, 3), (0x0100, 4),
            (0x0200, 5), (0x0020, 6), (0x0010, 7), (0x0040, 8), (0x0080, 9)]

    # D-Pad bitmasks (XInput)
    _DPAD_UP    = 0x0001
    _DPAD_DOWN  = 0x0002
    _DPAD_LEFT  = 0x0004
    _DPAD_RIGHT = 0x0008

    class _State(ctypes.Structure):
        _fields_ = [("pkt", ctypes.c_uint32), ("btn", ctypes.c_uint16), ("lt", ctypes.c_ubyte),
                    ("rt", ctypes.c_ubyte), ("lx", ctypes.c_short), ("ly", ctypes.c_short),
                    ("rx", ctypes.c_short), ("ry", ctypes.c_short)]

    def __init__(self):
        self.fn = None
        if sys.platform == 'win32':
            for dll in ('xinput1_4', 'xinput1_3', 'xinput9_1_0'):
                try:
                    self.fn = getattr(ctypes.windll, dll).XInputGetState
                    break
                except Exception:
                    pass

    @staticmethod
    def _n(v):
        """Normalise raw XInput stick value to -1.0 .. +1.0."""
        return max(-1.0, min(1.0, v / 32767.0))

    def read(self, index):
        """-> (axes, buttons) or None when no pad in that slot.

        axes has 8 elements (indices 0-7); see class docstring.
        """
        if self.fn is None:
            return None
        st = self._State()
        if self.fn(int(index), ctypes.byref(st)) != 0:
            return None

        # Sticks and triggers
        axes = [
            self._n(st.lx),          # 0  LX
            -self._n(st.ly),         # 1  LY  flipped: up = positive
            self._n(st.rx),          # 2  RX
            -self._n(st.ry),         # 3  RY  flipped: up = positive
            st.lt / 127.5 - 1.0,     # 4  LT  (-1=released, +1=pressed)
            st.rt / 127.5 - 1.0,     # 5  RT  (-1=released, +1=pressed)
        ]

        # Synthetic D-Pad axes (index 6 and 7) derived from D-Pad bitmask bits
        dpad_x = (1 if st.btn & self._DPAD_RIGHT else 0) - (1 if st.btn & self._DPAD_LEFT else 0)
        dpad_y = (1 if st.btn & self._DPAD_DOWN  else 0) - (1 if st.btn & self._DPAD_UP   else 0)
        axes.append(float(dpad_x))   # 6  D-Pad X: -1=left,  +1=right
        axes.append(float(dpad_y))   # 7  D-Pad Y: -1=up,    +1=down

        # Face / shoulder buttons
        buttons = [0] * 10
        for mask, i in self._BTN:
            buttons[i] = 1 if st.btn & mask else 0

        return axes, buttons


# ==========================================================================
# 1c. Controller button / trigger / D-pad state extractor
# ==========================================================================
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
        # Linux generic HID gamepad (19 buttons: A=0, B=1, C=2, X=3, Y=4, LB=6, RB=7, LT=8, RT=9)
        btn_A  = g(0)
        btn_B  = g(1)
        btn_X  = g(3) or g(2)
        btn_Y  = g(4)
        btn_LB = g(6)
        btn_RB = g(7)
        btn_LT = g(8)
        btn_RT = g(9)
    else:
        # Standard XInput / 10-12 button Xbox controller
        # 0: A, 1: B, 2: X, 3: Y, 4: LB, 5: RB, 6: Back, 7: Start, 8: LS, 9: RS
        btn_A  = g(0)
        btn_B  = g(1)
        btn_X  = g(2)
        btn_Y  = g(3)
        btn_LB = g(4)
        btn_RB = g(5)
        btn_LT = g(8) if n > 10 else False
        btn_RT = g(9) if n > 10 else False

    # Analog Triggers:
    # At rest, LT and RT raw axes are -1.0. When pulled, they increase towards +1.0.
    # We consider pulled if > -0.4 (more than ~30% travel from rest).
    lt_axis = raw_axes[4] if len(raw_axes) > 4 else -1.0
    rt_axis = raw_axes[5] if len(raw_axes) > 5 else -1.0
    lt_pulled = (lt_axis > -0.4)
    rt_pulled = (rt_axis > -0.4)

    # Either trigger (LT/RT) or shoulder bumper (LB/RB) triggers contract / expand:
    contract = lt_pulled or btn_LB or btn_LT
    expand   = rt_pulled or btn_RB or btn_RT

    # Directional Pad (D-Pad):
    gripper = 0.0
    dpad_up = False
    dpad_down = False

    # 1) Check Pygame Hats
    if hats and len(hats) > 0:
        hy = hats[0][1]
        if hy > 0:
            dpad_up = True
            gripper = -1.0   # D-Pad Up -> contract gripper
        elif hy < 0:
            dpad_down = True
            gripper = 1.0    # D-Pad Down -> expand gripper
    # 2) Check XInput / SDL synthetic axis
    elif len(raw_axes) > 7:
        dpad_y = raw_axes[7]
        if dpad_y < -0.5:
            dpad_up = True
            gripper = -1.0   # Up in XInput synthetic axis
        elif dpad_y > 0.5:
            dpad_down = True
            gripper = 1.0    # Down in XInput synthetic axis

    # 3) Check D-Pad digital buttons if mapped on Linux (buttons 11-14)
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


# ==========================================================================
# 2. XBee API Frame Builder & Non-Blocking Serial Link
# ==========================================================================
def wrap_xbee_api_tx(raw_payload: bytes, dest_addr_hex: str = '') -> bytes:
    """Build an XBee API mode (AP=1) Transmit Request frame (type 0x10).

    Settings:
      - 0x10 = Transmit Request
      - 0x00 = Frame ID (0 means no transmit status 0x8B callback, eliminating serial overhead)
      - 64-bit Address = Broadcast (00..FFFF) by default or 8-byte destination address
      - FF FE = 16-bit Broadcast
      - 0x00 = Broadcast Radius
      - 0x01 = Disable RF ACK (CRITICAL for fixing 20 Hz loop freeze and stutter)
    """
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


class XBeeLink:
    """Non-blocking, zero-stutter XBee API serial link.

    - Uses write_timeout=0 to guarantee serial.write() never blocks the 20 Hz loop
      even during heavy RTCM transmission.
    - Sends 0x10 Transmit Request with Frame ID 0x00 and Option 0x01 (Disable RF ACK).
    - Runs a background thread parsing incoming 0x90 Receive Packet frames.
    """

    def __init__(self, port, baud, remote_addr, on_message):
        self.port = port
        self.baud = int(baud)
        self.remote_addr = norm_addr(remote_addr) if remote_addr else ''
        self.on_message = on_message          # called from the reader thread
        self.ser = None
        self.running = False
        self.thread = None
        self.local_addr = ''
        self.err = ''
        self.sent = self.tx_ok = self.tx_fail = 0
        self.rx = self.rx_ignored = 0
        self.last_src = ''
        self.last_fail = ''

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self):
        self.close()
        try:
            import serial
            # Critical: timeout=0 and write_timeout=0 prevent freezing the 20 Hz loop
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0,
                write_timeout=0
            )
            self.running = True
            self.err = ''
            self.thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.thread.start()
            self._query_local_addr()
        except Exception as e:
            msg = str(e) or type(e).__name__
            self.err = msg
            self.ser = None

    def send(self, payload, dest_addr=None):
        """Send payload wrapped in an XBee API 0x10 frame (non-blocking, no RF ACK)."""
        if not self.ser or not self.ser.is_open:
            return False
        try:
            target = dest_addr if dest_addr is not None else self.remote_addr
            frame = wrap_xbee_api_tx(payload, target)
            self.ser.write(frame)
            self.sent += 1
            self.tx_ok += 1
            return True
        except Exception as e:
            # write_timeout=0 drops frame cleanly if buffer is full without freezing the loop
            self.tx_fail += 1
            self.last_fail = str(e)
            return False

    def _query_local_addr(self):
        """Query local 64-bit address using AT SH and AT SL frames."""
        try:
            for cmd in (b'SH', b'SL'):
                frame_data = bytes([0x08, 0x01]) + cmd
                length = len(frame_data)
                chk = 0xFF - (sum(frame_data) & 0xFF)
                pkt = struct.pack('>BH', 0x7E, length) + frame_data + bytes([chk])
                self.ser.write(pkt)
                time.sleep(0.02)
        except Exception:
            pass

    def _rx_loop(self):
        """Background thread reading and parsing incoming API frames."""
        buf = bytearray()
        while self.running and self.ser and self.ser.is_open:
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    time.sleep(0.005)
                    continue
                buf.extend(chunk)

                # Parse all complete 0x7E frames in buffer
                while len(buf) >= 4:
                    try:
                        idx = buf.index(0x7E)
                    except ValueError:
                        buf.clear()
                        break
                    if idx > 0:
                        del buf[:idx]

                    if len(buf) < 4:
                        break

                    length = (buf[1] << 8) | buf[2]
                    total_len = 3 + length + 1

                    if len(buf) < total_len:
                        break  # Incomplete frame, wait for more data

                    frame_data = bytes(buf[3:3 + length])
                    chk = buf[3 + length]
                    calc_chk = 0xFF - (sum(frame_data) & 0xFF)

                    del buf[:total_len]

                    if chk != calc_chk:
                        continue  # Checksum mismatch, discard

                    self._handle_frame(frame_data)
            except Exception:
                if not self.running:
                    break
                time.sleep(0.01)

    def _handle_frame(self, frame_data):
        if not frame_data:
            return
        frame_type = frame_data[0]

        # 0x90 = Receive Packet (Physical RF RX) or 0x10 = Transmit Request (Direct Loopback / SIL)
        if frame_type in (0x90, 0x10) and len(frame_data) >= (12 if frame_type == 0x90 else 14):
            if frame_type == 0x90:
                src_addr = frame_data[1:9].hex().upper()
                rf_data = bytes(frame_data[12:])
            else:
                src_addr = frame_data[2:10].hex().upper()
                rf_data = bytes(frame_data[14:])
            self.last_src = src_addr

            # Filter by remote_addr if set and not broadcast
            if (self.remote_addr and 
                not self.remote_addr.endswith('FFFF') and 
                self.remote_addr != src_addr):
                self.rx_ignored += 1
                return

            self.rx += 1
            if self.on_message:
                self.on_message(rf_data)

        # 0x88 = AT Command Response
        elif frame_type == 0x88 and len(frame_data) >= 5:
            cmd = frame_data[2:4]
            status = frame_data[4]
            data = frame_data[5:]
            if status == 0:
                if cmd == b'SH':
                    self._sh = data.hex().upper()
                elif cmd == b'SL':
                    self._sl = data.hex().upper()
                    if hasattr(self, '_sh'):
                        self.local_addr = self._sh + self._sl

    def close(self):
        self.running = False
        ser, self.ser = self.ser, None
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass


class UdpLink:
    """Zero-overhead UDP link to Base Master script (127.0.0.1:9001)."""
    def __init__(self, master_ip, joy_port, on_message=None):
        self.master_ip = master_ip
        self.joy_port = joy_port
        self.on_message = on_message
        self.sock = None
        self.running = False
        self.local_addr = 'UDP_MASTER'
        self.remote_addr = f'{master_ip}:{joy_port}'
        self.err = ''
        self.sent = self.tx_ok = self.tx_fail = 0
        self.rx = self.rx_ignored = 0
        self.last_src = 'MASTER_HUB'
        self.last_fail = ''
        self._connected = False

    @property
    def connected(self):
        return self._connected

    def connect(self):
        self.close()
        try:
            import socket
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)
            self._connected = True
            self.running = True
            self.err = ''
        except Exception as e:
            self.err = str(e)
            self._connected = False

    def send(self, payload, dest_addr=None):
        if not self._connected or not self.sock:
            return False
        try:
            self.sock.sendto(payload, (self.master_ip, self.joy_port))
            self.sent += 1
            self.tx_ok += 1
            return True
        except Exception as e:
            self.tx_fail += 1
            self.last_fail = str(e)
            return False

    def close(self):
        self.running = False
        self._connected = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


class UdpTelemetryReceiver:
    """Background receiver for Rover Telemetry ('T' or JSON) and ACKs ('A') on UDP port 9003."""
    def __init__(self, port, callback):
        self.port = int(port)
        self.callback = callback
        self.running = False
        self.thread = None
        self.sock = None

    def start(self):
        if self.running:
            return
        try:
            import socket
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if hasattr(socket, 'SO_REUSEPORT'):
                try:
                    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                except Exception:
                    pass
            self.sock.bind(('0.0.0.0', self.port))
            self.sock.settimeout(0.5)
            self.running = True
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
        except Exception:
            self.sock = None
            self.running = False

    def _run(self):
        while self.running and self.sock:
            try:
                data, _ = self.sock.recvfrom(4096)
                if data and self.callback:
                    self.callback(data)
            except Exception:
                continue

    def stop(self):
        self.running = False
        sock, self.sock = self.sock, None
        if sock:
            try:
                sock.close()
            except Exception:
                pass



# ==========================================================================
# 3. Radio check / fix / send
# ==========================================================================
BD_NAMES = {3: '9600', 6: '57600', 7: '115200'}


def radio_at_session(port, baud, fix):
    import serial
    s = serial.Serial(port, baud, timeout=1.5)
    try:
        time.sleep(1.1)
        s.write(b'+++')
        time.sleep(1.5)
        if not s.read(3).startswith(b'OK'):
            raise RuntimeError('no reply to +++ (radio already in API mode? wrong baud?)')

        def at(cmd):
            s.reset_input_buffer()
            s.write(f'AT{cmd}\r'.encode())
            return s.read_until(b'\r').decode().strip()

        out = {k: at(k) for k in ('SH', 'SL', 'VR', 'AP', 'BD', 'ID')}
        if fix:
            out['fix'] = f'AP1->{at("AP1")}  WR->{at("WR")}'
            out['AP'] = at('AP')
        at('CN')
        return out
    finally:
        s.close()


def radio_api_session(port, baud, send, log=print):
    dev = XBeeDevice(port, baud)
    dev.open()
    try:
        out = dict(addr=str(dev.get_64bit_addr()), proto=str(dev.get_protocol()),
                   fw=dev.get_firmware_version(),
                   AP=dev.get_parameter('AP').hex(), BD=dev.get_parameter('BD').hex(),
                   ID=dev.get_parameter('ID').hex())
        if send:
            log('broadcasting 20 zero frames (watch the TX LED) ...')
            for i in range(20):
                dev.send_data_broadcast(pack_cmd(i, 0.0, 0.0))
                time.sleep(0.05)
            log('done')
        return out
    finally:
        dev.close()


def radio_show(r, log=print):
    ap_i, bd_i = int(r['AP'], 16), int(r['BD'], 16)
    log(f"64-bit addr : {r['addr']}  <- put this in the OTHER side's address box")
    if r.get('proto'):
        log(f"protocol    : {r['proto']}   firmware: {r['fw']}")
    log(f"AP (API)    : {ap_i:02X}  ({'OK, API mode' if ap_i == 1 else 'NOT API mode - press FIX'})")
    log(f"BD (baud)   : {bd_i:08X}  ({BD_NAMES.get(bd_i, '?')} baud; 7 = 115200)")
    log(f"ID (network): {int(r['ID'], 16):X}  (must match on both radios)")


def radio_check(port, baud, fix=False, send=False, log=print):
    log(f'port        : {port} @ {baud}')
    if fix:
        r = radio_at_session(port, baud, True)
        r['addr'] = r['SH'].zfill(8) + r['SL'].zfill(8)
        radio_show(r, log)
        if r.get('fix'):
            log('fix        : ' + r['fix'])
        log('now unplug / re-plug the radio, then press CHECK again.')
        return
    try:
        r = radio_api_session(port, baud, send, log)
        radio_show(r, log)
    except Exception as e:
        log(f'API mode open failed ({e}); trying AT command mode ...')
        r = radio_at_session(port, baud, False)
        r['addr'] = r['SH'].zfill(8) + r['SL'].zfill(8)
        radio_show(r, log)


# ==========================================================================
# 4. TX core (logic without ROS — mirrors tx_node.py)
# ==========================================================================
class TxCore:
    """Holds the latest joystick state and sends MOTION packets to the rover."""

    def __init__(self, link, cfg):
        self.link, self.cfg = link, cfg

        # Drive values
        self.vx = 0.0
        self.wz = 0.0

        # Arm values: act1, act2, act3, wrist, gripper  (each -1 / 0 / +1)
        self.arm = (0.0, 0.0, 0.0, 0.0, 0.0)

        self.enable   = False
        self.joy_time = 0.0
        self.axes, self.buttons = [], []
        self.seq      = 0
        self.lock     = threading.Lock()
        self.ack      = None       # (monotonic_time, seq, lost) from last rover ACK
        self.ack_count = 0
        self._last    = (time.monotonic(), 0)  # for TX rate calculation

        # Rate-limit terminal printing to once per second
        self._print_time = 0.0

    # ---------------------------------------------------------------------- #
    # Arm command logic
    # ---------------------------------------------------------------------- #
    def _arm_commands(self, buttons, raw_axes, hats=None):
        """Compute arm channel values from buttons, triggers, and D-Pad.

        Returns (act1, act2, act3, wrist, gripper) each in {-1.0, 0.0, +1.0}.

        Rules:
          - A + (LT or LB) -> Actuator 1 contract (-1.0)
          - A + (RT or RB) -> Actuator 1 expand   (+1.0)
          - B + (LT or LB) -> Actuator 2 contract (-1.0)
          - B + (RT or RB) -> Actuator 2 expand   (+1.0)
          - X + (LT or LB) -> Actuator 3 contract (-1.0)
          - X + (RT or RB) -> Actuator 3 expand   (+1.0)
          - Y + (LT or LB) -> Wrist contract      (-1.0)
          - Y + (RT or RB) -> Wrist expand        (+1.0)
          - Releasing button OR trigger/bumper -> stays at 0.0 (hold position)
          - D-Pad Up   -> Gripper contract (-1.0)
          - D-Pad Down -> Gripper expand   (+1.0)
          - Releasing D-Pad -> stays at 0.0 (hold position)
        """
        btns, contract, expand, gripper = get_controller_state(buttons, raw_axes, hats)

        def _ch(is_held):
            """Return -1.0 / 0.0 / +1.0 for one arm channel."""
            if not is_held:
                return 0.0        # Button released -> stay still (0.0)
            if contract and not expand:
                return -1.0       # LT or LB -> contract (-1.0)
            if expand and not contract:
                return 1.0        # RT or RB -> expand (+1.0)
            return 0.0            # Button held but neither trigger/bumper -> stay still (0.0)

        act1  = _ch(btns['A'])
        act2  = _ch(btns['B'])
        act3  = _ch(btns['X'])
        wrist = _ch(btns['Y'])

        return act1, act2, act3, wrist, gripper

    # ---------------------------------------------------------------------- #
    # Joystick input handler
    # ---------------------------------------------------------------------- #
    def on_joy(self, axes, buttons, raw_axes=None, hats=None):
        """Update drive and arm state from new joystick data.
        axes must use ROS convention: stick up / left = positive.
        """
        g = lambda seq, i: seq[i] if 0 <= i < len(seq) else 0.0
        c = self.cfg

        raw_axes = raw_axes if raw_axes is not None else axes
        btns, contract, expand, _ = get_controller_state(buttons, raw_axes, hats)

        # Drive: enable can be triggered by deadman button or enable bypassed
        self.enable = bool(g(buttons, c['enable_button'])) or btns['LB']
        if self.enable or not c['require_enable']:
            if c['drive_mode'] == 'tank':
                left  = g(axes, c['axis_left'])
                right = g(axes, c['axis_right'])
                self.vx = c['scale_linear']  * (left + right) / 2.0
                self.wz = c['scale_angular'] * (right - left) / 2.0
            else:
                ang_axis = c['axis_angular']
                if ang_axis == 2 and len(buttons) <= 12 and len(axes) > 5 and not getattr(self, 'xin_ok', False):
                    ang_axis = 3
                # Left throttle (Left Stick Y) strictly controls front and back speed (vx); left/right is ignored.
                self.vx = c['scale_linear']  * g(axes, c['axis_linear'])
                # Right throttle (Right Stick X) strictly controls left/right turning (wz); front/back is ignored.
                self.wz = c['scale_angular'] * g(axes, ang_axis)
        else:
            self.vx = self.wz = 0.0   # deadman released -> stop drive

        # Arm: compute from buttons, raw un-inverted trigger axes, and hats
        self.arm = self._arm_commands(buttons, raw_axes, hats)

        self.axes, self.buttons = list(axes), list(buttons)
        self.joy_time = time.monotonic()

    def no_input(self):
        """Called when joystick data goes stale -> zero everything."""
        self.vx = self.wz = 0.0
        self.arm = (0.0, 0.0, 0.0, 0.0, 0.0)
        self.enable = False

    # ---------------------------------------------------------------------- #
    # Transmit one packet (called at 20 Hz)
    # ---------------------------------------------------------------------- #
    def send_tick(self):
        """Pack and send one MOTION packet with drive + arm data."""
        fresh = time.monotonic() - self.joy_time < self.cfg['cmd_timeout']
        vx, wz      = (self.vx, self.wz) if fresh else (0.0, 0.0)
        act1, act2, act3, wrist, gripper = self.arm if fresh else (0.0, 0.0, 0.0, 0.0, 0.0)

        if self.link.send(pack_motion(self.seq, vx, wz, act1, act2, act3, wrist, gripper)):
            self.seq = (self.seq + 1) & 0xFF

        # Print a readable status line to the terminal (rate-limited to ~1 Hz)
        now = time.monotonic()
        if now - self._print_time >= 1.0:
            self._print_time = now
            arm_active = any(v != 0.0 for v in (act1, act2, act3, wrist, gripper))
            print(
                f'[TX] seq={self.seq:3d}  |  '
                f'drive: vx={vx:+.2f}  wz={wz:+.2f}  |  '
                f'arm: act1={act1:+.0f}  act2={act2:+.0f}  act3={act3:+.0f}  '
                f'wrist={wrist:+.0f}  gripper={gripper:+.0f}  |  '
                f'rate={self.link.sent / max(now - 1e-3, 1e-3):.0f}?Hz'
                + ('  <<ARM ACTIVE>>' if arm_active else ''),
                flush=True
            )

    # ---------------------------------------------------------------------- #
    # Receive ACK from rover
    # ---------------------------------------------------------------------- #
    def on_radio(self, data):
        """Called by the XBee thread when the rover sends back an ACK."""
        m = unpack(data)
        if m and m[0] == 'ack':
            with self.lock:
                self.ack = (time.monotonic(), m[1], m[2])
                self.ack_count += 1

    # ---------------------------------------------------------------------- #
    # Status dict (read by the GUI refresh timer)
    # ---------------------------------------------------------------------- #
    def status(self):
        now = time.monotonic()
        t0, n0 = self._last
        rate = (self.link.sent - n0) / max(now - t0, 1e-3)
        self._last = (now, self.link.sent)
        with self.lock:
            ack, ack_count = self.ack, self.ack_count
        if not self.link.connected:    # don't show stale ACK when link is gone
            ack = None
        act1, act2, act3, wrist, gripper = self.arm
        return dict(
            link=self.link.connected, port=self.link.port, baud=self.link.baud,
            local=self.link.local_addr, remote=self.link.remote_addr, err=self.link.err,
            sent=self.link.sent, rate=round(rate, 1), tx_ok=self.link.tx_ok,
            tx_fail=self.link.tx_fail, why=self.link.last_fail,
            vx=self.vx, wz=self.wz, enable=self.enable,
            act1=act1, act2=act2, act3=act3, wrist=wrist, gripper=gripper,
            ack_count=ack_count,
            ack=None if ack is None else dict(age=now - ack[0], seq=ack[1], lost=ack[2]),
        )


# ==========================================================================
# 5. GUI
# ==========================================================================
import pygame
from PyQt5.QtCore import Qt, QEvent, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QColor, QPainter
from PyQt5.QtWidgets import (QApplication, QWidget, QGridLayout, QFormLayout, QGroupBox,
                             QHBoxLayout, QLabel, QVBoxLayout, QComboBox, QLineEdit,
                             QPushButton, QCheckBox, QTabWidget, QTextEdit, QMessageBox)

GREEN, RED, GREY, AMBER = '#2ecc71', '#e74c3c', '#7f8c8d', '#f1c40f'
KEYMAP = {Qt.Key_Up: 'up', Qt.Key_Down: 'down', Qt.Key_Left: 'left', Qt.Key_Right: 'right',
          Qt.Key_W: 'up', Qt.Key_S: 'down', Qt.Key_A: 'left', Qt.Key_D: 'right'}


def keyboard_axes(keys, cfg):
    """Held arrow/WASD keys -> axes list in ROS convention (up / left = +), plus a buttons list."""
    fwd = (1.0 if 'up' in keys else 0.0) - (1.0 if 'down' in keys else 0.0)
    turn = (1.0 if 'left' in keys else 0.0) - (1.0 if 'right' in keys else 0.0)
    n = max(cfg['axis_linear'], cfg['axis_angular'], cfg['axis_left'], cfg['axis_right']) + 1
    axes = [0.0] * n
    if cfg['drive_mode'] == 'tank':
        axes[cfg['axis_left']] = max(-1.0, min(1.0, fwd - turn))
        axes[cfg['axis_right']] = max(-1.0, min(1.0, fwd + turn))
    else:
        axes[cfg['axis_linear']] = fwd
        axes[cfg['axis_angular']] = turn
    buttons = [0] * (cfg['enable_button'] + 1)
    if keys:                                    # any drive key held counts as the deadman
        buttons[cfg['enable_button']] = 1
    return axes, buttons


class Bar(QWidget):
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
            if k in self.v:
                self.v[k].setText(str(t))


class SciencePanel(QGroupBox):
    def __init__(self, on_toggle_motor, on_toggle_led, on_step_servo, on_stop_all):
        super().__init__('Science Actuators (Ctrl+1..6, Ctrl+Arrows)')
        self.on_toggle_motor = on_toggle_motor
        self.on_toggle_led = on_toggle_led
        self.on_step_servo = on_step_servo
        self.on_stop_all = on_stop_all

        main_lay = QVBoxLayout(self)
        main_lay.setContentsMargins(8, 8, 8, 8)
        main_lay.setSpacing(4)

        # Header status (LED dot + status text)
        h_box = QHBoxLayout()
        self.led = QLabel()
        self.led.setFixedSize(14, 14)
        self.head = QLabel('Waiting for Science telemetry...')
        self.head.setStyleSheet('font-weight: bold; font-size: 11px;')
        h_box.addWidget(self.led)
        h_box.addWidget(self.head)
        h_box.addStretch(1)
        main_lay.addLayout(h_box)

        # Actuators form/grid layout
        form = QGridLayout()
        form.setHorizontalSpacing(8)
        form.setVerticalSpacing(3)

        self.motor_labels = {}
        self.motor_btns = {}

        # 5 Motors
        motor_names = [
            (1, 'Motor 1 (Sample 1) [Ctrl+1]'),
            (2, 'Motor 2 (Sample 2) [Ctrl+2]'),
            (3, 'Motor 3 (Reagent 1) [Ctrl+3]'),
            (4, 'Motor 4 (Reagent 2) [Ctrl+4]'),
            (5, 'Motor 5 (Flush)    [Ctrl+5]'),
        ]
        for row, (idx, name) in enumerate(motor_names):
            lbl_title = QLabel(name + ':')
            lbl_title.setStyleSheet('font-size: 11px;')

            lbl_status = QLabel('OFF')
            lbl_status.setAlignment(Qt.AlignCenter)
            lbl_status.setFixedWidth(55)
            lbl_status.setStyleSheet(
                'background: #34495e; color: #bdc3c7; font-weight: bold; '
                'border-radius: 3px; padding: 2px 4px; font-size: 10px;'
            )
            self.motor_labels[f'm{idx}'] = lbl_status

            btn = QPushButton('Toggle')
            btn.setFocusPolicy(Qt.NoFocus)
            btn.setFixedHeight(22)
            btn.setStyleSheet('font-size: 10px; padding: 1px 6px;')
            btn.clicked.connect(lambda _, i=idx: self.on_toggle_motor(i))
            self.motor_btns[f'm{idx}'] = btn

            form.addWidget(lbl_title, row, 0)
            form.addWidget(lbl_status, row, 1)
            form.addWidget(btn, row, 2)

        # LED
        row = len(motor_names)
        lbl_led_title = QLabel('Illumination LED [Ctrl+6]:')
        lbl_led_title.setStyleSheet('font-size: 11px;')
        self.lbl_led_status = QLabel('OFF')
        self.lbl_led_status.setAlignment(Qt.AlignCenter)
        self.lbl_led_status.setFixedWidth(55)
        self.lbl_led_status.setStyleSheet(
            'background: #34495e; color: #bdc3c7; font-weight: bold; '
            'border-radius: 3px; padding: 2px 4px; font-size: 10px;'
        )
        self.btn_led = QPushButton('Toggle')
        self.btn_led.setFocusPolicy(Qt.NoFocus)
        self.btn_led.setFixedHeight(22)
        self.btn_led.setStyleSheet('font-size: 10px; padding: 1px 6px;')
        self.btn_led.clicked.connect(self.on_toggle_led)

        form.addWidget(lbl_led_title, row, 0)
        form.addWidget(self.lbl_led_status, row, 1)
        form.addWidget(self.btn_led, row, 2)

        # Servo
        row += 1
        lbl_servo_title = QLabel('Carousel Servo [Ctrl+←/→]:')
        lbl_servo_title.setStyleSheet('font-size: 11px;')
        self.lbl_servo_status = QLabel('0°')
        self.lbl_servo_status.setAlignment(Qt.AlignCenter)
        self.lbl_servo_status.setFixedWidth(55)
        self.lbl_servo_status.setStyleSheet(
            'background: #2c3e50; color: #3498db; font-weight: bold; '
            'border-radius: 3px; padding: 2px 4px; font-size: 11px;'
        )
        s_btn_box = QHBoxLayout()
        s_btn_box.setSpacing(4)
        self.btn_servo_dec = QPushButton('◄ -15°')
        self.btn_servo_dec.setFocusPolicy(Qt.NoFocus)
        self.btn_servo_dec.setFixedHeight(22)
        self.btn_servo_dec.setStyleSheet('font-size: 10px; padding: 1px 4px;')
        self.btn_servo_dec.clicked.connect(lambda: self.on_step_servo(-15))

        self.btn_servo_inc = QPushButton('+15° ►')
        self.btn_servo_inc.setFocusPolicy(Qt.NoFocus)
        self.btn_servo_inc.setFixedHeight(22)
        self.btn_servo_inc.setStyleSheet('font-size: 10px; padding: 1px 4px;')
        self.btn_servo_inc.clicked.connect(lambda: self.on_step_servo(15))

        s_btn_box.addWidget(self.btn_servo_dec)
        s_btn_box.addWidget(self.btn_servo_inc)

        form.addWidget(lbl_servo_title, row, 0)
        form.addWidget(self.lbl_servo_status, row, 1)
        form.addLayout(s_btn_box, row, 2)

        main_lay.addLayout(form)

        # Bottom Safety / All Stop button
        self.btn_stop_all = QPushButton('⏹ STOP ALL MOTORS (z)')
        self.btn_stop_all.setFocusPolicy(Qt.NoFocus)
        self.btn_stop_all.setFixedHeight(24)
        self.btn_stop_all.setStyleSheet(
            'background: #c0392b; color: white; font-weight: bold; '
            'border-radius: 3px; font-size: 10px; margin-top: 2px;'
        )
        self.btn_stop_all.clicked.connect(self.on_stop_all)
        main_lay.addWidget(self.btn_stop_all)

        self.state(GREY, 'No Science Telemetry')

    def state(self, color, text):
        self.led.setStyleSheet(f'background:{color}; border-radius:7px; border:1px solid #222;')
        self.head.setText(text)

    def update_display(self, state):
        """Update widgets strictly from confirmed rover telemetry state."""
        now = time.monotonic()
        age = now - state.get('last_update', 0.0)
        is_fresh = age < 3.0 and state.get('last_update', 0.0) > 0.0
        mega_ok = state.get('connected', False)

        if not is_fresh:
            self.state(GREY, 'No Science Telemetry')
        elif not mega_ok:
            self.state(AMBER, f'Rover alive, Mega offline ({age:.1f}s)')
        else:
            self.state(GREEN, f'Arduino Mega Ready ({age * 1000:.0f}ms)')

        # Motors 1..5
        for i in range(1, 6):
            key = f'm{i}'
            val = state.get(key, 0)
            lbl = self.motor_labels.get(key)
            if lbl:
                if val:
                    lbl.setText('ON')
                    lbl.setStyleSheet(
                        'background: #27ae60; color: #ffffff; font-weight: bold; '
                        'border-radius: 3px; padding: 2px 4px; font-size: 10px;'
                    )
                else:
                    lbl.setText('OFF')
                    lbl.setStyleSheet(
                        'background: #34495e; color: #bdc3c7; font-weight: bold; '
                        'border-radius: 3px; padding: 2px 4px; font-size: 10px;'
                    )

        # LED
        led_val = state.get('led', 0)
        if led_val:
            self.lbl_led_status.setText('ON')
            self.lbl_led_status.setStyleSheet(
                'background: #f39c12; color: #ffffff; font-weight: bold; '
                'border-radius: 3px; padding: 2px 4px; font-size: 10px;'
            )
        else:
            self.lbl_led_status.setText('OFF')
            self.lbl_led_status.setStyleSheet(
                'background: #34495e; color: #bdc3c7; font-weight: bold; '
                'border-radius: 3px; padding: 2px 4px; font-size: 10px;'
            )

        # Servo
        servo_val = state.get('servo', 0)
        self.lbl_servo_status.setText(f'{servo_val}°')


class RadioWorker(QThread):
    line = pyqtSignal(str)

    def __init__(self, port, baud, fix, send):
        super().__init__()
        self.port, self.baud, self.fix, self.send = port, baud, fix, send

    def run(self):
        try:
            radio_check(self.port, self.baud, self.fix, self.send, log=self.line.emit)
        except Exception as e:
            self.line.emit(f'FAILED: {e}')


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('XBee Teleop — Drive + Arm Control')
        self.cfg = dict(CONFIG)
        self.link = self.tx = None
        self.auto = False                       # True = connected / keep reconnecting
        self.connecting = False
        self.joy, self.joy_ok = None, False
        self.xpad, self.xin_ok = XInputPad(), False
        self.raw_axes, self.buttons = [], []
        self.hats = []
        self.keys = set()
        self.keyboard_drive = False

        # Science actuators state tracking
        self.sci_seq = 0
        self.sci_lock = threading.Lock()
        self.sci_cmd = {
            'm1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'm5': 0,
            'led': 0, 'servo': 0
        }
        self.sci_confirmed = {
            'm1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'm5': 0,
            'led': 0, 'servo': 0,
            'connected': False,
            'last_update': 0.0
        }

        # pygame MUST be up before the poll timer fires
        pygame.init()
        pygame.joystick.init()

        # ---- connection bar ----
        bar = QHBoxLayout()
        bar.addWidget(QLabel('XBee port:'))
        self.port_box = QComboBox()
        self.port_box.setEditable(True)
        self.port_box.setMinimumWidth(160)
        self.refresh_ports()
        bar.addWidget(self.port_box)
        self.btn_refresh = QPushButton('↻')
        self.btn_refresh.setToolTip('Refresh available serial & virtual ports')
        self.btn_refresh.setFixedWidth(28)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        bar.addWidget(self.btn_refresh)
        bar.addWidget(QLabel(' Baud:'))
        self.baud_box = QComboBox()
        self.baud_box.setEditable(True)
        for b in AVAILABLE_BAUD_RATES:
            self.baud_box.addItem(str(b))
        baud_idx = self.baud_box.findText(str(CONFIG.get('baud', XBEE_BAUD_RATE)))
        if baud_idx >= 0:
            self.baud_box.setCurrentIndex(baud_idx)
        bar.addWidget(self.baud_box)
        bar.addWidget(QLabel(' Robot radio addr:'))
        self.addr_edit = QLineEdit(ROBOT_RADIO_ADDR)
        self.addr_edit.setMaxLength(18)
        self.addr_edit.setFixedWidth(150)
        bar.addWidget(self.addr_edit)
        self.btn_connect = QPushButton('Connect')
        self.btn_connect.clicked.connect(self.toggle_connect)
        bar.addWidget(self.btn_connect)
        self.kb_check = QCheckBox('Keyboard drive (arrows/WASD)')
        self.kb_check.setFocusPolicy(Qt.NoFocus)
        self.kb_check.toggled.connect(self.on_kb)
        bar.addWidget(self.kb_check)
        bar.addStretch(1)

        # ---- panels ----
        self.joy_p = Panel('Joystick', [('pad', 'Device'), ('seen', 'Devices seen by pygame'),
                                        ('deadman', 'Deadman (hold to drive)'),
                                        ('btn', 'Buttons pressed'), ('raw', 'Raw axes (SDL)'),
                                        ('trig', 'Triggers LT / RT')])
        self.sticks = {}
        for i in range(4):
            self.sticks[i] = Bar()
            self.joy_p.layout().addRow(f'Axis {i}:', self.sticks[i])

        # Drive command panel (unchanged)
        self.cmd = Panel('Drive Command', [('vx', 'Linear vx (m/s)'), ('wz', 'Angular wz (rad/s)')])
        self.vx_bar, self.wz_bar = Bar(), Bar()
        self.cmd.layout().addRow('vx:', self.vx_bar)
        self.cmd.layout().addRow('wz:', self.wz_bar)

        # Arm command panel (new)
        self.arm_p = Panel('Arm Control',
                           [('act1',    'Actuator 1  (A + LT/LB or RT/RB)'),
                            ('act2',    'Actuator 2  (B + LT/LB or RT/RB)'),
                            ('act3',    'Actuator 3  (X + LT/LB or RT/RB)'),
                            ('wrist',   'Wrist        (Y + LT/LB or RT/RB)'),
                            ('gripper', 'Gripper      (D-Pad Up/Down)')])

        self.tx_p = Panel('XBee TX (radio level only)',
                          [('port', 'Port'), ('local', 'This radio'), ('remote', 'Sending to'),
                           ('sent', 'Frames sent'), ('rate', 'Rate'),
                           ('ok', 'Robot RADIO acked / failed'), ('why', 'Last radio failure'), ('err', 'Error')])
        self.rx_p = Panel('Robot ACK (rx_node software)',
                          [('n', 'Last ACK seq'), ('cnt', 'ACKs received (total)'), ('age', 'ACK age'),
                           ('lost', 'Frames lost (robot count)'), ('src', 'Last radio source'),
                           ('ign', 'Ignored frames (wrong source)')])

        # Lay out in a 3-column grid: row0 = joystick | drive | arm
        #                              row1 = XBee TX  | robot ACK | science actuators
        grid = QGridLayout()
        grid.addWidget(self.joy_p,  0, 0)
        grid.addWidget(self.cmd,    0, 1)
        grid.addWidget(self.arm_p,  0, 2)
        grid.addWidget(self.tx_p,   1, 0)
        grid.addWidget(self.rx_p,   1, 1)

        # Science Actuators panel
        self.sci_p = SciencePanel(
            on_toggle_motor=self.toggle_motor,
            on_toggle_led=self.toggle_led,
            on_step_servo=self.step_servo,
            on_stop_all=self.stop_all_motors,
        )
        grid.addWidget(self.sci_p,  1, 2)

        # ---- radio check tab ----
        radio = QWidget()
        rlay = QVBoxLayout(radio)
        rbar = QHBoxLayout()
        rbar.addWidget(QLabel('Port:'))
        self.r_port = QComboBox()
        self.r_port.setEditable(True)
        rbar.addWidget(self.r_port)
        rbar.addWidget(QLabel('Baud:'))
        self.r_baud = QComboBox()
        self.r_baud.setEditable(True)
        for b in AVAILABLE_BAUD_RATES:
            self.r_baud.addItem(str(b))
        r_baud_idx = self.r_baud.findText(str(CONFIG.get('baud', XBEE_BAUD_RATE)))
        if r_baud_idx >= 0:
            self.r_baud.setCurrentIndex(r_baud_idx)
        rbar.addWidget(self.r_baud)
        self.radio_btns = []
        for text, fix, send in (('Check', False, False), ('Fix (set AP=1 + save)', True, False),
                                ('Send 20 test frames', False, True)):
            b = QPushButton(text)
            b.clicked.connect(lambda _=False, f=fix, s=send: self.run_radio(f, s))
            rbar.addWidget(b)
            self.radio_btns.append(b)
        rlay.addLayout(rbar)
        self.radio_log = QTextEdit()
        self.radio_log.setReadOnly(True)
        rlay.addWidget(self.radio_log)
        self.radio_worker = None
        for i in range(self.port_box.count()):
            self.r_port.addItem(self.port_box.itemText(i))

        tabs = QTabWidget()
        main_tab = QWidget()
        mlay = QVBoxLayout(main_tab)
        mlay.addLayout(bar)
        mlay.addLayout(grid)
        tabs.addTab(main_tab, 'Teleop')
        tabs.addTab(radio, 'Radio Check')
        QVBoxLayout(self).addWidget(tabs)

        # ---- timers (started last, after pygame + widgets exist) ----
        self.timers = []
        for ms, fn in ((20, self.poll_inputs), (int(1000 / self.cfg['send_rate_hz']), self.tx_tick),
                       (200, self.refresh),
                       (2000, self.reconnect_tick),
                       (3000, self.refresh_ports)):
            t = QTimer(self)
            t.timeout.connect(fn)
            t.start(ms)
            self.timers.append(t)

        # ---- background UDP telemetry receiver (port 9003) ----
        self.udp_rx = UdpTelemetryReceiver(UDP_STATUS_PORT, self.on_telemetry_rx)
        self.udp_rx.start()

    # ---------------- helpers ----------------
    def refresh_ports(self):
        curr = self.port_box.currentText().strip() if hasattr(self, 'port_box') else ""
        new_items = [f'MASTER_HUB (UDP :{UDP_JOY_PORT})']
        try:
            for p in scan_available_ports():
                if p not in new_items:
                    new_items.append(p)
        except Exception:
            pass
        if self.cfg.get('port') and self.cfg['port'] not in new_items:
            new_items.append(self.cfg['port'])

        current_items = [self.port_box.itemText(i) for i in range(self.port_box.count())] if hasattr(self, 'port_box') else []
        if new_items != current_items and hasattr(self, 'port_box'):
            self.port_box.clear()
            for item in new_items:
                self.port_box.addItem(item)
            if curr:
                idx = self.port_box.findText(curr)
                if idx >= 0:
                    self.port_box.setCurrentIndex(idx)
                else:
                    self.port_box.setEditText(curr)
            elif self.cfg.get('port'):
                idx = self.port_box.findText(self.cfg['port'])
                if idx >= 0:
                    self.port_box.setCurrentIndex(idx)

        # Also sync to radio check tab r_port if present
        if hasattr(self, 'r_port'):
            r_curr = self.r_port.currentText().strip()
            r_items = [p for p in new_items if not p.startswith('MASTER_HUB')]
            r_existing = [self.r_port.itemText(i) for i in range(self.r_port.count())]
            if r_items != r_existing:
                self.r_port.clear()
                for item in r_items:
                    self.r_port.addItem(item)
                if r_curr:
                    idx = self.r_port.findText(r_curr)
                    if idx >= 0:
                        self.r_port.setCurrentIndex(idx)

    def on_kb(self, on):
        self.keyboard_drive = on
        self.keys.clear()

    # ---------------- gamepad / keyboard ----------------
    def drop_gamepad(self):
        self.joy, self.joy_ok = None, False
        self.raw_axes, self.buttons = [], []
        self.hats = []

    def scan_gamepad(self):
        """Hot-plug aware: no quit()/init() cycling. Relies on pygame.event.pump() being called first."""
        # 1) unplugged?
        for ev in pygame.event.get(pygame.JOYDEVICEREMOVED):
            try:
                if self.joy and ev.instance_id == self.joy.get_instance_id():
                    self.drop_gamepad()
            except Exception:
                self.drop_gamepad()
        # 2) plugged in (or already present at start-up)?
        pygame.event.get(pygame.JOYDEVICEADDED)          # drain; get_count() is already up to date
        if not self.joy_ok:
            n = pygame.joystick.get_count()
            if n:
                try:
                    self.joy = pygame.joystick.Joystick(min(self.cfg['joy_index'], n - 1))
                    self.joy.init()                      # harmless on pygame 2, required on pygame 1.9
                    self.joy_ok = True
                except pygame.error:
                    self.drop_gamepad()

    def poll_inputs(self):
        """Read gamepad (XInput first, pygame fallback) / keyboard; feed TxCore when it exists."""
        axes = buttons = raw = None
        self.xin_ok = False
        xs = self.xpad.read(self.cfg['joy_index'])          # direct Windows XInput (independent of pygame)
        if xs is not None:
            self.xin_ok = True
            raw, buttons = xs
            # Synthetic D-Pad from XInputPad.read (axes 6 & 7)
            hx = int(round(raw[6])) if len(raw) > 6 else 0
            hy = -int(round(raw[7])) if len(raw) > 7 else 0  # flip so up = +1, down = -1
            self.hats = [(hx, hy)]
        else:
            try:
                pygame.event.pump()
                self.scan_gamepad()
                if self.joy_ok:
                    raw = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
                    buttons = [self.joy.get_button(i) for i in range(self.joy.get_numbuttons())]
                    self.hats = [self.joy.get_hat(i) for i in range(self.joy.get_numhats())]
            except pygame.error:
                self.drop_gamepad()
                raw = buttons = None
                self.hats = []
        if raw is not None:
            self.raw_axes, self.buttons = raw, buttons
            dz = self.cfg['joy_deadzone']
            axes = [0.0 if abs(v) < dz else -v for v in raw]      # SDL up/left = - -> flip to ROS convention
        else:
            self.raw_axes, self.buttons = [], []
            self.hats = []
        if axes is None and self.keyboard_drive:
            axes, buttons = keyboard_axes(self.keys, self.cfg)
            self.raw_axes, self.buttons = [], buttons
            self.hats = []
        if not self.tx:
            return
        if axes is None:
            self.tx.no_input()                  # joystick data goes stale -> zeros are sent
        else:
            self.tx.on_joy(axes, buttons, raw_axes=self.raw_axes, hats=self.hats)

    # ---------------- science actuator methods ----------------
    def on_telemetry_rx(self, data):
        """Callback from background UDP telemetry receiver."""
        self.on_radio(data)

    def send_science_cmd(self):
        """Dispatch dual-redundant 'S' packet via active link and BaseMaster UDP hub."""
        with self.sci_lock:
            self.sci_seq = (self.sci_seq + 1) & 0xFF
            seq = self.sci_seq
            m1 = int(bool(self.sci_cmd.get('m1', 0)))
            m2 = int(bool(self.sci_cmd.get('m2', 0)))
            m3 = int(bool(self.sci_cmd.get('m3', 0)))
            m4 = int(bool(self.sci_cmd.get('m4', 0)))
            m5 = int(bool(self.sci_cmd.get('m5', 0)))
            led = int(bool(self.sci_cmd.get('led', 0)))
            servo = max(0, min(180, int(self.sci_cmd.get('servo', 0))))

        pkt = pack_science(seq, m1, m2, m3, m4, m5, led, servo)

        sent = False
        if self.link and self.link.connected:
            sent = self.link.send(pkt)

        # Fallback / Dual broadcast to local Master Hub (UDP 9001)
        if not (self.link and isinstance(self.link, UdpLink)):
            try:
                import socket
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.sendto(pkt, (MASTER_IP, UDP_JOY_PORT))
                s.close()
                sent = True
            except Exception:
                pass

        return sent

    def toggle_motor(self, idx):
        """Toggle peristaltic motor (1..5) between ON and OFF."""
        key = f'm{idx}'
        with self.sci_lock:
            curr = self.sci_cmd.get(key, self.sci_confirmed.get(key, 0))
            self.sci_cmd[key] = 0 if curr else 1
        self.send_science_cmd()

    def toggle_led(self):
        """Toggle illumination / UV LED between ON and OFF."""
        with self.sci_lock:
            curr = self.sci_cmd.get('led', self.sci_confirmed.get('led', 0))
            self.sci_cmd['led'] = 0 if curr else 1
        self.send_science_cmd()

    def step_servo(self, delta):
        """Step sample carousel servo angle by delta degrees (0 to 180)."""
        with self.sci_lock:
            curr = self.sci_cmd.get('servo', self.sci_confirmed.get('servo', 0))
            self.sci_cmd['servo'] = max(0, min(180, curr + delta))
        self.send_science_cmd()

    def stop_all_motors(self):
        """Emergency stop: Turn all 5 peristaltic motors OFF immediately."""
        with self.sci_lock:
            for i in range(1, 6):
                self.sci_cmd[f'm{i}'] = 0
        self.send_science_cmd()

    def update_science_telemetry(self, sc):
        """Update confirmed actuator states from received rover telemetry."""
        if not isinstance(sc, dict):
            return
        with self.sci_lock:
            for k in ('m1', 'm2', 'm3', 'm4', 'm5', 'led', 'servo'):
                if k in sc:
                    self.sci_confirmed[k] = int(sc[k])
                    if self.sci_cmd.get(k) == self.sci_confirmed[k]:
                        self.sci_cmd[k] = self.sci_confirmed[k]
            if 'connected' in sc:
                self.sci_confirmed['connected'] = bool(sc['connected'])
            self.sci_confirmed['last_update'] = time.monotonic()

    def eventFilter(self, obj, ev):
        t = ev.type()
        if t == QEvent.ApplicationDeactivate:
            self.keys.clear()
            return False

        if t == QEvent.KeyPress:
            # Intercept Science Actuator shortcuts: Ctrl + (1-6) and Ctrl + (Left/Right)
            if ev.modifiers() & Qt.ControlModifier:
                k = ev.key()

                # Motor 1..5, LED 6, All-Stop Z
                if not ev.isAutoRepeat():
                    if k in (Qt.Key_1, getattr(Qt, 'Key_Numpad1', 0x01000031)):
                        self.toggle_motor(1)
                        return True
                    elif k in (Qt.Key_2, getattr(Qt, 'Key_Numpad2', 0x01000032)):
                        self.toggle_motor(2)
                        return True
                    elif k in (Qt.Key_3, getattr(Qt, 'Key_Numpad3', 0x01000033)):
                        self.toggle_motor(3)
                        return True
                    elif k in (Qt.Key_4, getattr(Qt, 'Key_Numpad4', 0x01000034)):
                        self.toggle_motor(4)
                        return True
                    elif k in (Qt.Key_5, getattr(Qt, 'Key_Numpad5', 0x01000035)):
                        self.toggle_motor(5)
                        return True
                    elif k in (Qt.Key_6, getattr(Qt, 'Key_Numpad6', 0x01000036)):
                        self.toggle_led()
                        return True
                    elif k == Qt.Key_Z:
                        self.stop_all_motors()
                        return True

                # Servo angle: Ctrl + Left Arrow / Ctrl + Right Arrow
                if k == Qt.Key_Left:
                    self.step_servo(-15)
                    return True
                elif k == Qt.Key_Right:
                    self.step_servo(15)
                    return True

        if t in (QEvent.KeyPress, QEvent.KeyRelease) and self.keyboard_drive:
            name = KEYMAP.get(ev.key())
            if name is None or isinstance(QApplication.focusWidget(), QLineEdit):
                return False
            if not ev.isAutoRepeat():
                (self.keys.add if t == QEvent.KeyPress else self.keys.discard)(name)
            return True
        return False

    # ---------------- connection ----------------
    def stop_link(self):
        if self.link:
            self.link.close()

    def do_connect(self):
        self.stop_link()
        try:
            port, baud = self.port_box.currentText().strip(), int(self.baud_box.currentText())
            remote_text = self.addr_edit.text().strip()
            remote = norm_addr(remote_text) if remote_text else '000000000000FFFF'
            if remote and len(remote) != 16:
                raise ValueError("Address must be 16 hex digits (or leave blank / FFFF for broadcast)")
        except Exception as e:
            QMessageBox.warning(self, 'Bad settings', f'Check baud / radio address:\n{e}')
            return
        self.cfg['port'], self.cfg['baud'] = port, baud
        if 'MASTER' in port.upper() or 'UDP' in port.upper():
            self.link = UdpLink(MASTER_IP, UDP_JOY_PORT, self.on_radio)
        else:
            self.link = XBeeLink(port, baud, remote, self.on_radio)
        self.tx = TxCore(self.link, self.cfg)           # fresh TxCore -> no stale ACK carried over
        self.auto = True
        self.start_connect()

    def start_connect(self):
        if self.connecting or not self.link or self.link.connected:
            return
        self.connecting, link = True, self.link

        def work():
            try:
                link.connect()
                if link is not self.link or not self.auto:      # replaced / disconnected meanwhile
                    link.close()
            finally:
                self.connecting = False
        threading.Thread(target=work, daemon=True).start()

    def toggle_connect(self):
        if self.auto:
            self.auto = False
            self.stop_link()
        else:
            self.do_connect()

    def reconnect_tick(self):
        if self.auto and self.link and not self.link.connected:
            self.start_connect()

    def on_radio(self, data):                   # XBee & UDP: ACKs and Telemetry from the robot
        data = bytes(data)
        if not data:
            return

        # 1. ACK packet ('A', seq, lost)
        if data[:1] == b'A' and len(data) == _ACK.size:
            if self.tx:
                self.tx.on_radio(data)
            return

        # 2. Telemetry JSON packet ('T' + json or direct json)
        json_bytes = None
        if data[:1] == b'T':
            json_bytes = data[1:]
        elif data[:1] == b'{':
            json_bytes = data

        if json_bytes:
            try:
                payload = json.loads(json_bytes.decode('utf-8', errors='ignore'))
                if isinstance(payload, dict):
                    if "science" in payload:
                        self.update_science_telemetry(payload["science"])
                return
            except Exception:
                pass

    # ---------------- tickers ----------------
    def tx_tick(self):
        if self.tx and self.link and self.link.connected:
            self.tx.send_tick()

    # ---------------- radio tab ----------------
    def run_radio(self, fix, send):
        if self.radio_worker and self.radio_worker.isRunning():
            return
        self.radio_log.clear()
        if self.auto or (self.link and self.link.connected):
            self.auto = False
            self.stop_link()
            self.radio_log.append('(teleop link closed to free the port - press Connect again afterwards)')
        for b in self.radio_btns:
            b.setEnabled(False)
        try:
            baud = int(self.r_baud.currentText())
        except ValueError:
            baud = 115200
        w = self.radio_worker = RadioWorker(self.r_port.currentText().strip(), baud, fix, send)
        w.line.connect(self.radio_log.append)
        w.finished.connect(lambda: [b.setEnabled(True) for b in self.radio_btns])
        w.start()

    # ---------------- refresh ----------------
    def refresh(self):
        link = self.link
        if not self.auto:
            self.btn_connect.setText('Connect')
        else:
            self.btn_connect.setText('Disconnect' if link and link.connected else 'Connecting... (click to stop)')
        tx = self.tx.status() if self.tx else None

        # ---- Joystick panel ----
        try:
            seen = pygame.joystick.get_count()
        except pygame.error:
            seen = 0
        if self.xin_ok or self.joy_ok:
            if self.xin_ok:
                name, how = f"XInput slot {self.cfg['joy_index']}", 'XInput'
            else:
                how = 'pygame'
                try:
                    name = self.joy.get_name()
                except pygame.error:
                    name = '-'
            self.joy_p.state(GREEN, f'gamepad connected ({how})')

            # Show raw LT/RT values and active button names
            raw = self.raw_axes
            lt_disp = f"{raw[4]:+.2f}" if len(raw) > 4 else '-'
            rt_disp = f"{raw[5]:+.2f}" if len(raw) > 5 else '-'

            btns, contract, expand, _ = get_controller_state(self.buttons, self.raw_axes, self.hats)
            active_names = [k for k, v in btns.items() if v]
            active_str = ', '.join(active_names) if active_names else '-'

            mode_str = 'CONTRACT' if (contract and not expand) else ('EXPAND' if (expand and not contract) else 'none')

            self.joy_p.put(
                pad=name, seen=seen,
                btn=f"{active_str}  [IDs: {', '.join(str(i) for i, b in enumerate(self.buttons) if b) or '-'}]",
                raw='  '.join(f'{i}:{a:+.2f}' for i, a in enumerate(raw)) or '-',
                trig=f"LT={lt_disp} RT={rt_disp} | Action={mode_str}",
            )
            ax = self.raw_axes + [0.0] * 4
            for i in range(4):
                self.sticks[i].set(ax[i])
        else:
            self.joy_p.state(AMBER if self.keyboard_drive else RED,
                             'keyboard drive' if self.keyboard_drive else 'no gamepad found (plug it in)')
            self.joy_p.put(pad='-', seen=seen, btn='-', raw='-', trig='-')
            for b in self.sticks.values():
                b.set(0)
        self.joy_p.put(deadman='-' if tx is None else ('HELD' if tx['enable'] else 'released'))

        # ---- Drive Command panel ----
        if tx is not None:
            moving = abs(tx['vx']) > 1e-3 or abs(tx['wz']) > 1e-3
            self.cmd.state(GREEN if moving else AMBER, 'driving' if moving else 'zero / stopped')
            self.cmd.put(vx=f"{tx['vx']:+.3f} m/s", wz=f"{tx['wz']:+.3f} rad/s")
            self.vx_bar.set(tx['vx'] / max(self.cfg['scale_linear'], 1e-3))
            self.wz_bar.set(tx['wz'] / max(self.cfg['scale_angular'], 1e-3))
        else:
            self.cmd.state(GREY, 'not connected')
            self.cmd.clear()
            self.vx_bar.set(0)
            self.wz_bar.set(0)

        # ---- Arm Control panel ----
        def _arm_label(v):
            """Convert -1/0/+1 to a readable string with colour hint in the text."""
            if v < -0.5:
                return 'CONTRACT  (-1)'
            if v > 0.5:
                return 'EXPAND    (+1)'
            return 'still      ( 0)'

        if tx is not None:
            arm_active = any(abs(tx[k]) > 0.5 for k in ('act1', 'act2', 'act3', 'wrist', 'gripper'))
            self.arm_p.state(GREEN if arm_active else GREY,
                             'arm moving' if arm_active else 'arm idle')
            self.arm_p.put(
                act1=_arm_label(tx['act1']),
                act2=_arm_label(tx['act2']),
                act3=_arm_label(tx['act3']),
                wrist=_arm_label(tx['wrist']),
                gripper=_arm_label(tx['gripper']),
            )
        else:
            self.arm_p.state(GREY, 'not connected')
            self.arm_p.clear()

        # ---- XBee TX panel ----
        if tx is not None:
            sending = tx['link'] and tx['rate'] > 0.5
            self.tx_p.state(GREEN if sending else (AMBER if tx['link'] else RED),
                            'radio sending (does NOT prove robot software is running)' if sending
                            else ('radio open, idle' if tx['link'] else 'radio not open'))
            self.tx_p.put(port=f"{tx['port']} @ {tx['baud']}", local=tx['local'] or '-', remote=tx['remote'],
                          sent=tx['sent'], rate=f"{tx['rate']:.1f} Hz", ok=f"{tx['tx_ok']} / {tx['tx_fail']}",
                          why=tx['why'] or '-', err=tx['err'] or '-')
        else:
            self.tx_p.state(GREY, 'not connected')
            self.tx_p.clear()

        # ---- Robot ACK panel ----
        # ACK from the robot's rx_node — the only real proof the robot software is alive
        ack = tx['ack'] if tx and tx['ack'] and tx['ack']['age'] < 2.0 else None
        if ack is not None:
            self.rx_p.state(GREEN, 'robot rx_node ACK received')
            self.rx_p.put(n=ack['seq'], cnt=tx['ack_count'], age=f"{ack['age'] * 1000:.0f} ms",
                          lost=ack['lost'], src=link.last_src or '-', ign=link.rx_ignored)
        else:
            waiting = tx is not None and tx['link']
            self.rx_p.state(RED if waiting else GREY,
                            'no ACK from robot rx_node' if waiting else 'no data')
            self.rx_p.clear()
            if link:
                self.rx_p.put(src=link.last_src or '-', ign=link.rx_ignored)

        # ---- Science Actuators panel ----
        if hasattr(self, 'sci_p'):
            with self.sci_lock:
                sci_copy = dict(self.sci_confirmed)
            self.sci_p.update_display(sci_copy)

    def closeEvent(self, e):
        self.auto = False
        if hasattr(self, 'udp_rx') and self.udp_rx:
            self.udp_rx.stop()
        for t in self.timers:
            t.stop()
        if self.radio_worker and self.radio_worker.isRunning():
            self.radio_worker.wait(3000)
        # Safety: send one final all-zeros MOTION packet before disconnecting
        if self.tx and self.link and self.link.connected:
            self.link.send(pack_motion(self.tx.seq, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.stop_link()
        pygame.quit()
        e.accept()


def _excepthook(et, ev, tb):
    """Never let a callback error kill the app. Works in a --windowed .exe (no console) too:
    errors are appended to xbee_teleop_error.log next to the exe / script."""
    import traceback
    text = ''.join(traceback.format_exception(et, ev, tb))
    try:
        if sys.stderr:
            sys.stderr.write(text)
        base = os.path.dirname(sys.executable if getattr(sys, 'frozen', False) else os.path.abspath(__file__))
        with open(os.path.join(base, 'xbee_teleop_error.log'), 'a') as f:
            f.write(time.strftime('%Y-%m-%d %H:%M:%S\n') + text + '\n')
    except Exception:
        pass


def main():
    sys.excepthook = _excepthook
    if sys.stdout:
        print('XInput available:', XInputPad().fn is not None, '| pygame', pygame.version.ver, flush=True)
    app = QApplication(sys.argv)
    win = MainWindow()
    app.installEventFilter(win)
    win.resize(1150, 720)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
