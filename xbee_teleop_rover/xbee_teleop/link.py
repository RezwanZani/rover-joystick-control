"""XBee link (radio in API mode, AP=1) + packet helpers.

Packet types
------------
  CMD    TX -> RX (legacy, 'C') : seq u8, vx i16, wz i16
  MOTION TX -> RX (new,    'M') : seq u8, vx i16, wz i16,
                                   act1 i16, act2 i16, act3 i16, wrist i16, gripper i16
  ACK    RX -> TX ('A')         : last seq u8, lost u16

Transport
---------
  Pure non-blocking serial (timeout=0, write_timeout=0).
  XBee API frame 0x10 with Frame ID 0x00 and Option 0x01 (Disable RF ACK)
  to ensure zero-stutter 20 Hz teleop alongside heavy RTCM or bidirectional data.
"""
import struct
import threading
import time
import serial

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

# Science actuator packet: 'S' seq m1 m2 m3 m4 m5 led servo  (9 bytes)
_SCIENCE = struct.Struct('<cBBBBBBBB')

# Operational mode packet: 'N' seq mode  (3 bytes: 0=TELEOP, 1=AUTONOMOUS)
_MODE = struct.Struct('<cBB')


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


def pack_science(seq, m1, m2, m3, m4, m5, led, servo):
    """Pack Science actuator command packet ('S', 9 bytes)."""
    return _SCIENCE.pack(
        b'S', seq & 0xFF,
        int(bool(m1)), int(bool(m2)), int(bool(m3)), int(bool(m4)), int(bool(m5)),
        int(bool(led)), max(0, min(180, int(servo)))
    )


def pack_mode(seq, mode):
    """Pack Rover operational mode toggle packet ('N', 3 bytes).
    mode: 0 = TELEOP, 1 = AUTONOMOUS
    """
    return _MODE.pack(b'N', seq & 0xFF, int(mode) & 0xFF)


# ---------------------------------------------------------------------------
# Unpack (handles all packet types)
# ---------------------------------------------------------------------------
def unpack(data):
    """Decode an incoming XBee payload.

    Returns one of:
      ('motion', seq, vx, wz, act1, act2, act3, wrist, gripper)  -- new combined packet
      ('cmd',    seq, vx, wz)                                     -- legacy drive-only
      ('science', seq, m1, m2, m3, m4, m5, led, servo)           -- science actuators
      ('mode',    seq, mode)                                      -- operational mode
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

    # Science actuator packet
    if data[:1] == b'S' and len(data) == _SCIENCE.size:
        _, seq, m1, m2, m3, m4, m5, led, servo = _SCIENCE.unpack(data)
        return ('science', seq, m1, m2, m3, m4, m5, led, servo)

    # Operational mode packet
    if data[:1] == b'N' and len(data) == _MODE.size:
        _, seq, mode = _MODE.unpack(data)
        return ('mode', seq, mode)

    # ACK from rover
    if data[:1] == b'A' and len(data) == _ACK.size:
        _, seq, lost = _ACK.unpack(data)
        return ('ack', seq, lost)

    return None



def norm_addr(a):
    return str(a).strip().upper().replace('0X', '').zfill(16)


def wrap_xbee_api_tx(raw_payload: bytes, dest_addr_hex: str = '') -> bytes:
    """Build an XBee API mode (AP=1) Transmit Request frame (type 0x10).

    Settings:
      - 0x10 = Transmit Request
      - 0x00 = Frame ID (0 disables transmit status 0x8B callback, eliminating serial overhead)
      - dest_addr = 64-bit Address (defaults to 00..FFFF broadcast)
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
    """Non-blocking, zero-stutter XBee API serial link.

    - Uses write_timeout=0 to guarantee serial.write() never blocks the 20 Hz loop.
    - Sends 0x10 Transmit Request with Frame ID 0x00 and Option 0x01 (Disable RF ACK).
    - Runs a background thread parsing incoming 0x90 Receive Packet frames.
    """

    def __init__(self, port, baud, remote_addr, on_message):
        self.port = port
        self.baud = int(baud)
        self.remote_addr = norm_addr(remote_addr) if remote_addr else ''
        self.on_message = on_message            # callback: on_message(bytes)
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
            # write_timeout=0 ensures write() never freezes if serial buffer fills
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
            self.err = str(e)
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
        except (serial.SerialTimeoutException, serial.SerialException) as e:
            # Dropped cleanly without blocking or freezing the loop
            self.tx_fail += 1
            self.last_fail = str(e)
            return False
        except Exception as e:
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

                # Parse all complete 0x7E frames in the buffer
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
                        break  # Incomplete frame, wait for more bytes

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

            # Accept if remote_addr is empty, broadcast (FFFF), or matches sender (ignore filter on 0x10 loopback)
            if (frame_type == 0x90 and
                self.remote_addr and 
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
