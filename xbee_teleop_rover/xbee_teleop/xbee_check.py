#!/usr/bin/env python3
"""Radio check:  ros2 run xbee_teleop xbee_check /dev/ttyUSB0 115200 [--send] [--fix]

Prints the radio settings that matter. Works even if the radio is still in transparent mode
(AP=0): it then talks AT commands over the serial port.
  --fix   set AP=1 (API mode) and save (WR)
  --send  broadcast 20 zero command frames (watch the TX LED)
"""
import argparse
import sys
import time

import serial

from xbee_teleop.link import pack_cmd

BD_NAMES = {3: '9600', 6: '57600', 7: '115200'}


def at_session(port, baud, fix):
    """Read settings with AT commands (radio in AP=0 or AP=1 idle)."""
    s = serial.Serial(port, baud, timeout=1.5)
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
        print('setting AP=1 and writing to flash:', at('AP1'), at('WR'))
        out['AP'] = at('AP')
    at('CN')
    s.close()
    return out


def api_session(port, baud, send):
    from digi.xbee.devices import XBeeDevice
    dev = XBeeDevice(port, baud)
    dev.open()
    try:
        out = dict(addr=str(dev.get_64bit_addr()), proto=str(dev.get_protocol()),
                   fw=dev.get_firmware_version(),
                   AP=dev.get_parameter('AP').hex(), BD=dev.get_parameter('BD').hex(),
                   ID=dev.get_parameter('ID').hex())
        if send:
            print('broadcasting 20 zero frames (watch the TX LED) ...')
            for i in range(20):
                dev.send_data_broadcast(pack_cmd(i, 0.0, 0.0))
                time.sleep(0.05)
            print('done')
        return out
    finally:
        dev.close()


def show(addr, proto, fw, ap, bd, net):
    ap_i, bd_i = int(ap, 16), int(bd, 16)
    print(f'64-bit addr : {addr}  <- put this in the OTHER side\'s params.yaml remote_addr')
    print(f'protocol    : {proto}')
    print(f'firmware    : {fw}')
    print(f'AP (API)    : {ap_i:02X}  ({"OK, API mode" if ap_i == 1 else "NOT API mode - run again with --fix"})')
    print(f'BD (baud)   : {bd_i:08X}  ({BD_NAMES.get(bd_i, "?")} baud; 7 = 115200)')
    print(f'ID (network): {int(net, 16):X}  (must match on both radios)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('port', nargs='?', default='/dev/ttyUSB0')
    ap.add_argument('baud', nargs='?', type=int, default=115200)
    ap.add_argument('--send', action='store_true')
    ap.add_argument('--fix', action='store_true', help='set AP=1 (API mode) and save')
    a = ap.parse_args()
    print(f'port        : {a.port} @ {a.baud}')

    if a.fix:
        r = at_session(a.port, a.baud, True)
        show(r['SH'].zfill(8) + r['SL'].zfill(8), '-', r['VR'], r['AP'], r['BD'], r['ID'])
        print('now power-cycle / re-plug the radio, then run again.')
        return
    try:
        r = api_session(a.port, a.baud, a.send)
        show(r['addr'], r['proto'], r['fw'], r['AP'], r['BD'], r['ID'])
    except Exception as e:  # noqa: BLE001
        print(f'API mode open failed ({e}); trying AT command mode ...')
        try:
            r = at_session(a.port, a.baud, False)
        except Exception as e2:  # noqa: BLE001
            sys.exit(f'failed: {e2}')
        show(r['SH'].zfill(8) + r['SL'].zfill(8), '-', r['VR'], r['AP'], r['BD'], r['ID'])


if __name__ == '__main__':
    main()
