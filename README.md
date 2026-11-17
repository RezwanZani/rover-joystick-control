# XBee Teleop — Drive + Arm Control

Wireless Xbox gamepad control for a ROS 2 rover, transmitted over XBee DigiMesh radio.

```
Xbox Pad  ──►  xbee_teleop_windows.py  ══ XBee RF ══►  rx_node (ROS 2)
               (Windows base station)                   (rover onboard)
                                                           │
                                                     /cmd_vel  (drive)
                                                     /arm_vel  (arm)
```

---

## Project Layout

```
joy_control/
├── run_teleop_base/            ← Windows base station (Python, no ROS needed)
│   ├── xbee_teleop_windows.py  ← Main script: reads gamepad, sends XBee packets
│   ├── requirements.txt        ← pip dependencies
│   ├── run_teleop.bat          ← Double-click launcher (creates .venv on first run)
│   └── README_WINDOWS.md       ← Quick-start for the base station
│
└── xbee_teleop_rover/          ← ROS 2 package (Jazzy) for the rover
    ├── xbee_teleop/
    │   ├── link.py             ← XBee link + packet definitions (shared protocol)
    │   ├── rx_node.py          ← Receives packets → publishes /cmd_vel + /arm_vel
    │   ├── tx_node.py          ← (optional) ROS 2 transmitter using joy_node
    │   └── gui.py              ← PyQt5 status window
    ├── config/params.yaml      ← All ROS parameters
    └── launch/
        ├── rx.launch.py        ← Launch on the rover
        └── tx.launch.py        ← Launch on a Linux base station (optional)
```

---

## Packet Protocol

A single **MOTION packet** carries both drive and arm commands in every frame (20 Hz).

| Offset | Field     | Type | Description                          |
|--------|-----------|------|--------------------------------------|
| 0      | `'M'`     | char | Packet type identifier               |
| 1      | seq       | u8   | Rolling sequence number (0–255)      |
| 2–3    | vx        | i16  | Forward velocity × 1000 (m/s)        |
| 4–5    | wz        | i16  | Turn rate × 1000 (rad/s)             |
| 6–7    | act1      | i16  | Actuator 1: −1000 / 0 / +1000        |
| 8–9    | act2      | i16  | Actuator 2: −1000 / 0 / +1000        |
| 10–11  | act3      | i16  | Actuator 3: −1000 / 0 / +1000        |
| 12–13  | wrist     | i16  | Wrist stepper: −1000 / 0 / +1000     |
| 14–15  | gripper   | i16  | Gripper gear motor: −1000 / 0 / +1000|

**Total: 16 bytes.** Little-endian. Format string: `<cBhhhhhhh`

An **ACK packet** (`'A'`, 5 bytes) is sent back from the rover at 2 Hz so the base station can display packet loss.

> **Backward compatibility:** The old 6-byte `'C'` (CMD) packet is still decoded by `rx_node` for compatibility with older base station versions.

---

## Xbox Gamepad Control Mapping

### Drive (Both Throttle Sticks — Unchanged)

| Input            | Action                         |
|------------------|--------------------------------|
| Left Stick Y     | Forward / backward (`vx`)      |
| Right Stick X    | Turn left / right (`wz`)       |

Drive is always active (no deadman button required by default).  
To require holding LB to drive, set `require_enable: true` in `params.yaml`.

---

### Robotic Arm Control

The arm uses a **button + trigger/bumper** combination:
- Hold an **arm button** (A / B / X / Y) to select which joint to move.
- Press **LT** (Left Trigger) OR **LB** (Left Bumper) to **contract** that joint → value = −1.
- Press **RT** (Right Trigger) OR **RB** (Right Bumper) to **expand** that joint → value = +1.
- **Release** the button at any time → that joint **holds its current position** (value = 0).

```
                 ┌─────────────────────────────────────────┐
                 │          Xbox Controller                │
                 │                                         │
                 │  LT / LB ────────────────────── RT / RB │
                 │  (contract)                 (expand)    │
                 │                                         │
                 │                           Y (wrist)     │
                 │   D-Pad                B (act2) A (act1)│
                 │   ↑ Gripper-         X (act3)           │
                 │     contract                            │
                 │   ↓ Gripper-                            │
                 │     expand                              │
                 └─────────────────────────────────────────┘
```

| Button Combo            | Joint / Motor           | Direction          | Value |
|-------------------------|-------------------------|--------------------|-------|
| **A** + (LT or LB)      | Actuator 1 (linear)     | Contract           | −1    |
| **A** + (RT or RB)      | Actuator 1 (linear)     | Expand             | +1    |
| **B** + (LT or LB)      | Actuator 2 (linear)     | Contract           | −1    |
| **B** + (RT or RB)      | Actuator 2 (linear)     | Expand             | +1    |
| **X** + (LT or LB)      | Actuator 3 (linear)     | Contract           | −1    |
| **X** + (RT or RB)      | Actuator 3 (linear)     | Expand             | +1    |
| **Y** + (LT or LB)      | Wrist (stepper)         | Contract           | −1    |
| **Y** + (RT or RB)      | Wrist (stepper)         | Expand             | +1    |
| **D-Pad Up**            | Gripper (gear motor)    | Contract (close)   | −1    |
| **D-Pad Down**          | Gripper (gear motor)    | Expand  (open)     | +1    |
| Release any button      | That joint              | **Hold position**  |  0    |

> **Trigger threshold:** A trigger must be pressed more than ~30% from its rest position to register as a command. Adjust `trigger_threshold` in the CONFIG block of `xbee_teleop_windows.py` if needed.

> **Multiple joints:** You can command multiple joints simultaneously (e.g., hold A + B + LT to contract both actuator 1 and 2 at the same time).

---

## ROS 2 Topics (Rover Side)

| Topic          | Type                            | Description                          |
|----------------|---------------------------------|--------------------------------------|
| `/cmd_vel`     | `geometry_msgs/Twist`           | Drive: `linear.x` = vx, `angular.z` = wz |
| `/arm_vel`     | `std_msgs/Float32MultiArray`    | Arm: `[act1, act2, act3, wrist, gripper]` |
| `xbee_rx/status` | `std_msgs/String` (JSON)      | Diagnostic: link stats, all values   |

**`/arm_vel` data layout:**

```
index 0 = act1    (Actuator 1)
index 1 = act2    (Actuator 2)
index 2 = act3    (Actuator 3)
index 3 = wrist   (Wrist stepper)
index 4 = gripper (Gripper gear motor)
```

Values: `−1.0` = contract · `0.0` = still · `+1.0` = expand

---

## Setup

### Radio Configuration
Both XBee radios must have:
- **AP = 1** (API mode)
- **BD = 7** (115200 baud)
- **Same ID** (network PAN ID, e.g. 3196)

| Side              | 64-bit Address        |
|-------------------|-----------------------|
| Base station (TX) | `0013A200425DD789`    |
| Rover (RX)        | `0013A200425E9501`    |

Each side's `remote_addr` is set to the **other** radio's address.

**Check / fix a radio (Linux):**
```bash
ros2 run xbee_teleop xbee_check /dev/ttyUSB0 115200 --fix   # sets AP=1 + saves
# unplug/replug the radio, then verify:
ros2 run xbee_teleop xbee_check /dev/ttyUSB0 115200 --send
```

**Check / fix a radio (Windows):** Use the **Radio Check** tab in `xbee_teleop_windows.py`.

---

### Base Station (Windows)

**One-time setup:**
```bat
rem Double-click run_teleop.bat  (creates .venv + installs packages on first run)
rem  OR manually:
py -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python xbee_teleop_windows.py
```

**Running:**
1. Plug in the XBee USB adapter. Find its COM port in Device Manager → Ports.
2. Plug in the Xbox gamepad.
3. Run the script. Select the COM port and baud (115200).
4. Enter the rover's radio address in "Robot radio addr".
5. Click **Connect**.
6. Use the gamepad — the Joystick, Drive Command, and Arm Control panels update in real time.

**Terminal output** (printed ~1 Hz):
```
[TX] seq= 42  |  drive: vx=+0.30  wz=+0.00  |  arm: act1=0  act2=0  act3=0  wrist=+1  gripper=0  |  rate=20?Hz  <<ARM ACTIVE>>
```

---

### Rover (ROS 2 Jazzy)

**Install:**
```bash
sudo apt install ros-jazzy-joy python3-pyqt5 python3-serial
pip install digi-xbee --break-system-packages
sudo usermod -aG dialout $USER    # log out/in once after this
cd ~/ros2_ws
colcon build --packages-select xbee_teleop
source install/setup.bash
```

**Run:**
```bash
ros2 launch xbee_teleop rx.launch.py          # rover (headless)
ros2 launch xbee_teleop rx.launch.py gui:=true  # rover with status GUI
```

**Monitor topics:**
```bash
ros2 topic echo /cmd_vel                      # drive commands
ros2 topic echo /arm_vel                      # arm commands
ros2 topic echo xbee_rx/status               # diagnostic JSON
```

**Terminal output on rover** (printed on every received packet):
```
[RX] seq= 42 | drive: vx=+0.300  wz=+0.000 | arm: act1=0  act2=0  act3=0  wrist=+1  gripper= 0
```

---

## Configuration (`config/params.yaml`)

```yaml
rx_node:
  ros__parameters:
    port: /dev/serial/by-id/...     # XBee USB serial port
    baud: 115200
    remote_addr: "0013A200425DD789" # base station radio address
    cmd_timeout: 0.5                # seconds before publishing zeros (failsafe)
    cmd_vel_topic: cmd_vel          # drive topic name
    arm_vel_topic: arm_vel          # arm topic name
```

---

## Safety

| Situation                                     | Behaviour                                 |
|-----------------------------------------------|-------------------------------------------|
| No gamepad for > `cmd_timeout` s (base)       | TX sends all-zeros MOTION packet          |
| No XBee packet for > `cmd_timeout` s (rover)  | Publishes zero `/cmd_vel` and `/arm_vel`  |
| Base station window closed                    | Sends one final all-zeros packet          |
| Rover shuts down                              | Publishes zero commands before exit       |
| Button released                               | That arm channel immediately → 0 (hold)   |

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `could not open port` | Wrong COM number, or another program holds the port (close XCTU / PuTTY) |
| `radio not in API mode` | Radio Check → Fix (sets AP=1) |
| `wrong source address` | The `remote_addr` on one side doesn't match the other radio |
| TX shows `delivered=0, failed>0` | Radios on different network ID, out of range, or wrong address |
| Gamepad not found | Plug it in; app rescans every second |
| Arm does nothing | Check trigger threshold — trigger must be pressed >30% from rest |
| D-Pad gripper not working | D-Pad is read as buttons via XInput on Windows; check raw axes 6/7 in Joystick panel |
| `/arm_vel` not appearing on rover | Check `arm_vel_topic` param in `params.yaml`; verify with `ros2 topic list` |
