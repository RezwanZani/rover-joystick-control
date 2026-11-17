# xbee_teleop (ROS 2 Jazzy)

gamepad -> `joy_node` -> **tx_node** ==XBee (DigiMesh, API mode)== **rx_node** -> `/cmd_vel`
plus a PyQt5 **gui** (joystick, command, XBee TX, XBee RX).

## Install
    sudo apt install ros-jazzy-joy python3-pyqt5 python3-serial
    pip install digi-xbee --break-system-packages
    sudo usermod -aG dialout $USER            # log out/in once
    cd ~/ros2_ws && colcon build --packages-select xbee_teleop && source install/setup.bash

## Radios (already matching: ID=3196, BD=7)
| side | 64-bit address   | AP |
|------|------------------|----|
| TX (gamepad PC) | 0013A200425DD789 | 1 OK |
| RX (robot)      | 0013A200425E9501 | **0 -> must become 1** |

Fix the receiver radio (plug it in, then):
    ros2 run xbee_teleop xbee_check /dev/ttyUSB0 115200 --fix     # sets AP=1 + WR
    # unplug/replug, then verify:
    ros2 run xbee_teleop xbee_check /dev/ttyUSB0 115200 --send
Each side's `remote_addr` in `config/params.yaml` is the OTHER radio's address (already filled in).

## Run
    ros2 launch xbee_teleop tx.launch.py          # gamepad PC (joy + tx_node + GUI)
    ros2 launch xbee_teleop rx.launch.py          # robot (add gui:=true if it has a screen)
Hold LB (button 9) to drive; left stick = speed/turn. The GUI lists pressed button numbers
and raw stick axes - change `enable_button` / `axis_*` in `params.yaml` if your pad differs.

## Safety
- Deadman released or no `/joy` for 0.5 s -> TX keeps sending zeros.
- RX publishes zero `/cmd_vel` if nothing arrives for `cmd_timeout`.
