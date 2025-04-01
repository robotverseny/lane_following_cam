#!/bin/bash

# STOP

echo "[INFO] Killing Screens"
killall -9 screen
echo "[INFO] Wiping empty Screens"
screen -wipe
chmod +x ~/ros2_ws/src/savtartas/shell/stop_all.sh
alias stop_all='~/ros2_ws/src/savtartas/shell/stop_all.sh'

