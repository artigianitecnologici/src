#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for PANTILT_DIR in \
  "$SCRIPT_DIR/../marrtinorobot2_dynamixel/marrtinorobot2_dynamixel" \
  "$HOME/src/marrtinorobot2/marrtinorobot2_dynamixel/marrtinorobot2_dynamixel" \
  "$HOME/marrtinorobot2_ws/src/marrtinorobot2/marrtinorobot2_dynamixel/marrtinorobot2_dynamixel" \
  "$HOME/marrtinorobot2_ws/marrtinorobot2/marrtinorobot2_dynamixel/marrtinorobot2_dynamixel"; do
  if [ -f "$PANTILT_DIR/pan_tilt_controller.py" ]; then
    cd "$PANTILT_DIR" || exit 1
    exec python3 pan_tilt_controller.py
  fi
done

echo "Errore: pan_tilt_controller.py non trovato." >&2
exit 1
