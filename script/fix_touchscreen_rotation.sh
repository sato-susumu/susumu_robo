#!/bin/bash

# Fix touchscreen rotation for portrait mode display
# This script corrects the coordinate mapping when display is rotated 90 degrees right

echo "Fixing touchscreen rotation for portrait mode..."

# Find WingCool touchscreen device IDs
TOUCH_IDS=$(xinput list | grep "WingCool Inc. TouchScreen" | grep -o "id=[0-9]*" | cut -d= -f2)

if [ -z "$TOUCH_IDS" ]; then
    echo "Error: No WingCool touchscreen devices found"
    exit 1
fi

# Apply 90-degree right rotation transformation matrix to each touchscreen
for id in $TOUCH_IDS; do
    echo "Applying rotation matrix to touchscreen ID: $id"
    xinput set-prop $id "Coordinate Transformation Matrix" 0 1 0 -1 0 1 0 0 1
done

echo "Touchscreen rotation fix applied successfully"
echo "Touch positions should now align correctly with the display"