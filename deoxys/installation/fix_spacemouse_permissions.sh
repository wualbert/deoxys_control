#!/bin/bash

echo "Fixing SpaceMouse permissions..."
echo

# Reload udev rules
echo "Step 1: Reloading udev rules..."
sudo udevadm control --reload-rules

# Trigger udev to reapply rules
echo "Step 2: Triggering udev rules..."
sudo udevadm trigger

# Wait a moment for rules to apply
sleep 1

# Check if the SpaceMouse device exists and show its permissions
echo
echo "Step 3: Checking SpaceMouse device permissions..."
SPACEMOUSE_DEV=$(find /sys/devices -name "hidraw*" -path "*256F:C62E*" 2>/dev/null | grep -o "hidraw[0-9]*$" | head -1)

if [ -n "$SPACEMOUSE_DEV" ]; then
    echo "SpaceMouse found at /dev/$SPACEMOUSE_DEV"
    ls -la /dev/$SPACEMOUSE_DEV
    echo
    if [ -r "/dev/$SPACEMOUSE_DEV" ] && [ -w "/dev/$SPACEMOUSE_DEV" ]; then
        echo "✓ Permissions are correct! You should be able to use the SpaceMouse now."
    else
        echo "✗ Permissions still incorrect. Trying manual fix..."
        sudo chmod 666 /dev/$SPACEMOUSE_DEV
        sudo chgrp plugdev /dev/$SPACEMOUSE_DEV
        echo "Manual permissions applied. Check again:"
        ls -la /dev/$SPACEMOUSE_DEV
    fi
else
    echo "SpaceMouse device not found. Please make sure it's plugged in."
fi

echo
echo "Done! Try running your script again."
