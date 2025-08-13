#!/bin/bash

echo "Creating udev rules file..."
sudo bash -c 'cat > /etc/udev/rules.d/99-ublox-tty.rules << EOF
# For F9P module (base)
KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK="tty_f9p", GROUP="dialout", MODE="0666"

# For F9H module with HL-340 adapter (rover)
KERNEL=="ttyUSB[0-9]*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK="tty_f9h", GROUP="dialout", MODE="0666"
EOF'

echo "Reloading udev rules..."
sudo udevadm control --reload-rules

echo "Triggering udev rules..."
sudo udevadm trigger

echo "Setup complete. Please reconnect your devices if necessary."