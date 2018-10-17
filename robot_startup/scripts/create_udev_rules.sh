#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  eai-ydlidar and serial module"
echo "ydlidar usb connection as /dev/ydlidar , check it using the command : ls -l /dev|grep ttyUSB"
echo "serial usb connection as /dev/mecanum_base , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy robot.rules to  /etc/udev/rules.d/"

sudo cp ./robot.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish create"
