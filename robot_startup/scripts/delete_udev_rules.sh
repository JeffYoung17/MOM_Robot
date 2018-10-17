#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  eai-ydlidar and serial module"
echo "start remove robot.rules from  /etc/udev/rules.d/"

sudo rm /etc/udev/rules.d/robot.rules
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish delete"
