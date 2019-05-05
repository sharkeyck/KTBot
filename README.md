# KTBot

## Hardware Notes

### Battery Charging

2S 13A 7.4V Li-ion Lithium LiPo 18650 Battery Charger BMS Protection PCB Board

- Model: HX-2S-D20
- Charge voltage: 8.4v-9v
- Over discharge voltage range: 2.5-3.0v ±0.05v
- Overcharge voltage range: 4.25-4.35v±0.05v

## Running the Robot

`roslaunch ktbot stdr_server_ktbot.launch`

## ROS Details / References

Laser message format: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
Setup reference: https://github.com/baudren/NoteOrganiser/wiki/Docker,-QML,-XServer-on-Windows
ROS basics reference: http://wiki.ros.org/turtlesim

STDR simulator launcher

## Next steps

- Spend time exploring the bot

- Mechanical fixes
  - do auto port recognition
  - Switch to single microcontroller USB to odroid, do UART for everything else (less USB cables)
  - Install wireless charging stuff
  - Add depth camera for MOAR SENSINGS and battery charge localization. https://github.com/IntelRealSense/librealsense/blob/master/doc/RaspberryPi3.md
- Software
  - Re-setup auto ros start on bot
  - Try out Google Cartographer (instead of slam_gmapping) https://google-cartographer-ros.readthedocs.io/en/latest/
  - Clearing the map easily
  - Experiment with http://wiki.ros.org/smach for better path planning
- Driver fixes
  - PWM control of wheel speed in servo
- Gazebo
  - Create a servo simulator model plugin (LX-16A) that parses/interprets serial data and affects a rotating joint. Consider upstreaming it in gazebo_ros_pkgs.
  - Construct a .world file to launch the ktbot and register the lidar/servo plugins
  - Construct hackerhouse model using [this tutorial](http://gazebosim.org/tutorials?cat=build_world&tut=building_editor) and drive the KTbot around in it.
- Autonomy
  - Build charge station (switch-based wireless charger)
  - Setup low battery sensing (via servos?)
  - Return-to-base behavior when low on battery or objective complete

For V2
- Fix bot size & cable routing/management
- Gear wheels for higher speed
- ARBS
- Figure out why there's checksum failures at particular angle values (email support@lewansoul.com?)

## Packages

- gmapping
- navigation
- stdr_simulator
- neato_robot (https://github.com/mikeferguson/neato_robot)

## Python (pip) dependencies

- inject
- paho-mqtt
- msgpack
- pyserial

## Gazebo Woes

- Use ubuntu 16.04.1 (ver \*.3 uses nouveau drivers, which with my nvidia card doesn't work out at all.)
- Install ros-kinetic-desktop-full
- Install gazebo7 as the ros-gazebo package requires it
- URDF spec needed for running a ros robot

## Odroid
- Installed Ubuntu 16.04, added wifi to network interfaces, no wpa-config
- Adding zsh, git, vim
- installed ros-kinetic-ros-base
- added KTBot repo, missing the followig dependencies:
  - ros-kinetic-ros-control-boilerplate
  - ros-kinetic-joint-state-controller
  - ros-kinetic-robot-state-publisher
  - ros-kinetic-diff-drive-controller
  - ros-kinetic-rqt-common-plugins, ros-kinetic-rqt-robot-plugins, ros-kinetic-desktop-full
  - ros-kinetic-rosserial-python
  - davetcoleman/ros-control-boilerplate repo from Github
  - davetcoleman/gflags from Github NOPE, use: libgflags-dev
  - ros-kinetic-mqtt-bridge
  - python-pip: paho-mqtt, inject, msgpack, pyserial
  - ros-kinetic-ros-control, ros-kinetic-ros-controllers
  - ssh
  - ros-kinetic-xacro
- echo catkin_ws/devel/setup.zsh to zshrc
-changed hostname from odroid to ktbot- login as kt, /etc/hosts and /etc/hostname
-installed avahi daemon for mDNS things
-add kt user to dialout group

For audio:

- Add kt user to audio group `sudo gpasswd -a kt audio`
- install audio controls `sudo apt-get install alsa-utils`
- use `aplay` to play audio files!

For I2C

- Add kt user to i2c group `sudo gpasswd -a kt i2c`
- `sudo apt-get install i2c-tools
- `i2cdetect -y 1` to scan the i2c_1 channel
- Current state: I2C not working, probably a software problem as the oscilloscope is not detecting anything on the pins.


For Arduino

- Install platformio (`pip install -U platformio`)
- Use `platformio run -t upload` to send a sketch to the board.

For crashing/freezing:

- CPU governor should be set to "conservative", see https://forum.odroid.com/viewtopic.php?f=112&t=18461 
- Don't follow the instructions about making "conservative" like "ondemand" though. Or maybe do. Never tried it.
- Then again, might be a network problem. 
  - Follow [here](https://unix.stackexchange.com/questions/269661/how-to-turn-off-wireless-power-management-permanently) to set the powersave option.
  - Also https://forum.odroid.com/viewtopic.php?f=117&t=7803 might help
- Neither of these worked TBH... seems to be stable when sending keepalive pings (via script or putty config) though.

Can't connect to bot via "ktbot.lan":

- Go to http://pihole.lan/admin and login
- --> Settings --> DHCP
- Search on the page for "ktbot" to find the IP

Using network drive:

- https://codeyarns.com/2018/05/03/how-to-mount-remote-directory-on-windows-using-sshfs-win/
- This PC -> "Map network drive" -> "\\sshfs\kt@ktbot.lan" with "use alternate credentials"

### Debugging servos

1. roscd lx16a_driver && cd src
2. python example.py /dev/ttyUSB0 1
3. Servo ids are 0 or 1

### Debugging Gazebo

1. "BadValue" when trying to run? Downgrade nvidia drivers to 340.12

### Intel Realsense D435

Example lsusb: `Bus 009 Device 002: ID 8086:0b07 Intel Corp.`

Installation for realsense example libs:

* `git clone https://github.com/IntelRealSense/librealsense.git`
* `sudo apt-get install libglfw3-dev`
* `cmake . && make`

To reload USB 3 drivers if/when they crash:
```shell
echo -n "0000:02:00.0" | sudo tee /sys/bus/pci/drivers/xhci_hcd/unbind
echo -n "0000:02:00.0" | sudo tee /sys/bus/pci/drivers/xhci_hcd/bind
```

Installation for ros: follow instructions at https://github.com/intel-ros/realsense

## Wireless Charging

- Power connector to xu4q 5V, PWRON, GND, and UARTs: https://wiki.odroid.com/odroid-xu4/hardware/expansion_connectors
- 1x Wemos chip on board, powered from expansion connector on xu4q
- 2x (2S) Battery charge indicator boards: https://www.ebay.com/itm/Li-po-Battery-Indicator-Display-Board-Power-Storage-Monitor-For-Rc-BatteryPartsT/352150842337?hash=item51fdd36be1:m:mQ-TSkRYDCdGHHW-ml6iv7g
- 2x Battery fuel gauges: https://www.ebay.com/itm/2pcs-INA219-DC-Current-Sensor-Voltage-Test-Module-Breakout-Board-I2C-for-Arduino/292349818573
- Speaker shield for audio sounds: https://ameridroid.com/collections/odroid/products/stereo-boom-bonnet-amp-speakers

## "Purposes"

- Detect where shadow is most often
- Detect dropped socks (dropped electronics) and return them to home base
- Follow people around (reinvent it as a shelf)
- Carry messages to and fro
- Put a google home on it
- Teach it to play fetch, then to fetch things
- Telepresence

## Failed At:

* Installing Qemu for ROS Compilation that Doesn't Suck
  *  https://www.cnx-software.com/2012/03/08/how-to-build-qemu-system-arm-in-linux/
* Docker gazebo
  1. Install Docker
  2. Build container: `docker build -t my-ros-app .`
  3. Install XMing Windows System Server (Google for download link for your OS)
  4. Create the container (Make sure to change the IP address and username folder to your computer's / user's): `docker run -it --rm --name rostest -e DISPLAY=YOURIP:0.0 -v C:\Users\USERNAME\Documents\KTBot\catkin_ws:/root/catkin_ws my-ros-app:latest`
  *  `docker run -it -p 7681:7681 -p 8080:8080 --name gzweb giodegas/gzweb /bin/bash`

