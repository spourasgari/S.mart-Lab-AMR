# Basic Instructions

## How to SSH into RPi

Device Name: `amr-rpi4`
Static IP: `192.168.137.64`
Admin Password: `Responsible2024`

Use -XC parameter to connect with graphics, but in a compressed way.

## WiFi connection settings

Without the interface, the RPi automatically connects to the following SSIDs with the same order of priority:

- _Sina_ with the password `sinapsinap`
- _TP-Link..._ with the password `111` (available in the platform, apparently without internet access)

The settings are in this directory: `/etc/netplan`. Last and previous config files are in the Others directory on this repository, named _50-cloud-init.yaml_ and _50-cloud-init.yaml.save_ respectively.

## Find serial connection (Arduino-RPi) port

Using this command `ls /dev/tty*`, the connected port to communicate data with Arduino will be revealed. It is used in the codes of ROS nodes.


## Communicating directly with Arduino

While it's not needed anymore (the robot can be controlled remotely using _teleop_ node), the direct communication with the Arduino can be done using the following command:
`screen /dev/ttyACM0 9600` and to quit: _Ctrl+A, then K, and then y_.

## Building ROS2 Packages

It's recommended to use ```colcon build --packages-up-to package-name``` at least for the first builds, to make sure all dependencies are also built.

## TODO Electronics

### Add a fuse

Between the connection to the motor and the boards, to prevent over-current damage the board and burn the amplifier IC!
