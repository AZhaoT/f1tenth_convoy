# convoy

This is a ROS package for controlling both simulated and real F1Tenth vehicles 
that will be used to test and evaluate convoy algorithms.

We will be using ROS2 for this project. For now, we will use the Foxy 
distribution. Make sure your workspace is called `cvy_ws`. To set up a 
workspace:

```bash
mkdir -p ~/cvy_ws/src/
cd ~/cvy_ws/src/
git clone --recurse-submodules git@github.com:RIVeR-Lab/convoy.git
git clone --recurse-submodules https://github.com/f1tenth/f1tenth_system.git
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/cvy_ws/
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
```

Follow the same instructions as from the build/setup instructions for the 
vehicles to assign static IP addresses for a computer/laptop. 
The vehicles will be numbered in order starting from 100, i.e., the first 
vehicle we set up will have IP address 192.168.1.100. Computers that are given 
static IP addresses will start at 200. So far, the RIVeR-Patuxent laptop is 
192.168.1.200 and the RIVeR-Patapsco desktop is 192.168.1.201.

## Bringing up the vehicle(s) on hardware

Make sure all of the below instructions and build instructions have been 
followed (except for the simulation stuff). Then, for example, if you are 
using vehicle 4 with a DS4 controller, run:

```bash
source-vehicle 4
ros2 launch convoy_ros rplidar_ds4.launch.py vehicle_number:=4
```

## Simulation

To launch the f1tenth simulator, first make sure you have followed all of the 
installation instructions at 
[`f1tenth_gym_ros`](https://www.github.com/f1tenth/f1tenth_gym_ros).
Make sure you properly install Docker (including post-installation steps), 
nvidia-docker2, and rocker.

It is recommended to configure some dot files to make coding in a terminal or 
a docker container easier. For example, to use Michael's dot files:

```bash
cd ~
git clone https://github.com/michael-shaham/dot-files.git
cd dot-files
source .bashrc
update-dot-files
```

To launch the simulator, we assume we have already set up a workspace called 
`cvy_ws` which contains the `convoy` package. 

```bash
cd ~/cvy_ws/src/convoy/
sudo docker build -t convoy -f Dockerfile .
sudo rocker --nvidia --x11 --volume .:/sim_ws/src/ --volume /home/$(echo $USER)/dot-files/:/root/ -- convoy
```

Note that the `dot-files` volume is optional. I use my dot-files in the 
container for better readability/code-editing in vim/tmux.

## Vehicle setup notes

If you are setting up a vehicle, make sure you closely follow the directions on 
the F1Tenth build site (for ROS2, don't use the ROS1 build directions). If the 
vehicle you are building is using the RPLiDAR S2 instead of the Hokuyo, do the 
following to set up the udev rules:

```
sudo vim /etc/udev/rules.d/99-rplidar-s2.rules
```

and then paste in the following:

```bash
KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="10c4", MODE="0666", GROUP="dialout", SYMLINK+="sensors/rplidar-s2"
```

You have to reboot for the changes to take effect. After installing ROS2, run 
the following to set up the workspace:

```bash
mkdir ~/cvy_ws/src && cd ~/cvy_ws/src
git clone git@github.com:RIVeR-Lab/convoy_ros.git
git clone git@github.com:Slamtec/sllidar_ros2.git
cd ~/cvy_ws/
colcon build --symlink-install
```

You only have to use `symlink-install` the first time you build the 
`sllidar_s2` package.

To launch the LiDAR with visualization to test it, make sure you have 
everything sourced properly, and then run:

```bash
ros2 launch sllidar_ros2 view_sllidar_s2_launch.py 
```

If you run into the issue of not being able to see the LiDAR output, make 
sure the user is in the dialout group, i.e.,

```bash
sudo adduser $USER dialout
```

and then log out and back in for the change to take effect. 

## Teleop information

`human_control/teleop` button: `L1`

`autonomous_control` button: `R1`

Note that these buttons act as a dead man's switch, i.e., if you let go of the 
button, the robot (should) stop. The left joystick is used for controlling the 
vehicle.

For teleop, the left joystick (up/down) controls the speed and the right 
joystick (left/right) controls the steering angle.

## Some useful USB commands

`lsusb` allows you to see connected USB devices. To figure out where a device 
is connected, run `lsusb` without the device plugged in, and then plug in the 
device and run again.

`usb-devices` lists all of the USB devices with more information including the 
vendor ID and product ID which are needed for creating symbolic links to the 
device.

`dmesg | less` and then hitting `/` and searching for keywords (like the 
product ID or vendor ID) can also be useful.

## Convenient dot files

Finally, Michael has some dot files that are nice for bash/vim/tmux. Do not use 
these if you are not comfortable with vim. Copy parts of the `.bashrc` to your 
own `.bashrc` as needed (like the sourcing or pulling shortcuts).

```bash
cd ~/Documents/
git clone git@github.com:michael-shaham/dot-files.git
cd dot-files
source .bashrc
update-dot-files
```

With this, we can then source the vehicle and correctly set the 
`ROS_DOMAIN_ID` using the command:

```
source-vehicle <vehicle_number>
```

where `vehicle_number` is the number we assign to the vehicle (i.e., if the 
vehicle is the 5th one set up and has IP 192.168.1.104, then replace 
`vehicle_number` with 4).

## Connect PS4 DS4 using Bluetooth via the command line

Find Bluetooth devices using the command

```bash
hcitool dev
```

This will return a device name and a MAC address. To find Bluetooth devices in 
range, run

```bash
hcitool -i <dev_name> scan
```

where `dev_name` is likely hci0. The Playstation 4 Dualshock 4 controller will 
be called `Wireless Controller`. Note the MAC address for this device, and 
then trust and connect to the device using the following commands:

```bash
bluetoothctl  # enters you into some terminal environment
trust <MAC_address>
connect <MAC_address>
```

## Map caps lock to esc/ctrl

Easier method is to use the Gnome Tweaks tool.

```
sudo apt install gnome-tweaks
```

Then hit the super (Windows) key and search for `Tweaks` and open it. Go to the 
`Keyboard & Mouse` section, hit `Additional Layout Options`, then 
`Caps Lock behavior` and select `Make Caps Lock an additional Esc`.

Another option is to use `keyd`, as described below. This is copied from the 
[keyd GitHub page](https://github.com/rvaiya/keyd).

```bash
git clone https://github.com/rvaiya/keyd
cd keyd
make && sudo make install
sudo systemctl enable keyd && sudo systemctl start keyd
sudo vim /etc/keyd/default.conf
```

Paste

```
[ids]

*

[main]

# Maps capslock to escape when (double?) pressed and control when held.
capslock = overload(control, esc)
```

into the `default.conf` file. Then run

```sudo systemctl restart keyd```

one more time.
