# QUTAS Ground Control Station

## Pre-Setup

1. Router Setup
   - TPLink N750 and running DD-WRT (v24-sp2)
   - WAN Port
     - Subnet isolated network
	 - To connect to QUT/internet, must login to [IAS](ias-user.qut.edu.au/login)
   - LAN port connections
     1. GCS Computer
     2. Vicon Computer(AUX Ethernet Port)
     3. (Ethernet connected for student use)
     4. (Ethernet connected for student use)
   - WIFI
     - 2.4GHz Band:
       - SSID: `FlyFi-ARC-2.4`
	   - Password: `aerospace`
     - 5.8GHz Band:
       - SSID: `FlyFi-ARC-5.8`
       - Password: `aerospace`
   	- Static Routes / Reserved IPs:
      - `ros-workstation`: 192.168.1.100
      - `vicon-server`: 192.169.1.101

2. Install GCS with Xubuntu 16.xx
   - Name the GCS: `ros-workstation`
   - Create an admin user named: `quas`
   - Make sure to enable "Require password to log in"

3. Install VICON Server with Windows 10 (QUT OS image is fine)
   - Name the PC: `vicon-server`
   - Install VICON Tracker 3.xx
   - Configure VICON Connection:
     - `Control Panel > View Network Connections > Ethernet 2 (Intel(R) PRO/1000 GT Desktop Adapter) > Properties > Configure > Advanced` ([source](https://docs.vicon.com/display/Connect/Configuring+network+card+settings#Configuringnetworkcardsettings-Configureadvancedadaptersettings))
       - `Jumbo Packet`: 9014 Bytes
       - `Receive Buffer`: 2048
       - `Interrupt Moderation`: Disabled
       - `Interrupt Moderation Rate`: Off
       - `Receive Side Scaling` (if present): Enabled
       - `Receive Side Scaling Queues` (if present): (Maximum value)
     - `Control Panel > View Network Connections > Ethernet 2 (Intel(R) PRO/1000 GT Desktop Adapter) > Properties`
     - `This connection uses the following items`
       - [x] QoS Packet Scheduler
       - [x] Internet Protocol Version 4 (TCP/IPv4)
       - [x] Internet Protocol Version 6 (TCP/IPv6)
       - [ ] (Everything else)
     - `Internet Protocol Version 4 (TCP/IPv4) > Properties`
       - `IP Address`: 192.168.10.1
       - `Subnet mask`: 255.255.254.0
       - `Default Gateway`: (Blank)
       - `Preferred DNS server`: (Blank)
       - `Alternate DNS server`: (Blank)

4. Telementry Setup
   - RFD900 Pair (running [SiK 2.0 Firmware](https://github.com/LorenzMeier/SiK))
   - Paremeters:
     1. `SERIAL_SPEED`: 115
     2. `AIR_SPEED`: 250
     3. `NETID`: 25
     4. `TXPOWER`: 20
     5. `ECC`: 0
     6. `MAVLINK`: 1
     7. `OPPRESEND`: 0
     8. `MIN_FREQ`: 915000
     9. `MAX_FREQ`: 928000
     10. `NUM_CHANNELS`: 50
     11. `DUTY_CYCLE`: 100
     12. `LBT_RSSI`: 0
     13. `MANCHESTER`: 0
     14. `RTSCTS`: 0
     15. `MAX_WINDOW`: 131
   - Connections
     - GCS: USB Serial Converter
     - Airborne: Connect to Telemetry 1 on Autopilot

## GCS Setup

#### Explaination of Scripted Files
The QUTAS GCS is setup using the following files and scripts:
- `README.md`: This file
- `setup.sh`: Setup the backend packages, desktop environment, and share folders
- `setup_drv.sh`: Setup the ROS shared workspace
- `setup_user.sh`: Setup a student user account
- `rdeps.txt`: List of unwanted Ubuntu packages to be removed
- `deps.txt`: List of required Ubuntu packages to be installed
- `deps_ros.txt`: List of required Ubuntu ROS packages to be installed
- `catkin_drv_pkgs.txt` List of QUTAS ROS packages to install into the shared workspace
- `groups_admin.txt`: List of groups that the admin account should belong to
- `groups_user.txt`: List of groups that a user account should belong to
- `etc/chrony/chrony.conf`: Configuration for time syncronization to work with QUT Time
- `home/quas/`: Configurations for the standard desktop layout:
  - `Desktop`: Useful desktop shortcuts
  - `.config`: Nice layout for GCS use in XFCE
- `usr/share/ros-workstation`:
  - `copy-config2user.sh`: Clones the admin desktop layout to a user account
  - `setup-catkin-workspace.sh`: Sets up a user accounts catkin workspace to use the shared workspace as well
  - `wifi-client-checker.py`: Small app to contact the router and displays number of WiFi clients connected (used with `genmon` for desktop display)
  - `fixHostname.sh`: Utility to reconnect ethernet to router to ensure hostname is dynamically set
  - `bash-additions.txt`: Additions for `.bashrc` to setup user accounts for the environment
  - `GroundControl.desktop`: Shortcut to run QGroundControl
  - `QUT Internet Login.desktop`: Shortcut to open the [IAS login page](ias-user.qut.edu.au/login)
  - `ros-logo.png`: Quirky logo to use as the "start menu" button
  - `qut-logo.png`: QUT logo to use for the IAS login shortcut

#### Base Setup
1. Make sure the OS is up to date, and install `git`:
```sh
sudo apt update
sudo apt upgrade
sudo apt install git
```
2. [Install ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu)
3. Download the administration scripts:
```sh
mkdir -p ~/Workspace/
git clone https://github.com/qutas/info
```

#### Configure the QUTAS GCS
Install the backend software:
```sh
cd ~/Workspace/info/Admin
./setup.sh
```

#### Configure the Shared ROS Workspace
Before continiuing, ensure that your bash environment is setup correctly, and that you can run ROS as follows:
```sh
roscore
```

Setting up the shared workspace:
```sh
cd ~/Workspace/info/Admin
./setup_drv.sh
echo "source /usr/share/ros-workspace/catkin_drv/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

At this point, you should be able to use `roscd` and `roslaunch` with the packages that have been installed into the `catkin_drv` workspace.

#### Configuring Student User Accounts
The following command will setup user accounts for students (without admin/sudo access). In the command, replace `USERNAME` with the desired username for the account (the password is set to `USERNAME` by default`):
```sh
cd ~/Workspace/info/Admin
./setup_user.sh USERNAME
```

To remove a user account from the system:
```sh
sudo userdel USERNAME
sudo rm -rf /home/USERNAME
```

#### Setting up QGroundControl
To set up QgroundControl, simply download the [Linux "Compressed Archive" ](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html#ubuntu-linux), and extract it into the `/usr/share/ros-workstation` folder. After that, you should be able to use the QGroundControl launcher (included in the `/usr/share/ros-workstation` setup) to start QGroundControl. If there are read permission errors, you may have to run the following command again to allow read access to the installation location:
```
chmod -R a+r /usr/share/ros-workstation
```





