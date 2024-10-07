# evologics_driver
This is ros driver for evologics USBL/modems.

## Installation:

### install goby dependency: [reference](https://goby.software/3.0/)
```sh
# To install release packages on Ubuntu
$ echo "deb http://packages.gobysoft.org/ubuntu/release/ `lsb_release -c -s`/" | sudo tee /etc/apt/sources.list.d/gobysoft_release.list
$ sudo apt-key adv --recv-key --keyserver keyserver.ubuntu.com 19478082E2F8D3FE
$ sudo apt update
# minimal
sudo apt install libgoby3-dev goby3-apps
# full
sudo apt install libgoby3-dev libgoby3-gui-dev goby3-apps goby3-gui goby3-doc goby3-test libgoby3-moos-dev goby3-moos
```

### build:
```sh
$ cd /Your_Workspace/src
# acomms_msgs pkg
$ git clone https://github.com/GSO-soslab/acomms_msgs
# evologics_ros pkg
$ git clone https://github.com/GSO-soslab/evologics_ros
$ cd evologics_ros
$ git submodule update --init --recursive
$ cd /Your_Workspace/src
$ colcon build --packages-select acomms_msgs evologics_ros
```

### Usage:
```sh
$ cd /Your_Workspace/
$ source ~/Your_Worksapce/install/setup.bash
$ ros2 launch evologics_ros start.launch.py
```

### basic introduction (todo...):
<pre>

src/modem.cpp:              A node the wraps around Goby dynamic buffer for queuing messages and MAC manager for Time Division Multiple Access (TDMA).
config/goby.yaml:           The configuration file for the modem node. The user should select the device driver (evologics/seatrac) they would like to use and define their dynamic buffer.
config/evologics.yaml:      Specific configuration settings for the evologics modem/usbl.


Publishers:
/rx                         All data received from the modem driver
/rx_bytearray               All data received from the modem driver in the bytearray form
usbl_data                   USBL data, check acomms_msgs/UsblData

Subscribers:
/tx                         All data the user wants to send
/tx_bytearray               All data in bytearray form the user wants to send

</pre>