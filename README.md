# Sick_Safetyscanners ROS Driver

## Table of contents

- [Supported Hardware](#supported-hardware)
- [Getting started](#getting-started)
- [ROS API](#ros-api)
- [Creators](#creators)

A ROS Driver which reads the raw data from the SICK Safetyscanners and publishes the data as a laser_scan msg.

## Supported Hardware

| Device Name  | Type | Description |
| ------------- | ------------- | ------------- |
| microScan3 Pro | MICS3-CBAZ55ZA1  | Safety Laserscanner,  EFIPRO, Protective field range 5.5m, Scanning angle 275°  |

## Getting started

The ROS driver will be released as a debian package, and therefore can be installed from binaries or from source.

### Prerequisites

* Linux
* Working ROS-Distro
* Correctly setup SickSafetyscanner
* Connected SickSafetyscanners and a correctly setup ethernet network. Both the host and the sensor have to be in the same network.

### Installation

In the following instructions, replace `<rosdistro>` with the name of your ROS distro (e.g., `kinetic`).

#### From Binaries

The driver is released at longer intervals as a binary package. 

`sudo apt-get install ros-<rosdistro>-sick_safetyscanners`

#### From Source

```bash
source /opt/ros/<rosdistro>/setup.bash
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/src/
git clone https://github.com/SICKAG/sick_safetyscanners.git
cd ..
catkin_make install
source ~/catkin_ws/install/setup.bash
```

### Starting

To start the driver the launch file has to be started. For the driver to work correctly, the sensor ip, sensor tcp port, host ip and host udp port have to be defined. These parameters can be passed to the sensor as arguments via launch file.

```
roslaunch sick_safetyscanners sick_safetyscanners.launch sensor_ip:=192.168.1.10 sensor_tcp_port:=2122 host_ip:=192.168.1.9 host_udp_port:=6060
```

This will start the driver and the dynamic reconfigure node. In this you can set different parameters on runtime, especially the angles and the data the sensor should publish. If these parameters should be set on startup they can be loaded to the parameter server beforehand.

To visualize the data start rviz and subscribe to the /sick_safetyscanner/laser_scan topic.

```
rosrun rviz rviz 
```

### Troubleshooting

* Check if the sensor has power and is connected to the host.
* Check if booth sensor and host are in the same subnet e.g. 192.168.1
* Check if the launch file is called with the correct parameters for IP-addresses and ports.

## ROS API



### Advertised ROS Topics


`
~/laser_scan (type: sensor_msgs/LaserScan)
`

Publishes a scan from the laserscanner

`
~/extended_laser_scan (type sick_safetyscanners/ExtendedLaserScanMsg)
`

Extends the basic laser scan message by reflektor data and intrusion data.


`
~/raw_data (type: sick_safetyscanners/RawMicroScanDataMsg)
`

Publishes the raw data from the sensor as a ros message.


### ROS parameters

| Parameter Name        | Type | Required on startup | Information |
| -------------                   |  ------------- | ------------| ------------- |
| sensor_ip                    | String  | ✔ |Sensor IP address. Can be passed as an argument to the launch file. |
| sensor_tcp_port          | Integer | ✔ | Sensor TCP Port.  Can be passed as an argument to the launch file. |
| host_ip                        |   String   | ✔ | IHost IP address.  Can be passed as an argument to the launch file.  |
| host_udp_port             | Integer | ✔ | Host UDP Port.  Can be passed as an argument to the launch file.  |
| laser_scan_frame_name  | String | | The frame name of the sensor message  |
| publish_frequency    | Integer | | How many scans should be publised (1 means every scan, 2 every second scan, ... ) |
| angle_start              | Double | | Start angle of scan, if both start and end angle are set to 0, all angels are regarded  |
| angle_end                | Double | | End angle of scan, if both start and end angle are set to 0, all angels are regarded  |
| channel_enabled     | Boolean | | If the channel should be enabled  |
| general_system_state  | Boolean | | If the general system state should be published  |
| derived_settings      | Boolean | | If the derived settings should be published  |
| measurement_data  | Boolean | | If the measurement data should be published  |
| intrusion_data          | Boolean | | If the intrusion data should be published  |
| application_io_data  | Boolean | | If the application IO data should be published  |

## Creators

**Lennart Puck** 
FZI Forschungszentrum Informatik


- <http://www.fzi.de>

on behalf of SICK AG 

- <http://www.sick.com>




