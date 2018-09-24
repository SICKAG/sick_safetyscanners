# Sick MicroScan3 ROS Driver

A ROS Driver which reads the raw data from the MicroScan3 and publishes the data as a laser_scan msg.

## Getting started

To get the ROS driver ready, one has to include the package in the project workspace.

### Prerequisites

* Linux
* Working ROS-Distro
* A catkin_ws
* Correctly setup MicroScan3
* Connected MicroScan3 and a correctly setup ethernet network

### Installing

Include the sick_microscan3_ros_driver package in the catkin workspace.
Run catkin-make in the catkin_ws to build the driver.
This should create all necessary binaries to start the driver.

### Starting

To start the driver adapt the launch file in the folder catkin_ws/src/sick_microscan3_ros_driver/launch with the correct ros parameters.
Most importantly the sensor IP, Sensor TCP port and the Host UDP port have to be defined, else the default values will be used which might lead to no connection being established.
For all ROS Parameters see Section ROS Parameters.
Once the launch file is adapted run:

'''
roslaunch sick_microscan3_ros_driver sick_microscan3_ros_driver.launch 
'''

This will start the driver and the dynamic reconfigure node. In this you can set different parameters on runtime, especially the angles and the data the sensor should publish.

To visualize the data start rviz

'''
rosrun rviz rviz -d <arg> 
'''
Where arg is the path to the rviz configuration.
In the project there is a default configuration, under catkin_ws/src/sick_microscan3_ros_driver/config/rviz/default.rviz




## Data

### ROS Topics

Currently only a sensor_msgs/LaserScan.msg is published.

'''
/sick_microscan3_ros_driver/laser_scan
'''

In the future there will be an extended message with more data available, but this is still in progress.

### ROS parameters

| Parameter Name  | Type | Information |
| ------------- | ------------- | ------------- |
| sensor_ip | String  | Sensor IP Adress, only set at startup  |
| sensor_tcp_port  | String  | Sensor TCP Port, set at startup  |
| host_ip  | String  | IP of the host, which should receive the data  |
| host_udp_port  | Integer  | UDP Port on which the data is received on the host  |
| laser_scan_frame_name  | String  | The frame name of the sensor message  |
| channel  | Enumeration  | Which channel should be used and modified, currently only supported for channel 0 by the MicroScan3  |
| channel_enabled  | Boolean  | If the cahnnel should be enabled  |
| e_interface_type  | Enumeration  | Interfacetype which should be used, range from [0-3]. Corresponding [EFI-pro, EtherNet/IP, Profinet, Non-safe Ethernet]  |
| publish_frequency  | Integer  | How many Scan should be publised, 1 means every scan, 2 every second scan, and so on  |
| angle_start  | Double  | Start angle, if both start and end angle are set to 0, all angels are regarded  |
| angle_end  | Double  | End angle, if both start and end angle are set to 0, all angels are regarded  |
| general_system_state  | Boolean  | If the general system state should be published  |
| derived_settings  | Boolean  | If the derived settings should be published  |
| measurement_data  | Boolean  | If the measurement data should be published  |
| intrusion_data  | Boolean  | If the intrusion data should be published  |
| application_io_data  | Boolean  | If the application IO data should be published  |





