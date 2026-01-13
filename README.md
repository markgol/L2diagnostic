This project is for learning about the unitree L2 Lidar hardware communication over ethernet using UDP.
This is a Qt Creator project.  This is just the source.  When you open this folder Qt Creator for the
first time Qt will not find the project file and will ask you to configaure a new project.  You should
select the project that matches your platform.  Qt will recognize this as a CmakeList project.

The 2 files unitree_lidar_protocols.h and unitree_lidar_utlities.h fall under a BSD 3 license.  See those
files for license.

Goals
1.) Only use unitree_lidar_protocols.h and unitree_lidar_utlities.h files to in application
2.) Complete open source to use the L2, no sourceless archive libraries
3.) Diagnostic app to tell if the L2 is operating properly
		Display packet info stats
		Point cloud data viewer

Current Status
	UDP only.
	Display a packet rate chart
	Display time stamp, packet counts for various packet types
	Display ACK packet results
	Display IMU accelerometer, gyro, quaternion data
	Displau Calibration and inside state data
	Send Start, Stop, Reset and get Version control commands
	Point cloud viewer
	Added dockable windows for Calibration/state, IMU packet, ACK packet, packet stats
	
	Portable source class definition "L2lidar" using only unitree_lidar_protocols.h and
	unitree_lidar_utilites.h to perform required function needs for using the L2  in the
	UDP mode without the use of the unitree archive library
	
	Still to do:
		Finish cleanup of user GUI
		Optimize renderer to improve performance
		UI for setting workmode parameters
