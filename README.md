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
	Display packet counts for various packet type
	Display ACK packet results
	Display current L2 time stamp
	Display IMU accelerometer and gyro data
	Display converted point cloud packet, num valid data points in packet, first 2 x,y,z,i points
	Start, Stop, Reset and get Version control
	Point cloud viewer
	
	Still to do:
		cleanup of user GUI
		Add calibration and Lidar state data
		Optimize renderer to improve performance
		GUI for setting workmode parameters
