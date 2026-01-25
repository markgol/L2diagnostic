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
	Point cloud viewer with mouse controls for pan, zoom, rotate
	Added dockable windows for Calibration/state, IMU packet, ACK packet, packet stats
	Made packet rate chart dockable window
	Control buttons are now dockable window
	3D and 2Dpoint cloud viewing
	Workmode controls implemented
	L2Lidar class works across multiple UDP datagrams for large packets
	PointCloudWindow class useable in other apps
	
	Portable source class definition "L2lidar" using only unitree_lidar_protocols.h and
	unitree_lidar_utilites.h to perform required function needs for using the L2  in the
	UDP mode without the use of the unitree archive library
	
	POINT CLOUD VIEWER WINDOW (can not be docked)
	The orientation of the display is assuming the the L2 base sitting parallel to the earth,
	like on a table.  So the x,y plane will be paraellel to the earth, z axis in distance
	vertical (perpendicular) to the earth.
	
	MOUSE CONTROLS FOR POINT CLOUD VIEWER
	left+right buttons+movement -> pans in x,y
	rightbutton+movement -> pans z (up/down z axis)
	leftbutton+movement -> orbits (yaw and pitch)
	shift+left+rightbuttons+movement -> resets view to default
	
	DOCKED WINDOWS
	The controls window (button to do things) is permanently docked
	All other windows are dockable and default to docked
	If you want to see the packet rate chart window in more detail you would undock
	and resize that window.  Once undocked the packet rate window can only
	docked again using the windows reset button.  All other windows can be 
	undocked and redocked.
	
	VISIBILITY OF WINDOWS
	Use the config dialog to control visibility of windows
	if you close a window you may need to go into the Config dialog, uncheck the window, click ok
	then use the Config dialog again and renable the windows cna click okay
	
	Still to do:
		Add UART commmunications
		Add IMU orientation to point cloud position calcuation
		build a release package installer
		test other target builds
		
	Using Qt 6.10.x
		The project has been devleoped using Qt Creator 18.0.1  Qt 6.10.1.
		Using MSVC 2022, x86_64
		I have not tested it for other targets.
		The this a CmakeList.txt project.  I have tried to keep things compatible to other targets.
		If you are copying the project then copy it using the existing folder structure to whatever
		folder will be your project folder.
		When you open Qt Creator -> File -> Open file or project
				Select the CmakeList.txt file and select open
		Qt will ask you to select a configuration. I selected Desktop Qt 6.10.1 MSVC2022 64bit
		This selection will only show up if you have MSVC 2022 already installed
		ctrl+b will build the project, f5 will run it.
