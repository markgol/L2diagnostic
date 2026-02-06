This project is for learning about the unitree L2 Lidar hardware communication over ethernet using UDP.
This is a Qt Creator project.  This is just the source.  When you open this folder Qt Creator for the
first time Qt will not find the project file and will ask you to configaure a new project.  You should
select the project that matches your platform.  Qt will recognize this as a CmakeList project.

The 2 files unitree_lidar_protocols.h and unitree_lidar_utlities.h fall under a BSD 3 license.  See those
files for license.

The Qt distributable modues fall under the Qt LGPL license

For license text see the docs folder associated with this project

A standalone Windows app has been created.  It does not rely on the installation of Qt on the host
system.  It consists of a Zip download file.  This should be extracted to the directory were you want 
to save and run rthe app from.  It does not require an installer.  Just run it directly from its directory.
All dependencies are included.  Always verify your download against the file hash.  This is also
on the github repo; https://github.com/markgol/L2diagnostic

	Goals
	1.) Only use unitree_lidar_protocols.h and unitree_lidar_utlities.h files to in application
	2.) Complete open source to use the L2, no sourceless archive libraries
	3.) Diagnostic app to tell if the L2 is operating properly
			Display packet info stats
			Point cloud data viewer
	
	V0.3.10 release
		The project started with only a few files so the project folder structure was flat.
		This release refactors the overall project organization into a more standardized folder structure.
		Project
			/docs
			/forms
			/include
			/src
			CmakeList.txt
	V0.3.11 release
		project CMakeLists.txt file updates to include building install
		project CMakeLists.txt file for supoprt across Windows x64, Linux x64 and Linux ARM64
			This has been tested on Windows 11 x64, Ubuntu 24.04 x64 and Ubuntu 24.04 ARM64 on a RPI5.
		The RPI5 implementation does not support the point cloud viewer at this time.  That is planned for V0.4.0.
		(The RPI5 does not support OpenGL Core 3.3. It uses  OpenGL ES 3.0/3.1 which requires slightly different
		vertex shader and fragment shader)
		The processing of the raw L2 point cloud packet into a point cloud frame has been moved into the L2lidar class.
	V0.3.12
		Windows standalone app, L2diagnostics0-3-12.zip included
		Future versions will include the standalone app
		Moved render timer to PointCloudWindow class
		Changed app type in CMakeLists.xt for Window apps (Gui without console terminal)

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
		IMU orientation correction to point cloud position
		Set UDP configuration in L2 (requires restart of the L2 and restart of the app)
		Measurement of RTT latency (instanteous, average, minimum, maximu, variance)
		Correction of L2 timestamp to account for incorrect time clock on the L2
			(L2 time clock reports 1 second for every 2 real world seconds)
		Sync the L2 to the host time at a user settable rate (default 20Hz)
		Get and set L2 workmode
		Get L2 parameters
		Checked build and run status in Ubuntu 24.04.3 LTS and Windows 11 on x64 platform
		Default is to disable point cloud viewer until explicitly enabled.
			This is to allow use on RPI5 before OpenGL code updated for RPI5 Ubuntu OpenGL version
			No plan for support in RPI5 Debian since this is not Qt or ROS2 supported.
		
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
		Documentation for the PointCloudWindowClass
		Cleanup PointCloudWindow class (mostly just reorganizing)
		Add UART commmunications (delayed while researching UART packages)
		build a release package installer
		port for RPI5 build (RPI5 doesn't support OpenGL 3.3 Core, requires OpenGL ES 3.0/3.1)
			Currently functional on RPI5 if Point cloud viewer is disabled.  If you enable it it will cause
			segment fault.
		Integrate the L2lidar class into a ROS2 package
		
	Issues:
		L2 UART baudrate is 4M.  The QSerialPort has issues operating at this baud rate
		an alternative UART library must be used.  This issue makes it challenging to find
		a platform independent UART solution.  Solutions can be found for specific platforms.
		
	Using Qt 6.10.x
		The project has been developed using Qt Creator 18.0.x  Qt 6.10.x
		Using MSVC 2022, x86_64
		It has been tested for targets: Windows x64, Ubuntu x64 and ARM64
		There this a CmakeList.txt project.  I have tried to keep things compatible to other targets.
		If you are copying the project then copy it using the existing folder structure to whatever
		folder will be your project folder.
		When you open Qt Creator -> File -> Open file or project
				Select the CmakeList.txt file and select open
		Qt will ask you to select a configuration.
			Select Desktop Qt 6.10.x MSVC2022 64bit for Windows x64
				This selection will only show up if you have MSVC 2022 already installed
			Select Desktop QT6.10.x Clang18 for Ubuntu x64
			Select Desktop Qt6.10.x GCC for Ubuntu ARM64 on RPI5
				When operating on RPI5 with UBUNTU, Clangd may not be able to be used
				because of memory limitations on the RPI5.
			
		ctrl+b will build the project, f5 will run it.
	
		In QtCreator 18.x the follwing Build Step need to be added:
			Projects(left pane) ->Build Settings
			Add Build Steps -> Custom Process Step
				Command: cmd
				Arguments:/c installbuild.bat
				Working Directory:
			Make sure this comes after Build:cmake.exe step

	
	Separate build  files:
	
	Linux bash
	installbuild.sh
		#!/bin/bash
		set -e
		linuxdeployqt build/bin/L2diagnostic -appimage
		
	Windows
	installbuild.bat
		windeployqt build\Release\L2diagnostic.exe
		
	Other requirements for Linux
		sudo apt install fuse libfuse2
		wget https://github.com/probonopd/linuxdeployqt/releases/download/continuous/linuxdeployqt-continuous-x86_64.AppImage
		chmod +x linuxdeployqt-continuous-x86_64.AppImage
		sudo mv linuxdeployqt-continuous-x86_64.AppImage /usr/local/bin/linuxdeployqt
		