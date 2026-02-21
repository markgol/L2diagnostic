**Updated: 2026-02-21**
This project is for learning about the unitree L2 Lidar hardware communication over ethernet using UDP.

This is a Qt Creator project.  If you are just using the just the source.  When you open the CMakkeList.txt file in this this folder in Qt Creator.  The first time Qt will not find the its .qtcreator project folder and  it will ask you to configure a new project.  You should select the project that matches your platform.

Qt will recognize this as a CmakeList project.

The 2 files unitree_lidar_protocols.h and unitree_lidar_utlities.h fall under a BSD 3 license.  See those
files for license.

The Qt distributable modues fall under the Qt LGPL license

For license text see the docs folder associated with this project

A standalone Windows app has been created.  It does not rely on the installation of Qt on the host
system.  It consists of a Zip download file.  This should be extracted to the directory were you want 
to save and run rthe app from.  It does not require an installer.  Just run it directly from its directory.
All dependencies are included.  Always verify your download against the file hash.

This is also on the github repo; https://github.com/markgol/L2diagnostic



**Goals**

1.) Only use unitree_lidar_protocols.h and unitree_lidar_utlities.h files to in application

2.) Open source for the L2 along with open source L2lidar class instead of Unitree archive library. This class has a ROS2 compatible architecture.

3.) A diagnostic app to tell if the L2 is operating properly

    Display packet, info, calibration and other stats

    Point cloud data viewer

4.) A L2lidar class open source implementation that can correct some of the issues observed with the L2 such as keeping the L2 time base constantly synced to the host.  This helps resolved the problem with the L2 timebase running 1/2 time (one second on L2 is 2 seconds in real world).



**V0.3.10 release**

The project started with only a few files so the project folder structure was flat.

This release refactors the overall project organization into a more standardized folder structure.

    Project

        /docs

        /forms

        /include

        /src

            CmakeList.txt



**V0.3.11 release**

project CMakeLists.txt file updates to include building install

project CMakeLists.txt file for supoprt across Windows x64, Linux x64 and Linux ARM64

This has been tested on Windows 11 x64, Ubuntu 24.04 x64 and Ubuntu 24.04 ARM64 on a RPI5.

The RPI5 implementation does not support the point cloud viewer at this time.  That is planned for V0.4.0.

(The RPI5 does not support OpenGL Core 3.3. It uses  OpenGL ES 3.0/3.1 which requires slightly different vertex shader and fragment shader)

The processing of the raw L2 point cloud packet into a point cloud frame has been moved into the L2lidar class.



**V0.3.12**

Windows standalone app, L2diagnostics0-3-12.zip included

Future versions will include the standalone app

Moved render timer to PointCloudWindow class

Changed app type in CMakeLists.xt for Window apps (Gui without console terminal)



**V0.4.0**

Implementation for both OpenGLES 3.x (RPI5 Ubuntu) and OpenGl Core 3.3 (Windows & Ubuntu) has been completed. Operation on RPI5 with point rendered operating withe loss without skipping any point cloud packets verified.

Port for ARM64 and x64 completed.

1 standalone app packages for: Windows 11 x64

Running on RPI5 with Ubuntu 24.04 , and Ubuntu 24.04 x64

To run on OpeGLES system like the RPI5.  You must start the app with a command line argument like:

    L2diagnostic OpenGLES

If you don't you it will operate properly and likely crash.



**V0.4.1**

Set MAC address added

More code cleanup

Removed debug output

Decode all known packets (including 2 that have no been onserved)

Added send to start packet receive (Qt needed a send UDP before it could start receiving packets)

Added code to stop QPA theme warnings

Allow disable of graphics display which allows operation on system without GPU support, use command line:

    L2diagnsotics No Graphics



**V0.4.2**

Added error messaging for UDP connection and send errors

Bug fix, L2L2lidar::connectL2()  was setting flag later than when it was suppoded to be set.

Added binary release for Ubuntu x86_64 and aarch64.  Note: These are executables only

and are not .appimage files.  They require Qt 6.10.2 to be installed before they will work.

.AppImage version are being worked on but not yet included.



**V0.4.3**

Updated L2lidar class error handling,  removal of all gui and ui interaction and added range to point cloud data.  Even though range can be recalculated from x,y,z data it is already availabe directly in the point data from the L2.  This allows downstream usage without have to recalculate each time.  

Updated conversion from L2 packet to point cloud to include range, and time stamping of each cloud point.  Added more stats to L2lidar class to assist in verifucation of a ROS2 publisher node.

Sycnhronized usage of the L2lidar calss between L2diagnostic app and l2lidar_ros2 publisher  node app.

Aggregation mode added to verify techniquue to be used in the l2lidar_ros2 publishenode so that it is compatible with LIO-SAM methodology.

Separated the L2lidar class into it own folder structure to support separate repo for it.



**Current Status**

UDP only. (serial workmode planned for later release)

Display a packet rate and packet rate chart history

Display time stamp, packet counts for various packet types

Display ACK packet results

Display IMU accelerometer, gyro, quaternion data

Display Calibration and inside state data

Send Start, Stop, Reset and get Version control commands

Point cloud viewer with mouse controls for pan, zoom, rotate

Added dockable windows for Calibration/state, IMU packet, ACK packet, packet stats

Made packet rate chart dockable window

Simple button selection to perform functions

3D and 2Dpoint cloud viewing

L2Lidar class works across multiple UDP datagrams for large 2D packets

PointCloudWindow class useable in other apps

IMU orientation correction for L2 pose applied to point cloud data (optional)

Set UDP configuration in L2 (requires restart of the L2 and restart of the app)

Set MAC address of L2 (requires restart of the L2 and restart of the app)

Measurement of RTT latency (instanteous, average, minimum, maximu, variance)

Correction of L2 timestamp to account for incorrect time clock on the L2

    (L2 time clock reports 1 second for every 2 real world seconds)

Sync the L2 to the host time at a user settable rate (default 30 msec)

Get and set L2 workmode

Get L2 parameters

Allow use on system without GPU support (disables the grpahics windows)

Checked build and run status in Ubuntu 24.04.3 LTS and Windows 11 on x64 platform

Checked build and run status in Ubuntu 24.04.3 LTS and Windows 11 on x64 platform

Checked build and run status in Ubuntu 24.04.3 LTS and Windows 11 on x64 platform

No plan for support in RPI5 Debian since this is not Qt and ROS2 supported.

ROS2 support completed through the l2lidar_ros2 app

    see: https://github.com/markgol/l2lidar_ros2

Portable source class definition "L2lidar" using only unitree_lidar_protocols.h and unitree_lidar_utilites.h to perform required function needs for using the L2  in the UDP mode without the use of the unitree archive library.

POINT CLOUD VIEWER WINDOW (can not be docked)

The orientation of the display is assuming the the L2 base sitting parallel to the earth, like on a table.  So the x,y plane will be paraellel to the earth, z axis in distance vertical (perpendicular) to the earth.  If point cloud correction using IMU pose is enable then the display will show correct orientation to earth.

This is only a diagnsotic app and does not have odometry input required for motion correction other than orientation (yaw, pitch,roll).

**MOUSE CONTROLS FOR POINT CLOUD VIEWER**

left+right buttons+movement -> pans in x,y

rightbutton+movement -> pans z (up/down z axis)

leftbutton+movement -> orbits (yaw and pitch)

shift+left+rightbuttons+movement -> resets view to default



DOCKED WINDOWS

The controls window (button to do things) is permanently docked.

All other windows are dockable and default to docked.

If you want to see the packet rate chart window in more detail you would undock and resize that window.  Once undocked the packet rate window can only docked again using the windows reset button.  All other dockable windows can be  undocked and redocked.

The point cloud window is not a dockable window.



VISIBILITY OF WINDOWS

Use the config dialog to control visibility of windows

if you close a window you may need to go into the Config dialog, uncheck the window, click ok then use the Config dialog again and renable the windows can click okay



**Still to do:**

Complete standalone impementation for Ubuntu 24.04 x64 and Ubuntu 24.04 ARM64 (RPI5)

Documentation for the PointCloudWindowClass

Cleanup PointCloudWindow class (mostly just reorganizing)

Add UART commmunications (delayed while researching UART packages)



**Issues:**

L2 UART baudrate is 4M.  The QSerialPort has issues operating at this baud rate an alternative UART library must be used.  This issue makes it challenging to find a platform independent UART solution.  Solutions can be found for specific platforms.



**Using Qt 6.10.x**

The project has been developed using Qt Creator 18.0.x  Qt 6.10.x

Using MSVC 2022 x86_64 and gcc 64

It has been tested for targets: Windows x64, Ubuntu x64 and ARM64

There this a CmakeList.txt project.  I have tried to keep things compatible to other targets.

If you are copying the project then copy it using the existing folder structure to whatever folder will be your project folder.

When you open Qt Creator -> File -> Open file or project

    Select the CmakeList.txt file and select open

Qt will ask you to select a configuration.

    Select Desktop Qt 6.10.x MSVC2022 64bit for Windows x64

        This selection will only show up if you have MSVC 2022 already installed

            Select Desktop QT6.10.x Clang18 for Ubuntu x64

            Select Desktop Qt6.10.x GCC for Ubuntu ARM64 on RPI5

When operating on RPI5 with Ubuntu, Clangd may not be able to be used because of memory limitations on that specific RPI5.

ctrl+b will build the project, f5 will run it.



**WINDOWS RELEASE BUILD**

The windows standalone release consists of .zip file.

The contents are extratced into a single folder.  If you run the application from the folder it will find everything it needs  without needing to be installed.

If you want to build your own release:

    The following is done after the release build has completed in Qt.

    It can be run from QtCreator terminal:

        You may need to modify this batch file to match your exact Qt installation.

        buildInstall4Windows.bat

        
