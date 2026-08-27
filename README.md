**Updated: 2026-08-26**
This project is for learning about the unitree L2 Lidar hardware communication over ethernet using UDP.

This is a Qt Creator project.  If you are just using the just the source.  When you open the CMakkeList.txt file in this this folder in Qt Creator.  The first time Qt will not find the its .qtcreator project folder and  it will ask you to configure a new project.  You should select the project that matches your platform.

Qt will recognize this as a CmakeList project.

The 2 files unitree_lidar_protocols.h and unitree_lidar_utlities.h fall under a BSD 3 license.  See those
files for license.

The Qt distributable modues fall under the Qt LGPL license

For license text see the docs folder associated with this project

A standalone Windows app has been created.  It does not rely on the installation of Qt on the host
system.  It consists of a Zip download file.  This should be extracted to the directory were you want 
to save and run the app from.  It does not require an installer.  Just run it directly from its directory.
All dependencies are included.  **Always verify your download against the file hash.**

This is also on the github repo; https://github.com/markgol/L2diagnostic

A standalone app has been generated for Windows x86_64.

Standalone (without Qt installed) apps for use on Ubuntu for NVidia Jetson Orin Jetpack 7.2 and Ubuntu 24.04 of RPIs, Windows WSL2 are being developed but are not ready for release.

**Goals**

1.) Only use unitree_lidar_protocols.h and unitree_lidar_utlities.h files from Unitree for the basis of the application.  Dp not use the Unitree examples or other source code.

2.) Open source for the L2 along with open source L2lidar class instead of Unitree archive library. This class has a ROS2 compatible architecture.

3.) A diagnostic app to tell if the L2 is operating properly

        Display packet, info, calibration and other stats
        Point cloud data viewer
        Allow override and adjustment of the calibration parameters in real time
        Generate a range correction LUT to compensate for the non-linear range response of the L2

4.) A L2lidar class open source implementation that can correct some of the issues observed with the L2 such as keeping the L2 time base constantly synced to the host.  This helps resolved the problem with the L2 timebase running ~1/2 time (one second on L2 is ~2 seconds in real world).  Allow the user to override the built-in calibration parameters.  On some L2 units there is a significant difference between the optimal value and the built-in values.



**Hardware used in development**

Unitree 4D LiDAR L2

    Hardware version: 2.2.1.1

    Firmware version: 2.8.11.1

    Build/Complile date: 2025-07-15 

Sager NP8876D (Clevo PD70SND) running Windows 11 with WSL2 Ubuntu 24.04

Raspberry PI 5 8GB running Ubuntu 24.04

Jetson Orin Super Nano running Jetpack 7.2 (Ubuntu 24.04)

1G Ethernet backbone



#### **CURRENT RELEASE**

**V2.0.1 release**

The is the initial V2.x release.
It added supoprt for overriding the fast scan elevation angle step size.
This release also changes which version of Qt for source builds is being used:

- QtCreator 20.0.x

- Qt 6.11.x

#### **RELEASE HISTORY**

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



**V1.0.0**

Full Release.

Added stats for the IMU

Updated Config UI with minor changes to limit and defaults

No further development is planned.  Updates will consist only of bug corrections.

Changed time units from double to long long to avoid truncation of Epoch time in nanoseconds



**V1.1.0**

Added override of built in calibration for range; Range Scale and Range Bias

If you see your point cloud has barrel or pincushion distortion you adjust Range Bias.  The more positive the more barrel distortion.  The more negative the more pincushion distortion.  When set properly there will be minimum distortion.

**V1.2.0**

Updated to L2lidarClass V1.3.0
Added point cloud save to have to file format options; save with timestamp and save without timestamp.  This was done to be more compatible with CloudCompare.

**V1.2.1**

Added settable time matching constraint when performing IMU pose (no translation) adjustment to point cloud data.  This is located in the config dialog

Updated to L2lidarClass V1.3.2

**V1.2.2**

Updated to L2lidarClass V1.3.3
Corrected bug in L2lidarClass that was incorrectly calculating timestamp correction when the enable timestamp correction was enabled

**V1.3.0**

Updated to L2lidarClass V1.3.4
The L2lidarClass changes address various timestamp correction configurations and initialization scenarios.  It also adds a Use system Now time for packet timestamps to allow emulation of what the L2 archive library does.  Checks for valid scalar to use in timestamp correction. Added more quaternion and euler methods to assist in various conversions.

Additional stats pane added to the IMU panel.  These shows stats for derived roll, pitch and yaw from the IMU quaternion and roll, pitch from the IMU accelleration data.

Added addiotnal controls in the config dialog to allow setting the timestamp scalar using a numerator and denominator.  These should be 6 significant digits like 200000 for numerator, 100000 for denominator.  This allows the user to more precisely adjust the L2 timebase correction.  A flag for Use system now time for packet timestamps was also added.

Removed the timestamp correction conditional restrainst for aggregation.  Aggregration can now be used regardless of the timestamp correction setting.

Removed the appimage section of the CMakeLists.txt file.  This used the linuxdeploy app.  The linuxdeplot and linuxdeployqt apps are not maintained and actually compatible with QT 6.10 or later Qt versions.  This only applied to Linux build with appimage build selected.
Added RPATH settings in CMakeLists.txt so that the ./lib folder is searched before system folders.

**V1.3.1**

This is a small update.  It initializes the IMU dock window to a correct minimum size.  It copies the current view settings to the default view settings in the config dialog.  Also in the config dialog the L2 UDP subnet mask and gateway can be changed from the default.  A standalone windows executable has been created and does not require Qt to be installed.  The aarch64 executable (RPI5 and Jetson Orin) has been created but does require Qt to be installed.

**V1.3.2**

This adds flattening a slow anglular scan range into a a single flat scan.
Added specifying a angular scan range to collect for the 3d point cloud.
Updated the range value saved in the PCD file to be the calibrated range value instead of the raw range value returned by the L2.



**V2.0.0 release candidate 1**

This is a significant upgrade to the L2diagnostic application.

The primary purpose was to split operation into 2 modes; diagnostics and calibration

The calibration mode allows the user to adjust override calibration parameters while observing realtime point clouds.

It also allows the collection of data needed to create a nonlinear range correction using a piecewise cubic spline. It generates a calibration file that is compatible for use with the L2lidar class so that it can be used on ROS2 and other applications. A separate writeup is being generated with the calibration process.

For source build. This release use Eigen V3.4.1 Template source library. This library is not compatible with github. It can be found at: https://libeigen.gitlab.io/

It must be extracted to the include/third_party folder as a subfolder named Eigen.

## **Current Status**

UDP only. (serial workmode currently planned)

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

    (L2 time clock reports 1 second for every ~2 real world seconds)

Sync the L2 to the host time at a user settable rate

Get and set L2 workmode

Get L2 parameters

Allow use on system without GPU support (disables the graphics windows)

Checked build and run status in Ubuntu 24.04 LTS on Windows 11 on x64 platform

Checked build and run status in Ubuntu 24.04 LTS on RPI platform

Checked build and run status in Ubuntu 24.04 LTS and Windows 11 on x64 platform

No plan for support in RPI with Raspberian.

ROS2 support completed through separate l2lidar_node app

see:[GitHub - markgol/l2lidar_node: ROS2 publisher node for the Unitree L2 lidar point cloud and IMU data · GitHub](https://github.com/markgol/l2lidar_node)

Portable source class definition "L2lidar" using only modified unitree_lidar_protocols.h and unitree_lidar_utilites.h to perform required function needs for using the L2  in the UDP mode without the use of the unitree archive library.

POINT CLOUD VIEWER WINDOW (can not be docked)

The orientation of the display is assuming the the L2 base sitting parallel to the earth, like on a table.  So the x,y plane will be paraellel to the earth, z axis in distance vertical (perpendicular) to the earth.  If point cloud correction using IMU pose is enable then the display will show correct orientation to earth.

This is only a diagnsotic app and does not have odometry input required for motion correction other than orientation (yaw, pitch,roll).

Override of builtin calibration for Range Bias and Range Scale plus enable added

Allows for various timestamp correction configurations including substituting system timestamp for packet timestamps

Added derived statistics for Roll, Pitch and Yaw using quaternion IMU data.

Added derived statisitcs for Roll and Pitch using gravity aligned acceleration IMU data.



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



**Issues:**

This supports only UPD Ethernet interface. No plans going forward to support the UART interface.
On some system the lettering in the dialog boxes can be obscured because of the resolution settings for the display.



**Using Qt 6.11.x**

The project has been developed using Qt Creator 20.0.x  Qt 6.11.x

Using MSVC 2022 x86_64 and gcc 64

It has been tested for targets: Windows x64, Ubuntu x64 and ARM64

There this a CmakeList.txt project.  I have tried to keep things compatible to other targets.



For source build. This release use Eigen V3.4.1 Template source library. This library is not compatible with github. It can be found at: https://libeigen.gitlab.io/
It must be extracted to the include/third_party folder as a subfolder named Eigen.



If you are copying the project then copy it using the existing folder structure to whatever folder will be your project folder.

When you open Qt Creator -> File -> Open file or project

    Select the CmakeList.txt file and select open

Qt will ask you to select a configuration.

    Select Desktop Qt 6.11.x MSVC2022 64bit for Windows x64

        This selection will only show up if you have MSVC 2022 already installed

            Select Desktop QT6.11.x Clang for Ubuntu x64

            Select Desktop Qt6.11.x GCC for Ubuntu ARM64 on RPI5

When operating on RPI5 with Ubuntu, Clangd may not be able to be used because of memory limitations on certain RPIs.

ctrl+b will build the project, f5 will run it.



**WINDOWS RELEASE BUILD**

The windows standalone release consists of .zip file.

The contents are extratced into a single folder.  If you run the application from the folder it will find everything it needs  without needing to be installed.

If you want to build your own release:

    The following is done after the release build has completed in Qt.

    It can be run from QtCreator terminal:

        You may need to modify this batch file to match your exact Qt installation.

        buildInstall4Windows.bat

        
