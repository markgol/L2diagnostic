//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: Main.cpp
//
//  Purpose:
//  Determine correct operation of the Unitreee L2 Lidar hardware
//  and software.  Establish platform independent software protocols
//  for using the L2 Lidar with its Ethernet interface.
//
//  Background:
//  Unitree provides undoucmented software files in the form:
//      include files
//      example application files
//      .a Archive Library
//
//  The source files rely on an Archive library using POSIX I/O
//  No source exists for the archive Library making it diffcult
//  to debug or port usage of the L2 Lidar for other platforms.
//  The hardware has 2 mutually exclusive communication interfaces:
//      Ethernet using UDP
//      Serial UART
//  The serial UART is limited in speed and does not operate at
//  the full sensor speed of 64K/sec sample points.
//
//  Solution:
//  This software skeleton was created using directed ChatGPT AI
//  conversation targetting a QT Creator development platform.
//  It reads UPD packets from the L2, caterorizes them, performs
//  error detection for bad packets (lost), display subsample
//  of packet.
//
//  V0.1.0  2025-12-27  compilable skeleton created by ChatGPT
//  V0.4.0  2026-02-06  Added initialization of OpenGL
//                      If no cmd line arguments are present then
//                          OpenGL Core 3.3 is used.
//                      if one command line argument is present then
//                          OpenGLES V3.0 is used
//                      if 2 or more command lne argumented then
//                          do not display graphics
//  V0.4.1  2026-02-11  Disabled QPA THEME GNOME
//  V1.0.0  2026-03-28  Offical release
//  V1.1.0  2026-04-20  Added override of range calibration params
//  V1.1.1  2026-04-29  Added CloudCompare PCD compatible output file
//                      with just x,y,z,intensity,range
//  V1.2.0  2026-05-12  Updated L2lidarClass for more precise timestamp calculations
//
//--------------------------------------------------------

//--------------------------------------------------------
// This app uses the following Unitree L2 sources modules:
//      unitree_lidar_protocol.h
//      unitree_lidar_utilities.h
// They have been modifed from the original sources
// to correct for errors, missing definitions and
// inconsistencies.  These have been minor in most
// instances. These are what are acutally being used:
//      unitree_lidar_protocolL2.h
//      unitree_lidar_utilitiesL2.h
//
// Copyright (c) 2024, Unitree Robotics
// The orignal source can be found at:
//      https://github.com/unitreerobotics/unilidar_sdk2
//      under License: BSD 3-Clause License (see files)
//
//--------------------------------------------------------

//--------------------------------------------------------
// GPL-3.0 license
//
// This file is part of L2diagnsotic.
//
// L2diagnsotic is free software : you can redistribute it and /or modify it under
// the terms of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// L2diagnsotic is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with L2diagnsotic.
// If not, see < https://www.gnu.org/licenses/>.
//--------------------------------------------------------

//--------------------------------------------------------
//  Standard application entry point
//  Generic main
//  Application is in MainWindow Class
//--------------------------------------------------------
#include <QApplication>
#include <QLoggingCategory>
#include "MainWindow.h"

int main(int argc, char* argv[])
{
    QSurfaceFormat fmt;

    // When OpenGL is used it needs to have
    // the surface initialized
    // This MUST be before QApplication
    //
    //  argc = 1 no command line argmuments use OpenGL Core 3.3
    //  argc = 2 one command line argument use OpenGL ES V3.x
    //  argc > 2 2 or more arguments then no graphics windows
    //
    if(argc<3) {
        if(argc==1) {
            // Use OpenGL Core 3.3
            fmt.setRenderableType(QSurfaceFormat::OpenGL);
            fmt.setVersion(3, 3);
            fmt.setProfile(QSurfaceFormat::CoreProfile);
        } else {
            // Use OpenGLES 3.0
            fmt.setRenderableType(QSurfaceFormat::OpenGLES);
            fmt.setVersion(3, 0);
            fmt.setProfile(QSurfaceFormat::NoProfile);
        }

        fmt.setDepthBufferSize(24);
        fmt.setStencilBufferSize(8);
        fmt.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
        fmt.setSamples(0);

        QSurfaceFormat::setDefaultFormat(fmt);
    }

    // not using this theme
    // This is a Linux thing not a Windows thing
    // This gets rid of meaningless Qt log output
    // since it may not find a theme through DBUS
    QLoggingCategory::setFilterRules(
        "qt.qpa.theme.gnome=false\n"
        );

    QApplication app(argc, argv);

    MainWindow* w;
    if(argc==1) {
        // no args then OpenGL Core 3.3
        w = new MainWindow(false,3,3);
    } else if(argc==2) {
        // 1 arg then OpenGL ES 3.x
        w = new MainWindow(true,3,0);
    } else {
        // 2 or more args, no graphics
        // disable point cloud viewer and rate chart
        w = new MainWindow(false,0,0);
   }
    w->show();

    return app.exec();
}

