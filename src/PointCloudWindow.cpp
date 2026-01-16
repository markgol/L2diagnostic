//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: PointCloudWindow.cpp
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
//  of packets and optionally saves them to a CSV file.
//
//  V0.2.3  2026-01-09  Added Point Cloud Renderer
//
//--------------------------------------------------------
#include "PointCloudWindow.h"

//--------------------------------------------------------
//  Point cloud viewer operates as an separate window
//  from the main GUI
//--------------------------------------------------------

PointCloudWindow::PointCloudWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("L2 diagnostic 3D Point Cloud Viewer");
    resize(800, 600);

    m_view = new PointCloudView(this);
    setCentralWidget(m_view);

    // Independent render timer (30 60 Hz)
    m_renderTimer.setInterval(33); // ~30 FPS
    connect(&m_renderTimer, &QTimer::timeout,
            m_view, QOverload<>::of(&PointCloudView::update));
    m_renderTimer.start();


}
