//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: PointCloudWindow.h
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
//  of packets.
//
//  V0.3.0  2026-01-18  Changed point cloud viewer back
//								into regular OpenGL window
//								Dockable QT windows with OpenGL
//								was not tractable
//                      updated mouse actions
//                      added default view settings
//  V0.3.2  2026-01-22  New renderer architecture
//  V0.3.3  2026-01-23  New renderer architecture completed
//  V0.3.5  2026-01-24  Moved the creation of the
//                      point cloud window into the class
//                      Moved much of the closing of the class here
//  V0.3.6  2026-01-24  Added clear point cloud
//  V0.3.9  2026-02-01  added LOAD/SAVE point cloud
//  V0.3.12 2026-02-05  Moved renderer timer to PointCloudWindow class
//  V0.4.0  2026-02-06  Added implementation for specific OpenGL
//                      If no cmd line arguments are present then
//                          OpenGL Core 3.3 is used.
//                      if any command line argument is present then
//                          OpenGLES V3.0 is used
//                      Added error checking for intializeGL()
//                      Saving the point time got lost, added back in
//                      Changed file save/load to use standard
//                          PCL PCD formatted file
//
//--------------------------------------------------------

//--------------------------------------------------------
// This uses the following Unitree L2 sources modules:
//      unitree_lidar_protocol.h
//      unitree_lidar_utilities
// The orignal source can be found at:
//      https://github.com/unitreerobotics/unilidar_sdk2
//      under License: BSD 3-Clause License (see files)
//
// Corrections/additions have been made to these 2 files
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
//
// Example of using PointCloudWindow
//
// ****** create cloud viewer window ******
//
// #include "PointCloudWindow.h"
//
// this needs updating
//
//--------------------------------------------------------


//--------------------------------------------------------
//  Data flow for point cloud
//      MainWindow:onNewLidarFrame()  this operates at L2 point cloud
//          |              packet rate ~200-250 packets/sec
//          |
//      throttling         save only every nth PC frame ( 0 every frame)
//          |
//      PointCloudWindow::appendFrame()
//          |
//      VBO sub-write       accumulates cloud points from frames
//          |
//      requestUpdate->renderer   queued for next paintGL
//          |
//      QOpenGLWindow::paintGL()  timer driven typically at 30-60Hz
//
//--------------------------------------------------------

#pragma once

#include <QOpenGLWindow>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QVector3D>
#include <QPoint>
#include <QSettings>
#include <QFile>
#include <QDataStream>
#include <QTimer>
//#include <QMessageBox>

#include "AxisGridRenderer.h"
#include "PCpoint.h"

// this identifies file format
// x,y,z,i
#define PCD1 0x50434431

// this identifies file format
// x,y,z,i,time
#define PCD2 0x50434432

struct GLPoint
{
    QVector3D pos;
    float intensity;
    float time;
};

// struct for the point cloud viewer state
typedef struct
{
    float Distance;
    float Yaw;
    float Pitch;
    float PointSize;
    float MinDistance;
    float MaxDistance;
} PCsettings;

// L2 lidar point has a min intensity of 0 and a max of 255
constexpr float INTENSITY_MIN = 0.0f;
constexpr float INTENSITY_MAX = 255.0f;

using Frame = QVector<PCpoint>;

// class PointCloudWindow

class PointCloudWindow : public QOpenGLWindow
    , protected QOpenGLFunctions

{
public:
    explicit PointCloudWindow(int maxPoints, bool OpenGLES, QWindow* parent = nullptr);
    ~PointCloudWindow() override;

    // intialization failed
    // This should always be called after creation of the class
    // if it has failed the class shouldbe deleted
    bool PointCouldInitFailed() {return mInitializeGLsuccess;}

    // Renderer Timer
    void InitializeRenderTimer(int Rate);
    void RendererTimerStart();
    void RendererTimerStop();
    void SetInterval(int Rate);
    int GetInterval();

    // Explicit persistence API
    void saveWindowState() const;
    void saveWindowState(QSettings& settings) const;

    void restoreWindowState();
    void restoreWindowState(QSettings& settings);

    void ResetView();

    // push frame from mainwindow to pointcloudwindow
    void appendFrame(const Frame& frame);

    // settings for point cloud viewer
    void getPCsettings(PCsettings& settings); // current settings
    void setPCsettings(PCsettings& settings); // current settings
    void setDefaultPCsettings(PCsettings& settings);
    void Initialize();
    void getPointSizeRange(float *SizeRange);
    void clearPointCloud();

    // File I/O
    bool savePCD(const QString& fileName);
    bool loadPCD(const QString& fileName);

    void setPointCloud(const QVector<GLPoint>& cloud);

public slots:
    void onRenderTick(); // timer driven renderer

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

private:
    void updateViewMatrix();
    void ensureVisibleOnScreen(QRect& geom) const;
    QVector3D cameraPosition() const;
    void closeEvent(QCloseEvent* e) override;
    void uploadAccumulatedPoints();

private:
    bool mOpenGLES;

    QOpenGLShaderProgram m_program;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_vbo{ QOpenGLBuffer::VertexBuffer };
    bool mInitializeGLsuccess{false};

    // Point cloud window renderer Timer
    QTimer* RendererTimer;

    AxisGridRenderer m_axisGrid;

    // buffering
    QVector<GLPoint> m_accumulatedPoints;
    int m_pointCount{0};
    const int m_maxPoints;  // maximum number of points
                            // passed as argument in constructor
    int m_writeOffset {0};
    bool m_wrapped{false};

    // =====================
    // Orbit camera state
    // =====================
    // camera view orientation
    // Right   = +X = ( 1, 0, 0 )
    // Forward = +Y = ( 0, 1, 0 )
    // Up      = +Z = ( 0, 0, 1 )

    QVector3D m_target { 0.0f, 0.0f, 0.0f };
    PCsettings mPCsettings {10.0f,145.0f,20.0f, 2, 0.1f, 10.0f};

    PCsettings DefaultPCsettings {10.0f,145.0f,20.0f, 2, 0.1f, 10.0f};

    // limits
    float m_pitchMin = -89.0f; // min pitch angle (degress)
    float m_pitchMax =  89.0f; // max pitch angle (degress)

    // projection
    float m_fov       = 60.0f;
    float m_nearPlane = 0.01f;
    float m_farPlane  = 1000.0f;

    // coloring limits
    float m_minIntensity = 0.0f;
    float m_maxIntensity = 255.0f;

    // ---- Matrices ----
    QMatrix4x4 m_view;
    QMatrix4x4 m_proj;
    QMatrix4x4 m_mvp;

    // interaction
    QPoint m_lastMousePos;
};
