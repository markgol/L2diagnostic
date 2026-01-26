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
// create cloud viewer window
// This is not a Qt window but a OpenGL managed window
// if(mmaxPoints>=50000) {
//     // only open point cloud window if there is a minimum
//     // number point cloud buffer size
//     // configure OpenGL before creating PointCloudWindow class
//     // so that it has the correct OpenGL context
//     QSurfaceFormat format;
//     format.setVersion(3, 3);
//     format.setProfile(QSurfaceFormat::CoreProfile);
//     format.setDepthBufferSize(24);
//     format.setRenderableType(QSurfaceFormat::OpenGL);

//     QSurfaceFormat::setDefaultFormat(format);
//     m_pointCloudWindow = new PointCloudWindow(mmaxPoints);
//     // set default view settings
//     SetDefaultView();
//     m_pointCloudWindow->setTransientParent(windowHandle());
//     m_pointCloudWindow->Initialize();
// }
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
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QVector3D>
#include <QPoint>
#include <QSettings>

#include "AxisGridRenderer.h"

// ---- Lidar point ----
struct PCpoint
{
    float x;
    float y;
    float z;
    float intensity; //  0-255
    float time; // timestamp in seconds
    uint32_t ring;
};

struct GLPoint
{
    QVector3D pos;
    float intensity;
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

class PointCloudWindow final
    : public QOpenGLWindow
    , protected QOpenGLFunctions_3_3_Core
{
public:
    explicit PointCloudWindow(int maxPoints, QWindow* parent = nullptr);
    ~PointCloudWindow() override;

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
    QOpenGLShaderProgram m_program;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_vbo{ QOpenGLBuffer::VertexBuffer };

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
    PCsettings mPCsettings {10.0,145.0,20.0, 2, 0.1, 10.0};

    PCsettings DefaultPCsettings {10.0,145.0,20.0, 2, 0.1, 10.0};

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
