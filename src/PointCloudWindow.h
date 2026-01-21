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
//  of packets and optionally saves them to a CSV file.
//
//  V0.3.0  2026-01-18  Changed point cloud viewer back
//								into regular OpenGL window
//								Dockable QT windows with OpenGL
//								was not tractable
//                      updated mouse actions
//                      added default view settings
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
//  Data flow for point cloud
//      MainWindow:onNewLidarFrame()  this operates at L2 point cloud
//          |              packet rate ~200-250 packets/sec
//          |
//      throttling         fifo, save only every nth PCpoint
//          |
//      MainWindow:updatePointCloud() create flattened point cloud
//          |              cloudTimer Timer driven
//          |
//      MainWindow:flattenedCloudReady() signals next step
//          |
//      PointCloudWindow::setPointCloud()
//          |              push flattened cloud to viewer
//          |
//      PointCloudWindow::requestUpdate()
//                         repaint window
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
    QVector3D color;
};

// struct for the point cloud viewer state
typedef struct
{
    float Distance;
    float Yaw;
    float Pitch;
} PCsettings;

// L2 lidar point has a min intensity of 0 and a max of 255
constexpr float INTENSITY_MIN = 0.0f;
constexpr float INTENSITY_MAX = 255.0f;

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

    void setPointCloud(const QVector<PCpoint>& points);

    // settings for point cloud viewer
    void getPCsettings(PCsettings& settings); // current settings
    void setPCsettings(PCsettings& settings); // current settings
    void setDefaultPCsettings(PCsettings& settings);

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

private:
    QOpenGLShaderProgram m_program;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_vbo{ QOpenGLBuffer::VertexBuffer };

    AxisGridRenderer m_axisGrid;

    QVector<PCpoint> m_lastCloud;
    int m_pointCount{0};

    // buffering
    const int m_maxPoints;
    QVector<GLPoint> m_stagingBuffer;

    // =====================
    // Orbit camera state
    // =====================
    // camera view orientation
    // Right   = +X = ( 1, 0, 0 )
    // Forward = +Y = ( 0, 1, 0 )
    // Up      = +Z = ( 0, 0, 1 )

    QVector3D m_target { 0.0f, 0.0f, 0.0f };
    PCsettings mPCsettings {10.0,145.0,20.0};

    PCsettings DefaultPCsettings {10.0,145.0,20.0};

    float m_minDistance = 0.1f; // closest distance (M)
    float m_maxDistance = 1000.0f; // farthest distance (M)

    // limits
    float m_pitchMin = -89.0f; // min pitch angle (degress)
    float m_pitchMax =  89.0f; // max pitch angle (degress)

    // projection
    float m_fov       = 60.0f;
    float m_nearPlane = 0.01f;
    float m_farPlane  = 1000.0f;

    // ---- Matrices ----
    QMatrix4x4 m_view;
    QMatrix4x4 m_proj;
    QMatrix4x4 m_mvp;

    // interaction
    QPoint m_lastMousePos;
};
