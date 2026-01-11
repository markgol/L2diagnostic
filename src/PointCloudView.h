//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: PointCloudView.h
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
//  V0.2.4  2026-01-10  Updated OpenGL approach
//
//--------------------------------------------------------
#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QVector3D>
#include <QPoint>

#include "AxisGridRenderer.h"

//--------------------------------------------------------
//  This point cloud renderer is completely sensor indepedent
//  point data is passed using the PCpoint structure
//--------------------------------------------------------

//--------------------------------------------------------
// additional definitions
//--------------------------------------------------------

//--------------------------------------------------------
// This is a common representation used for point cloud data
// in many other applications.
// x,y,z repersent position relative to the sensor poistion
// with the x,y plane normally parallel to the earth plane.
// Time can be used to determine the age of a point. It is
// normally in seconds.
// The ring value is used by some sensors to identify data slices
//--------------------------------------------------------
struct PCpoint
{
    float x;
    float y;
    float z;
    float intensity;
    float time;
    uint32_t ring;
};

//--------------------------------------------------------
// used for coloring point in cloud
// color can be based on differnt values
// such a point age, point intensity, point height, etc
//--------------------------------------------------------
struct GLPoint
{
    QVector3D pos;    // x, y, z
    QVector3D color;
};

//--------------------------------------------------------
//  This is temporary
//  This should be replaced by autoscaling from the
//  point cloud or set by user to match the sensor
//--------------------------------------------------------
constexpr float INTENSITY_MIN = 0.0f;
constexpr float INTENSITY_MAX = 255.0f;

// maximum number of points in viewer
static constexpr int POINTS_PER_FRAME = 300;
static constexpr int MAX_FRAMES       = 3000;
static constexpr int MAX_POINTS       = MAX_FRAMES * POINTS_PER_FRAME;

//--------------------------------------------------------
// class definitions
//--------------------------------------------------------
class PointCloudView final
    : public QOpenGLWidget
    , protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT

public:
    explicit PointCloudView(QWidget* parent = nullptr);
    ~PointCloudView() override;

public slots:
    void setPointCloud(const QVector<PCpoint>& points);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

private:  
    void updateViewMatrix();

    QOpenGLShaderProgram m_program;

    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_vbo { QOpenGLBuffer::VertexBuffer };

    int m_pointCount = 0;

    float m_distance = 10.0f;
    float m_yaw = -15.0f;
    float m_pitch = -10.0f;
    QVector3D m_cameraTarget { 0.0f, 0.0f, 0.0f };
    QPoint m_lastMouse;

    //QVector<PCpoint> m_points;

    AxisGridRenderer m_axisGrid;
    QMatrix4x4 m_proj;
    QMatrix4x4 m_view;

};
