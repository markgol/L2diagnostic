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
//  V0.3.0  2026-01-18  Changed point cloud viewer back
//								into regular OpenGL window
//								Dockable QT windows with OpenGL
//								was not tractable
//                      updated mouse actions
//                      added default view settings
//  V0.3.2  2026-01-22  New renderer architecture
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
//      throttling         fifo, save only every nth PC framet
//          |
//      PointCloudWindow::appendFrame()
//          |
//      VBO sub-write (or stage CPU buffer)
//          |
//      requestUpdate->renderer
//          |
//          |
//      QOpenGLWindow::paintGL()
//
//--------------------------------------------------------

#include "PointCloudWindow.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QGuiApplication>
#include <QScreen>
#include <QStandardPaths>
#include <algorithm>
#include <cmath>

//--------------------------------------------------------
//  PointCloudWindow constructor
//--------------------------------------------------------
PointCloudWindow::PointCloudWindow(int maxPoints, QWindow* parent)
    : QOpenGLWindow(NoPartialUpdate, parent), m_maxPoints(maxPoints)
{
    setTitle("Point Cloud Viewer");
    resize(640, 480);
    setFlags(flags() | Qt::Window);
    setProperty("_q_deleteOnClose", true);

    m_accumulatedPoints.reserve(m_maxPoints);
}

//--------------------------------------------------------
//  PointCloudWindow deconstructor
//--------------------------------------------------------
PointCloudWindow::~PointCloudWindow() = default;

void PointCloudWindow::closeEvent(QCloseEvent* e)
{
    e->accept();
}

//========================================================
//  PointCloudWindow methods
//========================================================

//--------------------------------------------------------
//  initializeGL
//--------------------------------------------------------
void PointCloudWindow::initializeGL()
{
    // this won't work until maxPoints, maxFrame and maxPointPerFrame are defined
    if(m_maxPoints==0) {
        return;
    }
    initializeOpenGLFunctions();

    // ---- HARD RESET ----
    m_program.removeAllShaders();
    if (m_vao.isCreated()) m_vao.destroy();
    if (m_vbo.isCreated()) m_vbo.destroy();
    m_axisGrid.reset();
    m_pointCount = 0;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glClearColor(0.05f, 0.05f, 0.05f, 1.0f);

    // ---- Shaders ----
    m_program.addShaderFromSourceCode(QOpenGLShader::Vertex,
        "#version 330 core\n"

            "layout(location = 0) in vec3 inPos;\n"
            "layout(location = 1) in float inIntensity;\n"

            "uniform mat4 mvp;\n"

            "uniform float uMinRange;\n"
            "uniform float uMaxRange;\n"
            "uniform float uMinIntensity;\n"
            "uniform float uMaxIntensity;\n"

            "out float vRangeNorm;\n"
            "out float vIntensityNorm;\n"

            "void main()\n"
            "{\n"
            "   float range = length(inPos);\n"

            "   vRangeNorm = clamp("
            "           (range - uMinRange) / (uMaxRange - uMinRange),"
            "           0.0, 1.0"
            "           );\n"

            "   vIntensityNorm = clamp("
            "          (inIntensity - uMinIntensity) / (uMaxIntensity - uMinIntensity),"
            "          0.0, 1.0"
            "          );\n"

            "   gl_Position = mvp * vec4(inPos, 1.0);\n"
            "   gl_PointSize = 2.0;\n"
        "}\n"
                                      );


    m_program.addShaderFromSourceCode(QOpenGLShader::Fragment,
    "#version 330 core\n"

        "in float vRangeNorm;\n"
        "in float vIntensityNorm;\n"

        "out vec4 outColor;\n"

        "vec3 hsv2rgb(vec3 c)\n"
        "{\n"
        "   vec4 K = vec4(1.0, 2.0/3.0, 1.0/3.0, 3.0);\n"
        "   vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);\n"
        "   return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);\n"
        "}\n"

        "void main()\n"
        "{\n"
        "   float hue = (1.0 - vRangeNorm) * 0.66; // blue → red\n"
        "   float saturation = 1.0;\n"
        "   float value = mix(0.2, 1.0, vIntensityNorm);\n"

        "   vec3 rgb = hsv2rgb(vec3(hue, saturation, value));\n"
        "   outColor = vec4(rgb, 1.0);\n"
        "}\n"
                                      );

    if (!m_program.link())
        return;

    // ---- Buffers ----
    m_vao.create();
    m_vbo.create();

    m_vao.bind();
    m_vbo.bind();

    m_vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_vbo.allocate(m_maxPoints * sizeof(GLPoint));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                          sizeof(GLPoint),
                          reinterpret_cast<void*>(offsetof(GLPoint, pos)));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE,
                          sizeof(GLPoint),
                          reinterpret_cast<void*>(offsetof(GLPoint, intensity)));

    m_vbo.release();
    m_vao.release();

    m_axisGrid.initialize();
    updateViewMatrix();

}

//--------------------------------------------------------
//  resizeGL
//--------------------------------------------------------
void PointCloudWindow::resizeGL(int w, int h)
{
    Q_UNUSED(w)
    Q_UNUSED(h)
    updateViewMatrix();
}

//--------------------------------------------------------
//  paintGL
//--------------------------------------------------------
void PointCloudWindow::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 mvp = m_proj * m_view;

    m_axisGrid.render(mvp);

    if (m_pointCount == 0)
        return;

    m_program.bind();

    m_program.setUniformValue("uMinRange", m_minRange);
    m_program.setUniformValue("uMaxRange", m_maxRange);
    m_program.setUniformValue("uMinIntensity", m_minIntensity);
    m_program.setUniformValue("uMaxIntensity", m_maxIntensity);

    m_program.setUniformValue("mvp", mvp);

    m_vao.bind();
    glDrawArrays(GL_POINTS, 0, m_pointCount);
    m_vao.release();

    m_program.release();
}

//--------------------------------------------------------
//  updateViewMatrix
//--------------------------------------------------------
void PointCloudWindow::updateViewMatrix()
{
    const QVector3D eye = cameraPosition();

    m_view.setToIdentity();
    m_view.lookAt(eye, m_target, QVector3D(0, 0, 1));

    m_proj.setToIdentity();
    m_proj.perspective(
        m_fov,
        float(width()) / float(height()),
        m_nearPlane,
        m_farPlane
        );

    m_mvp = m_proj * m_view;
}

//--------------------------------------------------------
//  onRenderTick
//--------------------------------------------------------
void PointCloudWindow::onRenderTick()
{
    if (isExposed())
        if (!m_accumulatedPoints.isEmpty()) {
            uploadAccumulatedPoints();
        }
        requestUpdate();
}

//--------------------------------------------------------
//  uploadAccumulatedPoints
//--------------------------------------------------------
void PointCloudWindow::uploadAccumulatedPoints()
{
    // ??? if (!m_glInitialized || !m_glValid)
    //     return;

    makeCurrent();

    if (!m_vbo.isCreated()) {
        qWarning() << "VBO not created";
        doneCurrent();
        return;
    }

    m_vbo.bind();

    const int byteSize = m_accumulatedPoints.size() * sizeof(GLPoint);

    if (byteSize > 0) {
        m_vbo.allocate(m_accumulatedPoints.constData(), byteSize);
    }

    m_vbo.release();
    doneCurrent();
    m_pointCount = m_accumulatedPoints.size();
}

//--------------------------------------------------------
//  appendFrame
//--------------------------------------------------------
void PointCloudWindow::appendFrame(const Frame& frame)
{
    if (!m_vbo.isCreated() || frame.isEmpty())
        return;

    // qDebug() << "appendFrame(): incoming =" << frame.size()
    //          << " total before =" << m_accumulatedPoints.size();

    for (const auto& p : frame) {
        GLPoint gp;
        gp.pos = QVector3D(p.x, p.y, p.z);
        gp.intensity = p.intensity;

        m_accumulatedPoints.push_back(gp);
    }

    // Trim oldest points if exceeding capacity
    if (m_accumulatedPoints.size() > m_maxPoints) {
        const int excess = m_accumulatedPoints.size() - m_maxPoints;
        m_accumulatedPoints.erase(
            m_accumulatedPoints.begin(),
            m_accumulatedPoints.begin() + excess);
    }

    uploadAccumulatedPoints();
    update();
}

//========================================================
//  setDefaultPCsettings
//========================================================
void PointCloudWindow::setDefaultPCsettings(PCsettings& settings)
{
    DefaultPCsettings = settings;
}

//========================================================
//  mouse GUI controls
//========================================================

//--------------------------------------------------------
//  mousePressEvent
//--------------------------------------------------------
void PointCloudWindow::mousePressEvent(QMouseEvent* e)
{
    m_lastMousePos  = e->pos();
}

//--------------------------------------------------------
//  mouseMoveEvent
//--------------------------------------------------------
void PointCloudWindow::mouseMoveEvent(QMouseEvent* e)
{
    const QPoint delta = e->pos() - m_lastMousePos;
    m_lastMousePos = e->pos();

    //  shift+Left+Right buttons+movement
    //  resets view
    if ((e->buttons() & Qt::LeftButton) &&
        (e->buttons() & Qt::RightButton) &&
        e->modifiers() & Qt::ShiftModifier) {
        ResetView();
        updateViewMatrix();
        update();
        return;
    }

    // left+right buttons+mmovement
    //  pan x,y
    if ((e->buttons() & Qt::LeftButton) &&
        (e->buttons() & Qt::RightButton))
    {
        const float panSpeed = 0.002f * mPCsettings.Distance;
        const float yawRad = qDegreesToRadians(mPCsettings.Yaw);

        QVector3D right(
            std::cos(yawRad),
            -std::sin(yawRad),
            0.0f
            );

        QVector3D forward(
            std::sin(yawRad),
            std::cos(yawRad),
            0.0f
            );

        m_target -= right   * delta.x() * panSpeed;
        m_target += forward * delta.y() * panSpeed;

        updateViewMatrix();
        update();
        return;
    }

    // left button
    // yaw and pitch
    if (e->buttons() & Qt::LeftButton) {
        mPCsettings.Yaw   += delta.x() * 0.3f;
        mPCsettings.Pitch += delta.y() * 0.3f;

        mPCsettings.Pitch = std::clamp(mPCsettings.Pitch, m_pitchMin, m_pitchMax);
        updateViewMatrix();
        update();
        return;
    }

    // right button
    // z pan
    if (e->buttons() & Qt::RightButton) {
        const float panSpeed = 0.002f * mPCsettings.Distance;

        // Vertical pan only (Z-up)
        m_target.setZ(m_target.z() + delta.y() * panSpeed);

        updateViewMatrix();
        update();
        return;
    }
}

//--------------------------------------------------------
//  wheelEvent
//--------------------------------------------------------
void PointCloudWindow::wheelEvent(QWheelEvent* e)
{
    const float zoomFactor = std::pow(1.001f, -e->angleDelta().y());

    mPCsettings.Distance *= zoomFactor;
    mPCsettings.Distance = std::clamp(mPCsettings.Distance, m_minDistance, m_maxDistance);

    updateViewMatrix();
    update();
}

//--------------------------------------------------------
//  cameraPosition
//--------------------------------------------------------
QVector3D PointCloudWindow::cameraPosition() const
{
    const float yawRad   = qDegreesToRadians(mPCsettings.Yaw);
    const float pitchRad = qDegreesToRadians(mPCsettings.Pitch);

    QVector3D eye;
    eye.setX(mPCsettings.Distance * std::cos(pitchRad) * std::sin(yawRad));
    eye.setY(mPCsettings.Distance * std::cos(pitchRad) * std::cos(yawRad));
    eye.setZ(mPCsettings.Distance * std::sin(pitchRad));

    return eye + m_target;
}

//========================================================
//  Window State
//========================================================

//--------------------------------------------------------
//  saveWindowState
//  This saves the window geometry and state
//  This does not save the point cloud view
//--------------------------------------------------------
void PointCloudWindow::saveWindowState() const
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    settings.beginGroup("PointCloudWindow");
    settings.setValue("geometry", geometry());
    settings.setValue("maximized", visibility() == Maximized);
    settings.endGroup();
}

void PointCloudWindow::saveWindowState(QSettings& settings) const
{
    settings.beginGroup("PointCloudWindow");
    settings.setValue("geometry", geometry());
    settings.setValue("maximized", visibility() == Maximized);
    settings.endGroup();
}

//--------------------------------------------------------
//  restoreWindowState
//  This only restores the window geometry and state
//  This does not affect the point cloud view
//--------------------------------------------------------
void PointCloudWindow::restoreWindowState()
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    settings.beginGroup("PointCloudWindow");
    QRect geom = settings.value("geometry").toRect();
    bool maximized = settings.value("maximized", false).toBool();
    settings.endGroup();

    if (geom.isValid()) {
        ensureVisibleOnScreen(geom);
        setGeometry(geom);
        maximized ? showMaximized() : show();
    }
}

void PointCloudWindow::restoreWindowState(QSettings& settings)
{
    settings.beginGroup("PointCloudWindow");
    QRect geom = settings.value("geometry").toRect();
    bool maximized = settings.value("maximized", false).toBool();
    settings.endGroup();

    if (geom.isValid()) {
        ensureVisibleOnScreen(geom);
        setGeometry(geom);
        maximized ? showMaximized() : show();
    }
}

//--------------------------------------------------------
//  ResetView
//--------------------------------------------------------
void PointCloudWindow::ResetView()
{
    m_target = QVector3D(0,0,0);
    mPCsettings.Distance = DefaultPCsettings.Distance;
    mPCsettings.Yaw = DefaultPCsettings.Yaw;
    mPCsettings.Pitch = DefaultPCsettings.Pitch;

    updateViewMatrix();
    update();
}

//--------------------------------------------------------
//  getPCsettings
//--------------------------------------------------------
void PointCloudWindow::getPCsettings(PCsettings& settings)
{
    settings.Distance = mPCsettings.Distance;
    settings.Pitch = mPCsettings.Pitch;
    settings.Yaw = mPCsettings.Yaw;
}

//--------------------------------------------------------
//  setPCsettings
//--------------------------------------------------------
void PointCloudWindow::setPCsettings(PCsettings& settings)
{
    mPCsettings.Distance = settings.Distance;
    mPCsettings.Pitch = settings.Pitch;
    mPCsettings.Yaw = settings.Yaw;

    m_target = QVector3D(0,0,0);

    updateViewMatrix();
    update();
}

//--------------------------------------------------------
//  ensureVisibleOnScreen
//--------------------------------------------------------
void PointCloudWindow::ensureVisibleOnScreen(QRect& geom) const
{
    for (QScreen* s : QGuiApplication::screens())
        if (s->availableGeometry().intersects(geom))
            return;

    QRect primary = QGuiApplication::primaryScreen()->availableGeometry();
    geom.moveCenter(primary.center());
}
