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
//  V0.4.1  2026-02-11  Remove Qdebug statements
//  V0.4.3  2026-02-18  Added range to cloud point
//  V0.4.4  2026-02-26  Changed LoadPCD() and SavePCD to use updated
//                      time format change from float to int64_t
//                      Changed time in GLpoint from float to int64_t
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
//      throttling         capture every nth PC frame ( 0 every frame)
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
PointCloudWindow::PointCloudWindow(int maxPoints, bool OpenGLES, QWindow* parent)
    : QOpenGLWindow(NoPartialUpdate, parent), mOpenGLES(OpenGLES), m_maxPoints(maxPoints)
{
    setTitle("Point Cloud Viewer");
    resize(320,240); // this is just initial size if it had never been created before
    setFlags(flags() | Qt::Window);

    // make sure to delete window when program exits
    // rather than hang until user close this window.
    setProperty("_q_deleteOnClose", true);

    // preaalocate the accumlated point cloud buffer
    // This is all the points since the since the last render
    // It also allows for loading a file up the
    // the max point size
    m_accumulatedPoints.reserve(m_maxPoints);
}

//--------------------------------------------------------
//  PointCloudWindow deconstructor
//--------------------------------------------------------
PointCloudWindow::~PointCloudWindow() = default;


//--------------------------------------------------------
//  closeEvent deconstructor
//  ensure window gets close when application closes
//--------------------------------------------------------
void PointCloudWindow::closeEvent(QCloseEvent* e)
{
    e->accept();
}

//========================================================
//  PointCloudWindow methods
//========================================================


//--------------------------------------------------------
//  Initialize
//--------------------------------------------------------
void PointCloudWindow::Initialize()
{
    setFlag(Qt::Window);
    // Window geometry and state for point cloud window
    restoreWindowState();
    ResetView();
}

//--------------------------------------------------------
//  InitializeRenderTimer
// setup timer point cloud renderering
//--------------------------------------------------------
void PointCloudWindow::InitializeRenderTimer(int Rate) {

    RendererTimer = new QTimer(this);
    RendererTimer->setInterval(Rate);
    // caller must explicitly start the timer
    RendererTimer->stop();
    connect(RendererTimer,
        &QTimer::timeout,
        this, &PointCloudWindow::onRenderTick);
}

//--------------------------------------------------------
//  RendererTimerStart
//--------------------------------------------------------
void PointCloudWindow::RendererTimerStart()
{
    RendererTimer->start();
}

//--------------------------------------------------------
//  RendererTimerStop
//--------------------------------------------------------
void PointCloudWindow::RendererTimerStop()
{
    RendererTimer->stop();
}

//--------------------------------------------------------
//  SetInterval
//  Sets the timer interval
//   goes into effect next time timer is started
//--------------------------------------------------------
void PointCloudWindow::SetInterval(int Rate)
{
    RendererTimer->setInterval(Rate);
}

//--------------------------------------------------------
//  GetInterval
//--------------------------------------------------------
int PointCloudWindow::GetInterval()
{
    return RendererTimer->interval();
}

//--------------------------------------------------------
//  initializeGL
//  Since this is a protected ovveride of type void
//  a member variable mInitializeGLsuccess is the error
//  return for this method.  You should check it immediatelty
//  after calling this method.
//
//  returns mInitializeGLsuccess = false if failed to intialize
//
//  Note: initializeOpenGLFunctions() is a built in OpenGL
//  method.  Unforutnately it can not be easily determiend which
//  OpenGL support is installed before this is called.
//  Ifit doesn't match the QSurfaceFormat settings set in main.cpp
//  then this will cause a segment fault.  It will also cause a
//  segment fault if the wrong setting is set in QSurfaceFormat.
//--------------------------------------------------------
void PointCloudWindow::initializeGL()
{
    // this won't work until maxPoints are defined
    if(m_maxPoints==0) {
        mInitializeGLsuccess = false;
        return;
    }
    // auto ctx = QOpenGLContext::currentContext();
    // Q_ASSERT(ctx);

    // qDebug() << "OpenGL:" << ctx->format().majorVersion()
    //          << ctx->format().minorVersion()
    //          << "GLES?" << ctx->isOpenGLES();

    initializeOpenGLFunctions();

    // ---- HARD RESET ----
    m_program.removeAllShaders();
    if (m_vao.isCreated()) m_vao.destroy();
    if (m_vbo.isCreated()) m_vbo.destroy();
    m_axisGrid.reset();
    m_pointCount = 0;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.05f, 0.05f, 0.05f, 1.0f);

    // mOpenGLES is set when the class is created
    // It should be set to match the version of OpenGL
    // on the system
    //      mOpenGLES   true  OpenGL ES 3.0
    //                  false OpenGL Core 3.3
    if(mOpenGLES) {
        // ---- Shaders for OpenGL ES V3.0 ----
        m_program.addShaderFromSourceCode(QOpenGLShader::Vertex,
            "#version 300 es\n"
            "precision highp float;\n"

            "layout(location = 0) in vec3 inPos;\n"
            "layout(location = 1) in float inIntensity;\n"

            "uniform mat4 mvp;\n"

            "uniform float uMinRange;\n"
            "uniform float uMaxRange;\n"
            "uniform float uMinIntensity;\n"
            "uniform float uMaxIntensity;\n"
            "uniform float uPointSize;\n"

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
            "   gl_PointSize = uPointSize;\n"
                                      "}\n"
        );


        m_program.addShaderFromSourceCode(QOpenGLShader::Fragment,
            "#version 300 es\n"
            "precision highp float;\n"

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
            "   // ---- circular point mask ----\n"
            "   vec2 c = gl_PointCoord * 2.0 - vec2(1.0);\n"
            "   if (dot(c,c) > 1.0)\n"
            "       discard;\n"
            "\n"
            "   float hue = (1.0 - vRangeNorm) * 0.66; // blue → red\n"
            "   float saturation = 1.0;\n"
            "   float value = mix(0.2, 1.0, vIntensityNorm);\n"

            "   vec3 rgb = hsv2rgb(vec3(hue, saturation, value));\n"
            "   outColor = vec4(rgb, 1.0);\n"
            "}\n"
        );
    } else {
        // ---- Shaders for OpenGL Core 3.3 ----
        m_program.addShaderFromSourceCode(QOpenGLShader::Vertex,
            "#version 330 core\n"

                "layout(location = 0) in vec3 inPos;\n"
                "layout(location = 1) in float inIntensity;\n"
                "layout(location = 2) in float inTime;\n"

                "uniform mat4 mvp;\n"

                "uniform float uMinRange;\n"
                "uniform float uMaxRange;\n"
                "uniform float uMinIntensity;\n"
                "uniform float uMaxIntensity;\n"
                "uniform float uPointSize;\n"

                "out float vRangeNorm;\n"
                "out float vIntensityNorm;\n"
                "out float vTime;\n"

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
                "   vTime = inTime; // future use \n"

                "   gl_Position = mvp * vec4(inPos, 1.0);\n"
                "   gl_PointSize = uPointSize;\n"
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
                "   // ---- circular point mask ----\n"
                "   vec2 c = gl_PointCoord * 2.0 - vec2(1.0);\n"
                "   if (dot(c,c) > 1.0)\n"
                "       discard;\n"
                "\n"
                "   float hue = (1.0 - vRangeNorm) * 0.66; // blue → red\n"
                "   float saturation = 1.0;\n"
                "   float value = mix(0.2, 1.0, vIntensityNorm);\n"

                "   vec3 rgb = hsv2rgb(vec3(hue, saturation, value));\n"
                "   outColor = vec4(rgb, 1.0);\n"
                "}\n"
            );
    }

    if (!m_program.link()) {
        //qDebug() << m_program.log();
        mInitializeGLsuccess = false;
        return;
    }

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

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE,
                          sizeof(GLPoint),
                          reinterpret_cast<void*>(offsetof(GLPoint, time)));

    m_vbo.release();
    m_vao.release();

    // notify set axis grid of correct OpenGL version
    // before intializing
    m_axisGrid.setOpenGLES(mOpenGLES);
    if(!m_axisGrid.initialize()) {
        // if can not create axis renderer then fail
        mInitializeGLsuccess = false;
        return;
    }
    updateViewMatrix();

    mInitializeGLsuccess = true;

    return;
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
//  getPointSizeRange
//--------------------------------------------------------
void PointCloudWindow::getPointSizeRange(float *SizeRange)
{
    glGetFloatv(GL_POINT_SIZE_RANGE, SizeRange);
    return;
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

    m_program.setUniformValue("mvp", mvp);
    m_program.setUniformValue("uMinRange", mPCsettings.MinDistance);
    m_program.setUniformValue("uMaxRange", mPCsettings.MaxDistance);
    m_program.setUniformValue("uMinIntensity", m_minIntensity);
    m_program.setUniformValue("uMaxIntensity", m_maxIntensity);
    float sizeRange[2];
    float pointsize = mPCsettings.PointSize;
    glGetFloatv(GL_POINT_SIZE_RANGE, sizeRange);
    if(pointsize < sizeRange[0])
        pointsize = sizeRange[0];
    if(pointsize > sizeRange[1])
        pointsize = sizeRange[1];

    m_program.setUniformValue("uPointSize", pointsize);

    m_vao.bind();
    if (!m_wrapped) {
        glDrawArrays(GL_POINTS, 0, m_pointCount);
    } else {
        glDrawArrays(GL_POINTS,
                     m_writeOffset,
                     m_maxPoints - m_writeOffset);

        glDrawArrays(GL_POINTS,
                     0,
                     m_writeOffset);
    }
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
    if (!isExposed())
        return;

    if (!m_accumulatedPoints.isEmpty()) {
        uploadAccumulatedPoints();   // GPU sync only
    }

    requestUpdate(); // schedules paintGL()
}


//--------------------------------------------------------
//  uploadAccumulatedPoints
//--------------------------------------------------------
void PointCloudWindow::uploadAccumulatedPoints()
{
    if (m_accumulatedPoints.isEmpty())
        return;

    m_vbo.bind();

    const int count = m_accumulatedPoints.size();
    if (count == 0) {
        m_vbo.release();
        return;
    }

    // how many we can write before wrapping
    const int spaceToEnd = m_maxPoints - m_writeOffset;

    // first chunk (until buffer end)
    const int firstChunk = qMin(count, spaceToEnd);

    m_vbo.write(m_writeOffset * sizeof(GLPoint),
                m_accumulatedPoints.constData(),
                firstChunk * sizeof(GLPoint));

    // second chunk (wrapped to beginning)
    const int remaining = count - firstChunk;
    if (remaining > 0) {
        m_vbo.write(0,
                    m_accumulatedPoints.constData() + firstChunk,
                    remaining * sizeof(GLPoint));
        m_wrapped = true;
    }

    // advance write offset
    m_writeOffset += count;
    if (m_writeOffset >= m_maxPoints) {
        m_writeOffset %= m_maxPoints;
        m_wrapped = true;
    }

    // update visible point count
    m_pointCount = m_wrapped ? m_maxPoints : m_writeOffset;

    m_vbo.release();
    m_accumulatedPoints.clear();

}

//--------------------------------------------------------
//  setPointCloud
//  This is only used for restoring a full point cloud
//  such as from a file load
//--------------------------------------------------------
void PointCloudWindow::setPointCloud(const QVector<GLPoint>& cloud)
{
    if (!m_vbo.isCreated())
        return;

    makeCurrent();

    // reset ring buffer state
    m_accumulatedPoints.clear();
    m_wrapped = false;
    m_writeOffset = 0;
    m_pointCount = 0;

    // stage data
    m_accumulatedPoints = cloud;

    // upload to GPU
    uploadAccumulatedPoints();

    doneCurrent();
    requestUpdate();
}

//--------------------------------------------------------
//  appendFrame
//--------------------------------------------------------
void PointCloudWindow::appendFrame(const Frame& frame)
{
    if (frame.isEmpty())
        return;

    QVector<GLPoint> converted;
    converted.reserve(frame.size());

    for (const auto& p : frame) {
        converted.push_back({
            QVector3D(p.x, p.y, p.z),
            p.intensity,
            p.range,
            p.time
        });
    }

    // Stage for GPU upload
    m_accumulatedPoints += converted;
}

//--------------------------------------------------------
//  clearPointCloud
//--------------------------------------------------------
void PointCloudWindow::clearPointCloud()
{
    // Clear CPU-side accumulation
    m_accumulatedPoints.clear();

    // Reset ring buffer state
    m_pointCount   = 0;
    m_writeOffset  = 0;
    m_wrapped      = false;

    // Clear GPU buffer safely if context exists
    if (isExposed() && m_vbo.isCreated())
    {
        makeCurrent();

        m_vbo.bind();
        m_vbo.allocate(m_maxPoints * sizeof(GLPoint)); // orphan old data
        m_vbo.release();

        doneCurrent();
    }

    requestUpdate();
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
    //mPCsettings.Distance = std::clamp(mPCsettings.Distance, m_minDistance, m_maxDistance);
    mPCsettings.Distance = std::clamp(mPCsettings.Distance, 0.1f, 200.0f);

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
    mPCsettings.PointSize = DefaultPCsettings.PointSize;
    mPCsettings.MinDistance = DefaultPCsettings.MinDistance;
    mPCsettings.MaxDistance = DefaultPCsettings.MaxDistance;

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
    settings.PointSize = mPCsettings.PointSize;
    settings.MinDistance = mPCsettings.MinDistance;
    settings.MaxDistance = mPCsettings.MaxDistance;
}

//--------------------------------------------------------
//  setPCsettings
//--------------------------------------------------------
void PointCloudWindow::setPCsettings(PCsettings& settings)
{
    mPCsettings.Distance = settings.Distance;
    mPCsettings.Pitch = settings.Pitch;
    mPCsettings.Yaw = settings.Yaw;
    mPCsettings.PointSize = settings.PointSize;
    mPCsettings.MinDistance = settings.MinDistance;
    mPCsettings.MaxDistance = settings.MaxDistance;

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

//========================================================
//  File I/O
//========================================================


//--------------------------------------------------------
//  savePCD
//--------------------------------------------------------
bool PointCloudWindow::savePCD(const QString& fileName)
{
    if (m_pointCount == 0)
        return false;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
        return false;

    QTextStream out(&file);
    out.setRealNumberPrecision(8);

    const int count = m_pointCount;

    // ---- Header ----
    out << "# .PCD v0.7 - Point Cloud Data file format\n";
    out << "VERSION 0.7\n";
    out << "FIELDS x y z intensity range time\n";
    out << "SIZE 4 4 4 4 4 8\n";
    out << "TYPE F F F F F I\n";
    out << "COUNT 1 1 1 1 1 1\n";
    out << "WIDTH " << count << "\n";
    out << "HEIGHT 1\n";
    out << "VIEWPOINT 0 0 0 1 0 0 0\n";
    out << "POINTS " << count << "\n";
    out << "DATA binary\n";
    out.flush();

    // ---- Write binary data ----
    makeCurrent();
    m_vbo.bind();

    const GLPoint* src = static_cast<const GLPoint*>(
        m_vbo.map(QOpenGLBuffer::ReadOnly)
        );

    if (!src) {
        m_vbo.unmap();
        m_vbo.release();
        doneCurrent();
        file.close();
        return false;
    }

    // Handle ring buffer wrap
    const int tailCount = m_maxPoints - m_writeOffset;
    int writesize;
    int written;
    if (!m_wrapped || tailCount==m_maxPoints) { // ring buffer not wrapped
        // contiguous block [0 .. m_pointCount)
        writesize = m_pointCount * sizeof(GLPoint);
        written = file.write(reinterpret_cast<const char*>(src),
                   writesize);
        if(written!=writesize) {
            m_vbo.unmap();
            m_vbo.release();
            doneCurrent();
            file.close();
            return false;
        }
    }
    else {
        // tail: [writeOffset .. maxPoints)
        writesize = tailCount * sizeof(GLPoint);
        if(writesize!=0) {
            written = file.write(reinterpret_cast<const char*>(src + m_writeOffset),
                   writesize);
            if(written!=writesize) {
                m_vbo.unmap();
                m_vbo.release();
                doneCurrent();
                file.close();
                return false;
            }
        }
        // head: [0 .. writeOffset)
        writesize = m_writeOffset * sizeof(GLPoint);
        if(writesize!=0) {
            written = file.write(reinterpret_cast<const char*>(src),
                        writesize);
            if(written!=writesize) {
                m_vbo.unmap();
                m_vbo.release();
                doneCurrent();
                file.close();
                return false;
            }
        }
    }

    m_vbo.unmap();
    m_vbo.release();
    doneCurrent();
    file.close();

    return true;
}


//--------------------------------------------------------
//  loadPCD
//--------------------------------------------------------
bool PointCloudWindow::loadPCD(const QString& fileName)
{
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
        return false;

    QTextStream header(&file);

    QStringList fields;
    QList<int> sizes;
    QList<char> types;
    QList<int> counts;
    int pointCount = 0;
    bool binary = false;

    // ---- Parse header ----
    while (!header.atEnd()) {
        QString line = header.readLine().trimmed();

        if (line.isEmpty() || line.startsWith("#"))
            continue;

        QString key = line.section(' ', 0, 0);

        if (key == "FIELDS") {
            fields = line.split(' ', Qt::SkipEmptyParts).mid(1);
        }
        else if (key == "SIZE") {
            auto parts = line.split(' ', Qt::SkipEmptyParts).mid(1);
            for (auto& p : parts) sizes.append(p.toInt());
        }
        else if (key == "TYPE") {
            auto parts = line.split(' ', Qt::SkipEmptyParts).mid(1);
            for (auto& p : parts) types.append(p[0].toLatin1());
        }
        else if (key == "COUNT") {
            auto parts = line.split(' ', Qt::SkipEmptyParts).mid(1);
            for (auto& p : parts) counts.append(p.toInt());
        }
        else if (key == "POINTS") {
            pointCount = line.section(' ',1).toInt();
        }
        else if (key == "DATA") {
            if (line.contains("binary"))
                binary = true;
            break;
        }
    }

    if (!binary || pointCount <= 0)
        return false;

    if (fields.isEmpty() || sizes.isEmpty() || types.isEmpty() || counts.isEmpty())
        return false;

    const int fieldCount = fields.size();
    if (sizes.size() != fieldCount ||
        types.size() != fieldCount ||
        counts.size() != fieldCount)
        return false;

    // ---- Locate indices of needed fields ----
    int idxX = fields.indexOf("x");
    int idxY = fields.indexOf("y");
    int idxZ = fields.indexOf("z");
    int idxIntensity = fields.indexOf("intensity");
    int idxRange = fields.indexOf("range");
    int idxTime = fields.indexOf("time");

    if (idxX < 0 || idxY < 0 || idxZ < 0) {
        qWarning() << "PCD missing x/y/z fields";
        return false;
    }

    const int bytesPerPoint = std::accumulate(
        sizes.begin(), sizes.end(), 0,
        [&](int sum, int s){ return sum + s; });

    QVector<GLPoint> cloud;
    cloud.resize(pointCount);

    // This is necesary to resync file ahead of
    // the QDataStream statement
    qint64 dataPos = header.pos();
    file.seek(dataPos);

    QDataStream bin(&file);
    bin.setByteOrder(QDataStream::LittleEndian);

    QByteArray raw;
    raw.resize(bytesPerPoint);

    // ---- Read points ----
    for (int i = 0; i < pointCount; ++i) {
        if (bin.readRawData(raw.data(), bytesPerPoint) != bytesPerPoint)
            return false;

        const char* ptr = raw.constData();

        float x=0,y=0,z=0,intensity=1.0f,range=0.0f, ftime=0.0f;
        double dtime {0.0};
        int64_t itime64 {0};

        for (int f = 0; f < fieldCount; ++f) {
            const char* fieldPtr = ptr;
            int fieldSize = sizes[f];

            if (f == idxX) memcpy(&x, fieldPtr, sizeof(float));
            if (f == idxY) memcpy(&y, fieldPtr, sizeof(float));
            if (f == idxZ) memcpy(&z, fieldPtr, sizeof(float));
            if (f == idxIntensity) memcpy(&intensity, fieldPtr, sizeof(float));
            if (f == idxRange) memcpy(&range, fieldPtr, sizeof(float));
            // if there is a time field, it may be float, double, int64
            // if it is float or double then it is in seconds
            // if it is int64 then it is in nanoseconds
            // the usage here is for nanoseconds
            // so float and double must be converted to nanoseconds
            if (f == idxTime) {
                if(fieldSize==4) {
                    if(types[f] == 'F') {
                        // this is float
                        memcpy(&ftime, fieldPtr, sizeof(float));
                        itime64 = ftime * 1.0e9; // convert to nanoseconds
                    } else {
                        itime64 = 0;
                    }
                } else if(fieldSize==8) {
                    if(types[f] == 'F') {
                        // this is double
                        memcpy(&dtime, fieldPtr, sizeof(double));
                        itime64 = dtime * 1.0e9; // convert to nanoseconds
                    } else if (types[f]== 'I' || types[f] == 'U') {
                        // this is int64 but read U as signed
                        memcpy(&itime64, fieldPtr, sizeof(int64_t));
                        if(itime64 < 0) itime64 = 0; // just in case
                    } else {
                        // type/precision is not supported
                        itime64 = 0;
                    }
                } else {
                    itime64 = 0;
                }
            }
            ptr += fieldSize * counts[f];
        }

        cloud[i].pos = QVector3D(x,y,z);
        cloud[i].intensity = intensity;
        cloud[i].range = range;
        cloud[i].time = itime64;
    }

    // if window is up the send point cloud
    // to the display
    if (isExposed()) {
        QMetaObject::invokeMethod(
            this,
            [cloud, this]() {
                setPointCloud(cloud);
            },
            Qt::QueuedConnection
            );
    }
    return true;
}

