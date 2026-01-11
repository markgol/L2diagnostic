//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: PointCloudView.cpp
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
#include "PointCloudView.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

//--------------------------------------------------------
//  PointCloudView uses OpenGL to display a point cloud
//--------------------------------------------------------

//--------------------------------------------------------
//  PointCloudView class constructor
//--------------------------------------------------------
PointCloudView::PointCloudView(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);
}

//--------------------------------------------------------
//  PointCloudView class deconstructor
//--------------------------------------------------------
PointCloudView::~PointCloudView()
{
    // nothing to do
}

//--------------------------------------------------------
//  initializeGL()
//--------------------------------------------------------
void PointCloudView::initializeGL()
{
    initializeOpenGLFunctions();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);

    // --- Shaders ---

    // --- Vertex ---
    m_program.addShaderFromSourceCode(QOpenGLShader::Vertex,
                                      // "#version 330 core\n"
                                      // "layout(location=0) in vec3 a_pos;\n"
                                      // "uniform mat4 u_mvp;\n"
                                      // "void main(){ gl_Position = u_mvp * vec4(a_pos,1.0); gl_PointSize=2.0; }\n"
                "#version 330 core\n"
                "layout(location = 0) in vec3 inPos;\n"
                "layout(location = 1) in vec3 inColor;\n"
                "uniform mat4 mvp;\n"
                "out vec3 fragColor;\n"
                "void main()\n"
                "{\n"
                 "fragColor = inColor;\n"
                 "gl_Position = mvp * vec4(inPos, 1.0);\n"
                 "gl_PointSize = 2.0;\n"
                 "}\n"
                );

    // --- Fragment ---
    m_program.addShaderFromSourceCode(QOpenGLShader::Fragment,
                //                       "#version 330 core\n"
                //                       "out vec4 fragColor;\n"
                //                       "void main(){ fragColor = vec4(0.2,0.9,0.2,1.0); }\n"
                "#version 330 core\n"
                "in vec3 fragColor;\n"
                "out vec4 outColor;\n"
                "void main()\n"
                "{\n"
                "   outColor = vec4(fragColor, 1.0);\n"
                "}\n"
                );

    m_program.link();

    // --- Buffers ---
    m_vao.create();
    m_vbo.create();

    m_vao.bind();
    m_vbo.bind();

    m_vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_vbo.allocate(MAX_POINTS * sizeof(GLPoint));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                          sizeof(GLPoint),
                          reinterpret_cast<void*>(offsetof(GLPoint, pos)));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                          sizeof(GLPoint),
                          reinterpret_cast<void*>(offsetof(GLPoint, color)));

    m_vao.release();
    m_vbo.release();

    m_axisGrid.initialize();
    updateViewMatrix();
}

//--------------------------------------------------------
//  resizeGL()
//--------------------------------------------------------
void PointCloudView::resizeGL(int w, int h)
{
    m_proj.setToIdentity();
    m_proj.perspective(60.0f, float(w) / float(h ? h : 1), 0.1f, 1000.0f);
}

//--------------------------------------------------------
//  paintGL()
//--------------------------------------------------------
void PointCloudView::paintGL()
{
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 mvp = m_proj * m_view;

    // --- Draw grid / axes FIRST ---
    m_axisGrid.render(mvp);

    if (m_pointCount == 0)
        return;

    m_program.bind();
    m_program.setUniformValue("mvp", mvp);

    m_vao.bind();

    // Reassert attributes (cheap and safe)
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    glDrawArrays(GL_POINTS, 0, m_pointCount);

    m_vao.release();
    m_program.release();
}

//--------------------------------------------------------
//  updateViewMatrix()
//--------------------------------------------------------
void PointCloudView::updateViewMatrix()
{
    // Yaw rotates around Z (vertical)
    // Pitch tilts up/down relative to Z
    const float yawRad   = qDegreesToRadians(m_yaw);
    const float pitchRad = qDegreesToRadians(m_pitch);

    QVector3D forward;
    forward.setX(std::cos(pitchRad) * std::sin(yawRad));
    forward.setY(std::cos(pitchRad) * std::cos(yawRad));
    forward.setZ(std::sin(pitchRad));

    forward.normalize();

    QVector3D eye = m_cameraTarget - forward * m_distance;

    m_view.setToIdentity();
    m_view.lookAt(
        eye,
        m_cameraTarget,
        QVector3D(0.0f, 0.0f, 1.0f) // Z is UP
        );
}

//--------------------------------------------------------
// normalizeIntensity
//--------------------------------------------------------
inline float normalizeIntensity(float intensity)
{
    intensity = std::clamp((intensity - INTENSITY_MIN) / (INTENSITY_MAX - INTENSITY_MIN), 0.0f, 1.0f);
    intensity = 0.5; // ??? this is only for debug
    return intensity;
}

//--------------------------------------------------------
// grayscale color
//--------------------------------------------------------
inline QVector3D intensityToColor(float intensity)
{
    float v = normalizeIntensity(intensity);
    return QVector3D(v, v, v);
}

//--------------------------------------------------------
//  setPointCloud()
//--------------------------------------------------------
void PointCloudView::setPointCloud(const QVector<PCpoint>& pcCloud)
{
    // if no data then nothing to display
    if (pcCloud.isEmpty())
        return;

    //pcCloud is the complete point could set to be displayed

    QVector<GLPoint> glPoints;
    glPoints.reserve(pcCloud.size());

    for (const auto& p : pcCloud)
    {
        glPoints.push_back({
            QVector3D(p.x, p.y, p.z),
            // intensityToColor generates the color the point
            intensityToColor(p.intensity)
        });
    }

    makeCurrent();               // REQUIRED (Qt OpenGL rule)

    if(glPoints.size()>MAX_POINTS) {
        qDebug() << "Max points failure";
    }

    m_vbo.bind();
    m_vbo.write(0,
                glPoints.constData(),
                glPoints.size() * sizeof(GLPoint));
    m_vbo.release();

    doneCurrent();

    m_pointCount = glPoints.size();

    // Optional: auto-fit camera once
    // if (!m_cameraInitialized)
    // {
    //     autoFitCameraToCloud(pcCloud);
    //     m_cameraInitialized = true;
    // }

    update(); // trigger repaint
}

//
// ----------------  Camera View  ------------------------
//                  mouse control
//--------------------------------------------------------
//  mousePressEvent()
//--------------------------------------------------------
void PointCloudView::mousePressEvent(QMouseEvent* e)
{
    m_lastMouse = e->pos();
}

//--------------------------------------------------------
//  mouseMoveEvent()
//--------------------------------------------------------
void PointCloudView::mouseMoveEvent(QMouseEvent* e)
{
    QPoint delta = e->pos() - m_lastMouse;
    m_lastMouse = e->pos();

    const Qt::MouseButtons buttons = e->buttons();

    if (buttons == Qt::LeftButton)
    {
        // Orbit
        m_yaw   += delta.x() * 0.5f;
        m_pitch += delta.y() * 0.5f;
        m_pitch = std::clamp(m_pitch, -89.0f, 89.0f);
    }
    else if (buttons == (Qt::LeftButton | Qt::RightButton))
    {
        // Pan
        const float panSpeed = 0.002f * m_distance;

        QVector3D right(
            std::sin(qDegreesToRadians(m_yaw - 90.0f)),
            0.0f,
            std::cos(qDegreesToRadians(m_yaw - 90.0f))
            );

        QVector3D up(0.0f, 1.0f, 0.0f);

        m_cameraTarget -= right * delta.x() * panSpeed;
        m_cameraTarget += up    * delta.y() * panSpeed;
    }

    updateViewMatrix();
    update();
}

//--------------------------------------------------------
//  wheelEvent()
//--------------------------------------------------------
void PointCloudView::wheelEvent(QWheelEvent* e)
{
    m_distance *= std::pow(0.999f, e->angleDelta().y());
    m_distance = std::clamp(m_distance, 1.0f, 500.0f);
    updateViewMatrix();
    update();
}

