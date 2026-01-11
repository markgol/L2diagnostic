//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: AxisGridRenderer.cpp
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
#include "AxisGridRenderer.h"
#include <QVector3D>
#include <QVector>
#include <QOpenGLShader>

// ----------------- initialize() -----------------
void AxisGridRenderer::initialize()
{
    // Must be called once a valid context is current
    initializeOpenGLFunctions();

    // ----- Shader sources -----
    static const char* vertexSrc = R"(#version 330 core
        layout(location = 0) in vec3 a_pos;
        layout(location = 1) in vec3 a_color;
        uniform mat4 u_mvp;
        out vec3 v_color;
        void main()
        {
            gl_Position = u_mvp * vec4(a_pos, 1.0);
            v_color = a_color;
        })";

    static const char* fragmentSrc = R"(#version 330 core
        in vec3 v_color;
        out vec4 fragColor;
        void main()
        {
            fragColor = vec4(v_color, 1.0);
        })";

    m_program.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexSrc);
    m_program.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentSrc);
    m_program.link();

    // ----- Build vertex data -----
    struct Vertex { QVector3D pos; QVector3D color; };
    QVector<Vertex> vertices;

    constexpr float axisLen = 2.0f;
    constexpr int gridHalf = 20;
    constexpr float gridStep = 1.0f;

    // X axis (red)
    vertices.append({ {0,0,0}, {1,0,0} });
    vertices.append({ {axisLen,0,0}, {1,0,0} });

    // Y axis (green)
    vertices.append({ {0,0,0}, {0,1,0} });
    vertices.append({ {0,axisLen,0}, {0,1,0} });

    // Z axis (blue)
    vertices.append({ {0,0,0}, {0,0,1} });
    vertices.append({ {0,0,axisLen}, {0,0,1} });

    // Ground grid (gray lines)
    for(int i=-gridHalf; i<=gridHalf; ++i)
    {
        float v = i * gridStep;
        float c = (i==0)?0.6f:0.25f;

        // lines parallel to X
        vertices.append({ {-gridHalf*gridStep, v, 0}, {c,c,c} });
        vertices.append({ { gridHalf*gridStep, v, 0}, {c,c,c} });

        // lines parallel to Y
        vertices.append({ {v, -gridHalf*gridStep,0}, {c,c,c} });
        vertices.append({ {v,  gridHalf*gridStep,0}, {c,c,c} });
    }

    m_vertexCount = vertices.size();

    // ----- VAO + VBO -----
    m_vao.create();
    m_vao.bind();

    m_vbo.create();
    m_vbo.bind();
    m_vbo.allocate(vertices.constData(), vertices.size() * sizeof(Vertex));

    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);

    glEnableVertexAttribArray(1); // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(sizeof(QVector3D)));

    m_vbo.release();
    m_vao.release();
}

// ----------------- render() -----------------
void AxisGridRenderer::render(const QMatrix4x4& mvp)
{
    if(!m_program.isLinked()) return;

    m_program.bind();
    m_program.setUniformValue("u_mvp", mvp);

    m_vao.bind();
    glDrawArrays(GL_LINES, 0, m_vertexCount);
    m_vao.release();

    m_program.release();
}
