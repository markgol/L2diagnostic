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
//  V0.4.0  2026-02-06  Added support for both OpenGL Core 3.3
//                      and OpenGLES 3.x
//  V1.0.0  2026-03-28  Offical release
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

#include "AxisGridRenderer.h"
#include <QVector3D>
#include <QVector>
#include <QOpenGLShader>


// ----------------- initialize() -----------------
bool AxisGridRenderer::initialize()
{
    bool isGLES = isOpenGLES;

    reset();

    // Must be called once a valid context is current
    initializeOpenGLFunctions();

    const char* vertexSrc = isGLES ?
        R"(#version 300 es
            precision mediump float;

            layout(location = 0) in vec3 a_pos;
            layout(location = 1) in vec3 a_color;

            uniform mat4 u_mvp;
            out vec3 v_color;

            void main()
            {
                gl_Position = u_mvp * vec4(a_pos, 1.0);
                v_color = a_color;
            }
        )"
    :
        R"(#version 330 core
            layout(location = 0) in vec3 a_pos;
            layout(location = 1) in vec3 a_color;

            uniform mat4 u_mvp;
            out vec3 v_color;

            void main()
            {
                gl_Position = u_mvp * vec4(a_pos, 1.0);
                v_color = a_color;
            }
        )";

    const char* fragmentSrc = isGLES ?
        R"(#version 300 es
            precision mediump float;

            in vec3 v_color;
            out vec4 fragColor;

            void main()
            {
                fragColor = vec4(v_color, 1.0);
            }
        )"
    :
        R"(#version 330 core
            in vec3 v_color;
            out vec4 fragColor;

            void main()
            {
                fragColor = vec4(v_color, 1.0);
            }
        )";

    if(!m_program.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexSrc)) {
        return false;
    }
    if(!m_program.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentSrc)){
        return false;
    }

    if(!m_program.link()) {
        return false;
    }

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

    // Ground grid in X–Y plane (Z = 0)
    for (int i = -gridHalf; i <= gridHalf; ++i)
    {
        float v = i * gridStep;
        float c = (i == 0) ? 0.6f : 0.25f;

        // Lines parallel to X (vary Y)
        vertices.append({ {-gridHalf * gridStep, v, 0}, {c,c,c} });
        vertices.append({ { gridHalf * gridStep, v, 0}, {c,c,c} });

        // Lines parallel to Y (vary X)
        vertices.append({ {v, -gridHalf * gridStep, 0}, {c,c,c} });
        vertices.append({ {v,  gridHalf * gridStep, 0}, {c,c,c} });
    }

    m_vertexCount = vertices.size();

    // ----- VAO + VBO -----
    m_vao.create();
    m_vao.bind();

    m_vbo.create();
    m_vbo.bind();
    m_vbo.allocate(vertices.constData(), (int)(vertices.size() * sizeof(Vertex)));

    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);

    glEnableVertexAttribArray(1); // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(sizeof(QVector3D)));

    m_vbo.release();
    m_vao.release();

    return true;
}

// ----------------- reset() -----------------
void AxisGridRenderer::reset()
{
    if (m_program.isLinked())
        m_program.removeAllShaders();

    if (m_vao.isCreated())
        m_vao.destroy();

    if (m_vbo.isCreated())
        m_vbo.destroy();

    m_vertexCount = 0;
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
