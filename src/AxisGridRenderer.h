//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: AxisGridRenderer.h
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
#pragma once

#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QMatrix4x4>

//--------------------------------------------------------
//  OpenGL renderer
//--------------------------------------------------------

class AxisGridRenderer : protected QOpenGLFunctions
{
public:
    void initialize();
    void render(const QMatrix4x4& mvp);

private:
    QOpenGLShaderProgram m_program;   // ✅ MUST be this type
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_vbo{ QOpenGLBuffer::VertexBuffer };

    int m_vertexCount = 0;
};
