#ifndef PCFILEIO_H
#define PCFILEIO_H
//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: PCfileIO.h
//
//  Purpose:
//  basic file i/o for point cloud files
//
//  V2.0.0 RC1  2026-08-01
//              Moved point cloud file i/o to here to genaralize it
//              along with GLPoint and CCPoint structures
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
#include <QVector3D>
#include <QFile>
#include <QTextStream>
#include <QDebug>

// These fields should reflect a PCpoint
// This structure must be packed since it
// is also used to save and read PCD files
;  // this dummy statement is so that known
// bug in clangd doesn't issue warning
//   unterminated ‘#pragma pack (push, …)’ at end of file
#pragma pack(push,1)
// this structure is written to a file
// It is 28 bytes in size.
struct GLPoint
{
    QVector3D pos;
    float intensity;
    float range;
    int64_t time; // changed from float to int64
    // be accurately save time stamp
};

struct CCPoint
{
    float x,y,z;
    float intensity;
    float range;
};

#pragma pack(pop)

bool readPCDfile(const QString& fileName, QVector<GLPoint>* cloud);
#endif // PCFILEIO_H
