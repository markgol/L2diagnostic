//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: PCfileIO.cpp
//
//  Purpose:
//  basic file i/o for point cloud files
//
//  V2.0.0 RC1  2026-08-01
//              Moved point cloud file i/o to here to genaralize it.
//  V2.0.1  2026-08-24  This is the intial V2.x release
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
#include "PCfileIO.h"

//--------------------------------------------------------
//  readPCDfile
//--------------------------------------------------------
bool readPCDfile(const QString& fileName, QVector<GLPoint>* cloud)
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

    //QVector<GLPoint> cloud;
    cloud->resize(pointCount);

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

        (*cloud)[i].pos = QVector3D(x,y,z);
        (*cloud)[i].intensity = intensity;
        (*cloud)[i].range = range;
        if(idxTime>0) {
            // time data is in point cloud file
            (*cloud)[i].time = itime64;
        } else {
            // no time data in the point cloud file
            (*cloud)[i].time = 0;
        }
    }

    return true;
}
