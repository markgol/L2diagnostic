//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: SettingINI.cpp
//
//
//  Purpose:
//      Save and load single INI parameter methods
//
//  V2.0.0 RC1 2026-08-02
//                      Save/Load single individual INI settings
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
#include "settingINI.h"

//--------------------------------------------------------
//  saveINI()
//--------------------------------------------------------
void saveINI(QString group, QString key, QString value)
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    settings.beginGroup(group);
        settings.setValue(key, value);
    settings.endGroup();
}

void saveINI(QString group, QString key, double value)
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    settings.beginGroup(group);
    settings.setValue(key, value);
    settings.endGroup();
}

void saveINI(QString group, QString key, int value)
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    settings.beginGroup(group);
    settings.setValue(key, value);
    settings.endGroup();
}

void saveINI(QString group, QString key, uint value)
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    settings.beginGroup(group);
    settings.setValue(key, value);
    settings.endGroup();
}

//--------------------------------------------------------
//  loadINI()
//--------------------------------------------------------
QString loadINI(QString group, QString key, QString defValue)
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    QString result;
    settings.beginGroup(group);
        result = settings.value(key, defValue).toString();
    settings.endGroup();
    return result;
}

double loadINI(QString group, QString key, double defValue)
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    double result;
    settings.beginGroup(group);
    result = settings.value(key, defValue).toDouble();
    settings.endGroup();
    return result;
}

int loadINI(QString group, QString key, int defValue)
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    int result;
    settings.beginGroup(group);
    result = settings.value(key, defValue).toInt();
    settings.endGroup();
    return result;
}

uint loadINI(QString group, QString key, uint defValue)
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    int result;
    settings.beginGroup(group);
    result = settings.value(key, defValue).toUInt();
    settings.endGroup();

    return result;
}

bool loadINI(QString group, QString key, bool defValue)
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    int result;
    settings.beginGroup(group);
    result = settings.value(key, defValue).toBool();
    settings.endGroup();
    return result;
}
