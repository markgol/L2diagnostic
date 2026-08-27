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
//  V2.0.1  2026-08-24  This is the intial V2.x release
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
#pragma once

//--------------------------------------------------------
//  Qt includes
//--------------------------------------------------------
#include <QSettings>
#include <QStandardPaths>

//--------------------------------------------------------
//  saveINI()
//--------------------------------------------------------
void saveINI(QString group, QString key, QString value);
void saveINI(QString group, QString key, double value);
void saveINI(QString group, QString key, int value);
void saveINI(QString group, QString key, uint value);
void saveINI(QString group, QString key, bool value);

//--------------------------------------------------------
//  loadINI()
//--------------------------------------------------------
QString loadINI(QString group, QString key, QString defValue);
double loadINI(QString group, QString key, double defValue);
int loadINI(QString group, QString key, int defValue);
uint loadINI(QString group, QString key, uint defValue);
bool loadINI(QString group, QString key, bool defValue);
