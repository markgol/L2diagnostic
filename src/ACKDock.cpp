//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: ACKDock.cpp
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
//  of packet.
//
//  V.02.5  2026-01-10  added ACK packet dockable window
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

#include "ACKDock.h"
#include "ui_ACKDock.h"

#include <QString>

//--------------------------------------------------------
// DiagnosticsDock constructor
//--------------------------------------------------------
ACKDock::ACKDock(QWidget* parent)
    : QDockWidget(parent)
    , ui(new Ui::ACKDock)
{
    ui->setupUi(this);
}

//--------------------------------------------------------
// DiagnosticsDock destructor
//--------------------------------------------------------
ACKDock::~ACKDock()
{
    delete ui;
}

//--------------------------------------------------------
//  updateACK
//  signal callback to update ACK window
//--------------------------------------------------------
void ACKDock::updateACK(LidarAckData& ACKdata)
{
    QString Result;

    // The LidarAckData packet is supposed to provide
    // the acknowledgement of a control/query cmd sent to
    // L2 unit.  It primary purpose is to provide success or failure
    // in receiving the control/query cmd.
    //
    // It should be noted that there are discrepancies in the other
    // return values in this packet were it may not provide the
    // correct packet type, cmd type or value.  These values should
    // not be relied upon without verfication of correct reporting
    // specifically for that command
    //
    // In at least one instance no ACK packet will be return.
    // This is for a reset a L2 command

    // packet_type

    switch(ACKdata.packet_type) {
    case LIDAR_USER_CMD_PACKET_TYPE:
        Result = QString("%1 : LIDAR_USER_CMD_PACKET_TYPE").arg(ACKdata.packet_type);
        break;

    case LIDAR_ACK_DATA_PACKET_TYPE:
        Result = QString("%1 : LIDAR_ACK_DATA_PACKET_TYPE").arg(ACKdata.packet_type);
        break;

    case LIDAR_POINT_DATA_PACKET_TYPE:
        Result = QString("%1 : LIDAR_POINT_DATA_PACKET_TYPE").arg(ACKdata.packet_type);
        break;

    case LIDAR_2D_POINT_DATA_PACKET_TYPE:
        Result = QString("%1 : LIDAR_2D_POINT_DATA_PACKET_TYPE").arg(ACKdata.packet_type);
        break;

    case LIDAR_IMU_DATA_PACKET_TYPE:
        Result = QString("%1 : LIDAR_IMU_DATA_PACKET_TYPE").arg(ACKdata.packet_type);
        break;

    case LIDAR_VERSION_PACKET_TYPE:
        Result = QString("%1 : LIDAR_VERSION_PACKET_TYPE").arg(ACKdata.packet_type);
        break;

    case LIDAR_TIME_STAMP_PACKET_TYPE:
        Result = QString("%1 : LIDAR_TIME_STAMP_PACKET_TYPE").arg(ACKdata.packet_type);
        break;

    case LIDAR_WORK_MODE_CONFIG_PACKET_TYPE:
        Result = QString("%1 : LIDAR_WORK_MODE_CONFIG_PACKET_TYPE").arg(ACKdata.packet_type);
        break;

    case LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE:
        Result = QString("%1 : LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE").arg(ACKdata.packet_type);
        break;

    case LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE:
        Result = QString("%1 : LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE").arg(ACKdata.packet_type);
        break;

    case LIDAR_COMMAND_PACKET_TYPE:
        Result = QString("%1 : LIDAR_COMMAND_PACKET_TYPE").arg(ACKdata.packet_type);
        break;

    case LIDAR_PARAM_DATA_PACKET_TYPE:
        Result = QString("%1 : LIDAR_PARAM_DATA_PACKET_TYPE").arg(ACKdata.packet_type);
        break;

    default:
        Result = QString("%1 : undocumented").arg(ACKdata.packet_type);
        break;
    }
    ui->lblPacketTypeValue->setText(Result);

    // cmd_type

    switch(ACKdata.packet_type) {
    case LIDAR_USER_CMD_PACKET_TYPE:
        switch(ACKdata.cmd_type) {
        case USER_CMD_RESET_TYPE:
            Result = QString("%1 : USER_CMD_RESET_TYPE").arg(ACKdata.cmd_type);
            break;

        case USER_CMD_STANDBY_TYPE:
            Result = QString("%1 : USER_CMD_STANDBY_TYPE").arg(ACKdata.cmd_type);
            break;

        case USER_CMD_VERSION_GET:
            Result = QString("%1 : USER_CMD_VERSION_GET").arg(ACKdata.cmd_type);
            break;

        case USER_CMD_LATENCY_TYPE:
            Result = QString("%1 : USER_CMD_LATENCY_TYPE").arg(ACKdata.cmd_type);
            break;

        case USER_CMD_CONFIG_RESET:
            Result = QString("%1 : USER_CMD_CONFIG_RESET").arg(ACKdata.cmd_type);
            break;

        case USER_CMD_CONFIG_GET:
            Result = QString("%1 : USER_CMD_CONFIG_GET").arg(ACKdata.cmd_type);
            break;

        case USER_CMD_CONFIG_AUTO_STANDBY:
            Result = QString("%1 : USER_CMD_CONFIG_AUTO_STANDBY").arg(ACKdata.cmd_type);
            break;

        default:
            Result = QString("%1").arg(ACKdata.cmd_type);
            break;
        }
        break;

    case LIDAR_COMMAND_PACKET_TYPE:
        switch(ACKdata.cmd_type) {
        case CMD_RESET_TYPE:
            Result = QString("%1 : CMD_RESET_TYPE").arg(ACKdata.cmd_type);
            break;

        case CMD_PARAM_SAVE:
            Result = QString("%1 : CMD_PARAM_SAVE").arg(ACKdata.cmd_type);
            break;

        case CMD_PARAM_GET:
            Result = QString("%1 : CMD_PARAM_GET").arg(ACKdata.cmd_type);
            break;

        case CMD_VERSION_GET:
            Result = QString("%1 : CMD_VERSION_GET").arg(ACKdata.cmd_type);
            break;

        case CMD_STANDBY_TYPE:
            Result = QString("%1 : CMD_STANDBY_TYPE").arg(ACKdata.cmd_type);
            break;

        case CMD_LATENCY_TYPE:
            Result = QString("%1 : CMD_LATENCY_TYPE").arg(ACKdata.cmd_type);
            break;

        case CMD_CONFIG_RESET:
            Result = QString("%1 : CMD_CONFIG_RESET").arg(ACKdata.cmd_type);
            break;

        default:
            Result = QString("%1").arg(ACKdata.cmd_type);
            break;
        }
        break;

    default:
        Result = QString("%1").arg(ACKdata.cmd_type);
        break;
    }
    ui->lblCmdTypeValue->setText(Result);

    // cmd_value
    Result = Result.asprintf("%3d",ACKdata.cmd_value);
    ui->lblCmdValueValue->setText(Result);

    // status

    switch(ACKdata.status) {
    case ACK_SUCCESS:
        Result = Result.asprintf("%3d : ACK_SUCCESS",ACKdata.status);
        break;

    case ACK_CRC_ERROR:
        Result = Result.asprintf("%3d : ACK_CRC_ERROR",ACKdata.status);
        break;

    case ACK_HEADER_ERROR:
        Result = Result.asprintf("%3d : ACK_HEADER_ERROR",ACKdata.status);
        break;

    case ACK_BLOCK_ERROR:
        Result = Result.asprintf("%3d : ACK_BLOCK_ERROR",ACKdata.status);
        break;

    case ACK_WAIT_ERROR:
        Result = Result.asprintf("%3d : ACK_WAIT_ERROR",ACKdata.status);
        break;

    default:
        Result = Result.asprintf("%3d : unknown result",ACKdata.status);
        break;
    }
    ui->lblStatusValue->setText(Result);

}
