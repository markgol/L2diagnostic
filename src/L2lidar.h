//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: LidarDecoder.h
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
//  V0.1.0  2025-12-27  compilable skeleton created by ChatGPT
//  V0.2.0  2026-01-02  Documentation, start of debugging
//                      CRC32normal() added to unitree_lidar_utilies.h
//                      implementation of LidarDecoder
//  V0.2.1  2026-01-05  Changed LidarDecoder.h and cpp to L2lidar
//                      Changed class name from LidarDecoer to L2lidar
//                      Added USER commands to control L2 lidar
//                      Updated @notes for unitree_lidar_protocols.h
//                      Consolidated all UDP operations into this class
//                      CRC32normal() normal removed from unitree_lidar_utilies.h
//  V.2.2   2026-01-08  Added Mutex access to packet copies
//  V0.3.4  2026-01-23  Changed processingDatagram() to process multiple
//                      UDP datagrams into one L2 Lidar packet
//  V0.3.6  2026-01-26  Added quaternion spatial correction routine
//                      Added Serial UART support
//  V0.3.7  2026-01-28  Documentation updates
//                      Minor bug corrections
//                      Added Set UPD configuration in the L2
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
//
//  No ui (user interface) elements are contained in this class
//
//  class L2lidar
//
//  See the L2lidar.cpp file for class method() documentaion
//
//--------------------------------------------------------

#pragma once

#include <QObject>
#include <QByteArray>
#include <QUdpSocket>
#include <Qhostaddress>
#include <QMutex>
#include <QSerialPort>

#include <cstdint>

// this is required, DO NOT REMOVE
#pragma pack(push, 1)
#include "unitree_lidar_protocolL2.h"
#pragma pack(pop)
// This is typically needed by the parent classes
#include "unitree_lidar_utilitiesL2.h"

//--------------------------------------------------------
//  L2lidar class definitions
//--------------------------------------------------------
class L2lidar : public QObject {
    Q_OBJECT
public:
    explicit L2lidar(QObject* parent = nullptr);

    // Accessors for external data acces in other threads
    // such as a timer based GUI
    const LidarImuData imu() const {
        QMutexLocker locker(&PacketMutex);
        return latestImu_;
    }

    const LidarPointDataPacket Pcl3Dpacket() const {
        QMutexLocker locker(&PacketMutex);
        return latest3DdataPacket_;
    }

    const Lidar2DPointDataPacket Pcl2Dpacket() const {
        QMutexLocker locker(&PacketMutex);
        return latest2DdataPacket_;
    }
    const LidarAckData ack() const {
        QMutexLocker locker(&PacketMutex);
        return latestACKdata_;
    }
    const LidarVersionData version() const {
        QMutexLocker locker(&PacketMutex);
        return latestVersion_;
    }
    const LidarTimeStampData timestamp() const {
        QMutexLocker locker(&PacketMutex);
        return latestTimestamp_;
    }

    // packet stats from L2 (only updates when L2 socket connected)
    // These are not actually critical, only for reporting stats
    const uint64_t totalIMU() const { return totalIMUpackets_;}
    const uint64_t total3D() const { return total3Dpackets_;}
    const uint64_t total2D() const { return total2Dpackets_;}
    const uint64_t totalACK() const { return totalACKpackets_;}
    const uint64_t totalPackets() const { return totalPackets_; }
    const uint64_t lostPackets() const { return lostPackets_; }
    const uint64_t totalOther() const { return lostPackets_; }

    void ClearCounts(); // clears the packet totals

    // L2 commands
    bool LidarStartRotation(void);
    bool LidarStopRotation(void);
    bool LidarReset(void);
    bool LidarGetVersion(void);
    //bool QueryWorkMode(void);  // no method of determing this from L2 has been found
    bool SetWorkMode(uint32_t mode);  // requires reset or power cycle after setting

    // UDP ethernet communications

    // this is only to set the UDP parameters in the class
    // It DOES NOT change the L2 configuration settings
    void LidarSetCmdConfig(QString srcIP, uint32_t srcPort,
                           QString dstIP, uint32_t dstPort);

    // This set the stored UDP configuration on the L2
    // a power cycle is required after this for it to take effect
    bool setL2UDPconfig(QString hostIP, uint32_t hostPort,
                           QString LidarIP, uint32_t LidarPort);

    bool ConnectL2();  // bind to create, bind socket, connect callback for decode
    void DisconnectL2();   // close socket

signals:
    void imuReceived();
    void PCL3DReceived();
    void PCL2DReceived();
    void versionReceived();
    void timestampReceived();
    void ackReceived();

private: // functions
    // Generic Send/receive packets
    bool SendPacket(uint8_t *Buffer,uint32_t Len);
    void processDatagram(const QByteArray& datagram);

    // This is the readyread Qt callback for processing
    // UDP packets that have been recieved
    void readUDPpendingDatagrams();
    bool SendUDPpacket(uint8_t *Buffer,uint32_t Len);

    // UART packets
    bool SendUARTpacket(uint8_t *Buffer,uint32_t Len);
    void readUARTpendingDatagrams();

    // UDP packet decoders
    void decode3D(const QByteArray& datagram, uint64_t Offset);
    void decode2D(const QByteArray& datagram, uint64_t Offset);
    void decodeImu(const QByteArray& datagram, uint64_t Offset);
    void decodeVersion(const QByteArray& datagram, uint64_t Offset);
    void decodeTimestamp(const QByteArray& datagram, uint64_t Offset);
    void decodeAck(const QByteArray& datagram, uint64_t Offset);
    void handleRaw(uint32_t packetType,
                   const QByteArray& datagram, uint64_t Offset);

    // helper functions
    void setPacketHeader(FrameHeader *FrameHeader, uint32_t packet_type,
                         uint32_t packet_size);
    void setPacketTail(FrameHeader *FrameTale);

private: // variables
    // mutex for critical packet access while copying packet
    mutable QMutex  PacketMutex;

    // Communicatopns selector
    bool UseSerial {false}; // false -  use UDP
                            // true - use UART

    // UDP socket
    QUdpSocket L2socket;

    // Serial UART
    QString SerialPort {"com4"};
    // serial port settings are fixed and can not be changed
    // 4M buadrate, 8 bit, even partity, 1 stop, no flow control ?
    QSerialPort L2serial;

    // Packet buffer
    QByteArray PacketBuffer;
    bool IncompletePacket {false};  // if true needs more UDP datagrams
                                    // to complete packet

    // Latest decoded values
    // Accessing these should use mutex lock, PacketMutex
    LidarImuData        latestImu_{};
    LidarVersionData    latestVersion_{};
    LidarTimeStampData  latestTimestamp_{};
    Lidar2DPointDataPacket latest2DdataPacket_{};
    LidarPointDataPacket latest3DdataPacket_{};
    LidarAckData latestACKdata_{};

    // Packet counters, these do not have a mutex lock
    // and should not be relied on for downstream processing
    // They are intended to be only informative
    uint64_t totalPackets_{0};
    uint64_t lostPackets_{0};
    uint64_t totalIMUpackets_{0};
    uint64_t totalACKpackets_{0};
    uint64_t total3Dpackets_{0};
    uint64_t total2Dpackets_{0};
    uint64_t totalOther_{0};

    // QudpSocket parmameters
    // These should only be a reflection of L2
    // UDP ethernet interface. They do not set
    // ethernet configuration on the L2
    QString src_ip {"192.168.1.2"}; // factory default
    QString dst_ip {"192.168.1.62"}; // factory default
    uint32_t src_port {6201}; // factory default
    uint32_t dst_port {6101}; // factory default
};
