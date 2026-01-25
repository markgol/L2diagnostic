//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: LidarDecoder.cpp
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
//                      Changed class name from LidarDecoder to L2lidar
//                      Added L2 start rotation, stop rotation,
//                              reset and get Version info
//                      Added crc32() check on incoming packets
//                      Added crc32() calculation on outgoing packets
//                      Moved all socket calls to this class
//                      removed CRC32normal()from unitree_lidar_utilies.h
//                      Full PCL packet is saved because the
//                          unitree utility to convert PCL cloud
//                          requires it
//  V0.3.4  2026-01-23  Changed processingDatagram() to process multiple
//                      UDP datagrams into one L2 Lidar packet
//  V0.3.5  2026-01-24  Correction to last2D packet
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
//  There are 2 interface sides to the L2 Lidar
//  The packets sent from the L2 are recieved on different port
//  then the command packets that are sent to the L2
//
//  No ui (user interface) elements are contained in this class
//
//  class L2lidar
//
//--------------------------------------------------------

#include "L2lidar.h"
#include <QDebug>

//--------------------------------------------------------------------
// L2lidar class constructor
//--------------------------------------------------------------------
L2lidar::L2lidar(QObject* parent)
    : QObject(parent)
{
    PacketBuffer.clear(); // make sure buffered starts cleared
}

//--------------------------------------------------------
//  readPendingDatagrams()
//  This is a callback function that receives the UDP
//  data packets.  It is a Qt non-blocking I/O service called
//  when data in received on the ehternet interface
//--------------------------------------------------------
void L2lidar::readPendingDatagrams()
{
    // only process incoming UPD packets
    while (L2socket.hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(static_cast<int>(L2socket.pendingDatagramSize()));
        QHostAddress sender;
        quint16 senderPort;

        // read next Datagram
        L2socket.readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        // packets decoder also updates the specific packet ID count totals and packet loss counts
        // csv file processing also happens in the decoder
        processDatagram(datagram);
    }
}

//--------------------------------------------------------
//  processDatagram()
//  processing the payload package from the datagram
//  note: more than one packet may be in the datagram
//  Each packet must be processed.
//
//  processing requirements:
//  All but one of the L2 Lidar packets can be contained
//  in one UDP datagram.  Some UDP datagrams contain more
//  than one L2 Lidar packet.
//  Only the L2 Lidar packet for 2D scan data spans multiple
//  UDP datagrams (4 datagrams)
//
//  This means that a processing buffer needs to be allocated
//  that can span multiple UDP datagrams.
//
//--------------------------------------------------------
void L2lidar::processDatagram(const QByteArray& datagram)
{
    uint64_t DatagramLength;
    uint64_t Offset {0};



    if(!IncompletePacket) {
        PacketBuffer.clear();
    }
    PacketBuffer.append(datagram);

    // Get the Datagram size
    // Will need to determine UDP packet size using
    DatagramLength = PacketBuffer.size();

    // first check if this is too small to be valid UPD frame
    if (DatagramLength < sizeof(FrameHeader) + sizeof(FrameTail)) {
        totalPackets_++;
        lostPackets_++;
        return;
    }

    // make sure first UDP packet starts with correct Header
    // since we will need packet size from header to determine
    // if multiple UPD packets are in the datagram.

    do {
        // point to current UDP packet
        const auto* header =
            reinterpret_cast<const FrameHeader*>(PacketBuffer.constData() + Offset);

        // check header frame correct format
        if (header->header[0] != FRAME_HEADER_ARRAY_0 ||
            header->header[1] != FRAME_HEADER_ARRAY_1 ||
            header->header[2] != FRAME_HEADER_ARRAY_2 ||
            header->header[3] != FRAME_HEADER_ARRAY_3) {

            totalPackets_++;
            lostPackets_++;
            return;
        }

        if(header->packet_size > PacketBuffer.size()) {
            // The PacketBuffer does not have enough data for this
            // L2 Lidar packet
            // This will add the next UDP datagram
            IncompletePacket = true;
            return;
        }

        // enough data in packet buffer to process packet
        IncompletePacket = false;

        // Verify Tail and CRC
        auto* tail = reinterpret_cast<const FrameTail*>(
                        PacketBuffer.constData() + ((Offset + header->packet_size) - sizeof(FrameTail)));
	
	    // check tail frame correct format
        // 2D packets do not have consistent tail code

        bool GoodTailCode =
            ((tail->tail[0] == FRAME_TAIL_ARRAY_0) &&
             (tail->tail[1] == FRAME_TAIL_ARRAY_1));

        if (!GoodTailCode) {
            totalPackets_++;
            lostPackets_++;
            return;
        }

        // perform crc just on the data without hreader or tail
        uint32_t crc = unilidar_sdk2::crc32(reinterpret_cast<const uint8_t*>(PacketBuffer.constData()+Offset+sizeof(FrameHeader)),
                               header->packet_size - (sizeof(FrameHeader) + sizeof(FrameTail)));

        if (crc != tail->crc32) {
            totalPackets_++;
            lostPackets_++;
            return;
        }

        switch (header->packet_type) {
            case LIDAR_IMU_DATA_PACKET_TYPE:
                decodeImu(PacketBuffer,Offset);
                Offset += header->packet_size;
                break;

            case LIDAR_POINT_DATA_PACKET_TYPE:
                decode3D(PacketBuffer,Offset);
                Offset += header->packet_size;
                break;

            case LIDAR_2D_POINT_DATA_PACKET_TYPE:
                decode2D(PacketBuffer,Offset);
                Offset += header->packet_size;
                break;

            case LIDAR_VERSION_PACKET_TYPE:
                decodeVersion(PacketBuffer,Offset);
                Offset += header->packet_size;
                break;

            case LIDAR_TIME_STAMP_PACKET_TYPE:
                decodeTimestamp(PacketBuffer,Offset);
                Offset += header->packet_size;
                break;

            case LIDAR_ACK_DATA_PACKET_TYPE:
                decodeAck(PacketBuffer,Offset);
                Offset += header->packet_size;
                break;

            default:
                handleRaw(header->packet_type, PacketBuffer,Offset);
                Offset += header->packet_size;
                break;
        }
    } while(Offset < DatagramLength);
}

//--------------------------------------------------------------------
// ClearCounts()
//--------------------------------------------------------------------
void L2lidar::ClearCounts()
{
    totalPackets_ = 0;
    lostPackets_ = 0;
    totalIMUpackets_ = 0;
    totalACKpackets_ = 0;
    total3Dpackets_ = 0;
    total2Dpackets_ = 0;
    lostPackets_ = 0;
}

//--------------------------------------------------------------------
// 3D point cloud Decoder (verified 80-byte packet)
//--------------------------------------------------------------------
void L2lidar::decode3D(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarPointDataPacket)) {
        lostPackets_++;
        return;
    }

    totalPackets_++;
    total3Dpackets_++;
    const auto* pkt =
        reinterpret_cast<const LidarPointDataPacket*>(datagram.constData()+Offset);

    PacketMutex.lock();
    latest3DdataPacket_.header = pkt->header;
    latest3DdataPacket_.data = pkt->data;
    latest3DdataPacket_.tail = pkt->tail;

    latestTimestamp_.data.sec = latest3DdataPacket_.data.info.stamp.sec;
    latestTimestamp_.data.nsec = latest3DdataPacket_.data.info.stamp.nsec;
    PacketMutex.unlock();

    emit timestampReceived();
    emit PCL3DReceived();
}

//--------------------------------------------------------------------
// 2D point cloud Decoder (verified 80-byte packet)
//--------------------------------------------------------------------
void L2lidar::decode2D(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(Lidar2DPointDataPacket)) {
        lostPackets_++;
        return;
    }
    totalPackets_++;
    total2Dpackets_++;
    const auto* pkt =
        reinterpret_cast<const Lidar2DPointDataPacket*>(datagram.constData()+Offset);

    PacketMutex.lock();

    latest2DdataPacket_.header = pkt->header;
    latest2DdataPacket_.data = pkt->data;
    latest2DdataPacket_.tail = pkt->tail;

    latestTimestamp_.data.sec = latest2DdataPacket_.data.info.stamp.sec;
    latestTimestamp_.data.nsec = latest2DdataPacket_.data.info.stamp.nsec;

    PacketMutex.unlock();

    emit timestampReceived();
    emit PCL2DReceived();
}

//--------------------------------------------------------------------
// IMU Decoder (verified 80-byte packet)
//--------------------------------------------------------------------
void L2lidar::decodeImu(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarImuDataPacket)) {
        lostPackets_++;
        return;
    }
    totalPackets_++;
    totalIMUpackets_++;
    const auto* pkt =
        reinterpret_cast<const LidarImuDataPacket*>(datagram.constData()+Offset);

    PacketMutex.lock();
    latestImu_ = pkt->data;

    latestTimestamp_.data.sec = latestImu_.info.stamp.sec;
    latestTimestamp_.data.nsec = latestImu_.info.stamp.nsec;
    PacketMutex.unlock();

    emit timestampReceived();
    emit imuReceived();
}

//--------------------------------------------------------------------
// VERSION Decoder
//--------------------------------------------------------------------
void L2lidar::decodeVersion(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarVersionDataPacket)) {
        lostPackets_++;
        return;
    }
    totalOther_++;
    totalPackets_++;

    const auto* pkt =
        reinterpret_cast<const LidarVersionDataPacket*>(datagram.constData()+Offset);

    PacketMutex.lock();
    latestVersion_ = pkt->data;
    PacketMutex.unlock();

    emit versionReceived();
}
//--------------------------------------------------------------------
// TIMESTAMP Decoder
//--------------------------------------------------------------------
void L2lidar::decodeTimestamp(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarTimeStampPacket)) {
        lostPackets_++;
        return;
    }

    totalOther_++;
    totalPackets_++;

    const auto* pkt =
        reinterpret_cast<const LidarTimeStampPacket*>(datagram.constData()+Offset);

    PacketMutex.lock();
    latestTimestamp_ = pkt->data;
    PacketMutex.unlock();

    emit timestampReceived();
}

//--------------------------------------------------------------------
// ACK Decoder
//--------------------------------------------------------------------
void L2lidar::decodeAck(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarAckDataPacket)) {
        lostPackets_++;
        return;
    }
    totalPackets_++;
    totalACKpackets_++;

    const auto* pkt =
        reinterpret_cast<const LidarAckDataPacket*>(datagram.constData()+Offset);

    PacketMutex.lock();
    latestACKdata_ = pkt->data;
    PacketMutex.unlock();

    emit ackReceived();
}

//--------------------------------------------------------------------
// Raw Packet Handler (CSV / Hex Dump)
//--------------------------------------------------------------------
void L2lidar::handleRaw(uint32_t packetType,
                             const QByteArray& datagram, uint64_t Offset)
{
    // ??? This will need updating
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    QString hex = datagram.toHex(' ').toUpper(); // ??? TBD
    qDebug() << "RAW packet type:" << packetType
             << "size:" << header->packet_size
             << hex.left(128) << "...";

    totalOther_++;
    totalPackets_++;

}

//====================================================================
// L2 commmands
//====================================================================

//--------------------------------------------------------------------
// LidarStartRotation
//--------------------------------------------------------------------
bool L2lidar::LidarStartRotation(void)
{
    // USER_CMD_STANDBY_TYPE, value = 0

    // set header
    LidarUserCtrlCmdPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_USER_CMD_PACKET_TYPE,
                    sizeof(LidarUserCtrlCmdPacket));

    // set data
    cmd.data.cmd_type = USER_CMD_STANDBY_TYPE;
    cmd.data.cmd_value = 0;
    
    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));
    cmd.tail.msg_type_check = 0; // 0xf7d55f5b known value from recorded command
    cmd.tail.reserve[0] = 0xff; // known value from recorded command
    cmd.tail.reserve[1] = 0x7f; // known value from recorded command
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarUserCtrlCmdPacket))) {
        qDebug() << "Start cmd failed";
        return false;
    }

    return true;
}

//--------------------------------------------------------------------
// LidarStopRotation
//--------------------------------------------------------------------
bool L2lidar::LidarStopRotation(void)
{
    // USER_CMD_STANDBY_TYPE, value = 1

    // set header
    LidarUserCtrlCmdPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_USER_CMD_PACKET_TYPE,
                    sizeof(LidarUserCtrlCmdPacket));

    // set data
    cmd.data.cmd_type = USER_CMD_STANDBY_TYPE;
    cmd.data.cmd_value = 1;

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));
    cmd.tail.msg_type_check = 0; // 0xf7d55f5b known value from recorded command
    cmd.tail.reserve[0] = 0xff; // known value from recorded command
    cmd.tail.reserve[1] = 0x7f; // known value from recorded command
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarUserCtrlCmdPacket))) {
        qDebug() << "Stop cmd failed";
        return false;
    }

    return true;
}

//--------------------------------------------------------------------
// LidarReset
//--------------------------------------------------------------------
bool L2lidar::LidarReset(void)
{
    // USER_CMD_RESET_TYPE
    // NOTE: L2 Reset is immediate and does not send an ACK

    // set header
    LidarUserCtrlCmdPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_USER_CMD_PACKET_TYPE,
                    sizeof(LidarUserCtrlCmdPacket));

    // set data
    cmd.data.cmd_type = USER_CMD_RESET_TYPE;
    cmd.data.cmd_value = 1;

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));
    cmd.tail.msg_type_check = 0; //0xf790ed93 known value from recorded command
    cmd.tail.reserve[0] = 0xff; // known value from recorded command
    cmd.tail.reserve[1] = 0x7f; // known value from recorded command
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarUserCtrlCmdPacket))) {
        qDebug() << "Rest cmd failed";
        return false;
    }


   return true;
}

//--------------------------------------------------------------------
// LidarGetVersion
//--------------------------------------------------------------------
bool L2lidar::LidarGetVersion(void)
{
    // USER_CMD_VERSION_GET

    // set header
    LidarUserCtrlCmdPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_USER_CMD_PACKET_TYPE,
                    sizeof(LidarUserCtrlCmdPacket));

    // set data
    cmd.data.cmd_type = USER_CMD_VERSION_GET;
    cmd.data.cmd_value = 0;  // value guess

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));

    cmd.tail.msg_type_check = 0;
    cmd.tail.reserve[0] = 0xff;
    cmd.tail.reserve[1] = 0x7f;
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarUserCtrlCmdPacket))) {
        qDebug() << "Get Version cmd failed";
        return false;
    }

    return true;
}

//--------------------------------------------------------------------
// SetWorkMode
//--------------------------------------------------------------------
bool L2lidar::SetWorkMode(uint32_t mode)
{
    // set header
    LidarWorkModeConfigPacket cmd;
    //  Note: this is an undocumented command
    //  It was discovered using Wireshark to see how the
    //  Unitree software sends commands
    //  This is how the Unitree software sets workmode
    //  The define LIDAR_PARAM_WORK_MODE_TYPE
    //  was added to be inline with the documented commands
    setPacketHeader(&cmd.header,LIDAR_PARAM_WORK_MODE_TYPE,
                    sizeof(LidarWorkModeConfigPacket));

    // set data
    cmd.data.mode = mode;

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));
    cmd.tail.msg_type_check = 0x00007fff;
    cmd.tail.reserve[0] = 0x5b; // known value from recorded command
    cmd.tail.reserve[1] = 0x5f; // known value from recorded command
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarWorkModeConfigPacket))) {
        qDebug() << "Set work mode failed";
        return false;
    }

    return true;
}

//--------------------------------------------------------------------
// LidarSetConfig
//--------------------------------------------------------------------
void L2lidar::LidarSetCmdConfig(QString srcIP, uint32_t srcPort,
                                QString dstIP, uint32_t dstPort)
{
    src_ip = srcIP;
    src_port = srcPort;
    dst_ip = dstIP;
    dst_port = dstPort;
    return;
}

//--------------------------------------------------------------------
// setPacketHeader
//--------------------------------------------------------------------
void L2lidar::setPacketHeader(FrameHeader *FrameHeader, uint32_t packet_type, uint32_t packet_size)
{
    FrameHeader->header[0] = 0x55;
    FrameHeader->header[1] = 0xaa;
    FrameHeader->header[2] = 0x05;
    FrameHeader->header[3] = 0x0a;

    FrameHeader->packet_type = packet_type;
    FrameHeader->packet_size = packet_size;
    return;
}

//--------------------------------------------------------------------
// SendPacket
//--------------------------------------------------------------------
bool L2lidar::SendPacket(uint8_t *Buffer,uint32_t Len)
{
    QByteArray byteArray(reinterpret_cast<const char*>(Buffer), Len);

    if(L2socket.state()!= QAbstractSocket::BoundState) {
        qDebug() << "Socket not open";
        return false;
    }
    // write to target ip:port
    qint64 bytesWritten = L2socket.writeDatagram(byteArray,QHostAddress(dst_ip),dst_port);
    if( bytesWritten == -1) {
        qDebug() << "Error sending datagram:" << L2socket.errorString();
        return false;
    }
    //qDebug() << "Sent" << bytesWritten << "bytes to" << dstip << ":" << dstport;
    return true;
}

//--------------------------------------------------------------------
// ConnectL2
//--------------------------------------------------------------------
bool L2lidar::ConnectL2()
{
    // Receive packets from L2
    // QHostAddress(srcip),srcport
    // QHostAddress::AnyIPv4, src_port, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)
    if (!L2socket.bind(QHostAddress(src_ip),src_port)) {
        qWarning() << "Failed to bind UDP socket to port" << src_port;
        return false;
    }

    // Connect readyRead signal
    connect(&L2socket, &QUdpSocket::readyRead, this, &L2lidar::readPendingDatagrams);
    return true;
}

//--------------------------------------------------------------------
// DisconnectL2
//--------------------------------------------------------------------
void L2lidar::DisconnectL2()
{
    L2socket.close();
}
