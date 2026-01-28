//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: L2lidar.cpp
//
//  Purpose:
//
//  The L2lidar class consists of 2 files:
//      L2lidar.cpp
//      L2lidar.h
//
//  This is to provide necesary software interfaces to control and
//  receive data for the Unitree L2 LiDAR.
//
//  This is to support applications such as diagnotic, point cloud viewer
//  and ROS2 interfaces to the L2
//
//  Background:
//      Unitree provides marginally documented software files
//      in the form:
//          include files (open source)
//          example application files (open source)
//          .a Archive Library (proprietary)
//
//      The source files rely on an Archive library using POSIX I/O
//      No source exists for the archive Library making it diffcult
//      to debug or port usage of the L2 Lidar for other platforms.
//
//      The hardware has 2 mutually exclusive communication interfaces:
//          Ethernet using UDP
//          Serial UART
//
//  Observations:
//      The serial UART on various platforms can be limited in speed and
//      may not operate at the full sensor speed of 64K/sec sample points.
//
//  Current status:
//      Implementation of class verified using UDP interface only.
//      Serial UART impementation is being explored but not included.
//      Working on integration and use of this class in support of ROS2
//      as substitute for Unitree's SDK proprietary archive library.
//
//  Planned offical release will be V0.4.0
//
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
//  V0.3.6  2026-01-26  Added skeleton for Serial UART only
//                      (std QSerialPort has issues with
//                       4M buadrate and 250K byte/sec)
//  V0.3.7  2026-01-26  Primarily documentation updates
//                      Added setting UDP settings on L2
//                      Minor bug corrections
//
//--------------------------------------------------------

//--------------------------------------------------------
// This uses the following Unitree L2 open sources:
//      unitree_lidar_protocol.h
//      unitree_lidar_utilities
// They have been modifed from the original sources
// to correct for errors, missing definitions and
// inconsistencies. These have been minor in most
// instances.
//
// Copyright (c) 2024, Unitree Robotics
// The orignal source can be found at:
//      https://github.com/unitreerobotics/unilidar_sdk2
//      under License: BSD 3-Clause License (see files)
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

//--------------------------------------------------------
//
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
#include <QMessageBox>

//--------------------------------------------------------------------
// L2lidar class constructor
//
//  There is no complex constructor required
//
//--------------------------------------------------------------------
L2lidar::L2lidar(QObject* parent)
    : QObject(parent)
{
    PacketBuffer.clear(); // make sure buffered starts cleared
}


//====================================================================
//  start of received packet handling section
//  This handles the receipt of packets from the L2
//  This includes error checking of the packets
//  When certain packets are decoded a signal is emitted
//  to conneted subscribers
//  The packet types or conditions that emit signals are:
//      3D point cloud packets
//      2D point cloud packets
//      ACK packet
//      VERSION packet
//      Time stamp updates from IMU, point cloud or timestamp packets
//====================================================================

//--------------------------------------------------------
//  readUDPpendingDatagrams()
//
//  It is a Qt non-blocking I/O service called when data
//  is received on the ethernet interface.
//  This is a callback function that receives the UDP
//  datagram packets.
//
//  It is connected to the readyread Qt thread in
//  this ConnectL2() in this class.
//
//  It is not used externally from the class
//
//--------------------------------------------------------
void L2lidar::readUDPpendingDatagrams()
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
//  readUARTpendingDatagrams()
//  This is a callback function that receives the UDP
//  data packets.  It is a Qt non-blocking I/O service called
//  when data in received on the UART interface
//  This is just skeleton fragment, to reserve for
//  future implementation
//--------------------------------------------------------
void L2lidar::readUARTpendingDatagrams()
{
    qDebug() << "UART Not yet implemented\n"
             << "readUARTpendingDatagram()";
    // // only process incoming UPD packets
    // while (L2SerialPort.hasPendingDatagrams()) {
    //     QByteArray datagram;
    //     datagram.resize(static_cast<int>(L2SerialPort.pendingDatagramSize()));
    //     QHostAddress sender;
    //     quint16 senderPort;

    //     // read next Datagram
    //     L2SerialPort.readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

    //     // packets decoder also updates the specific packet ID count totals and packet loss counts
    //     // csv file processing also happens in the decoder
    //     processDatagram(datagram);
    // }
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
//  This means that a processing buffer needs to be appended
//  so that it can span multiple UDP datagrams.
//
//--------------------------------------------------------
void L2lidar::processDatagram(const QByteArray& datagram)
{
    uint64_t Length;
    uint64_t Offset {0};



    // check if additional UDP datagrams needed for complete packet
    if(!IncompletePacket) {
        // if last packet was complete then this is first
        // datagram in new packet
        PacketBuffer.clear();
    }

    PacketBuffer.append(datagram);

    // Get the Datagram size
    Length = PacketBuffer.size();

    // first check if this is too small to be valid UPD frame
    if (Length < sizeof(FrameHeader) + sizeof(FrameTail)) {
        totalPackets_++;
        lostPackets_++;
        return;
    }

    // make sure first UDP packet starts with correct Header
    // since we will need packet size from header to determine
    // if multiple UPD packets are in the datagram.

    do {
        // point to current packet
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

        Length = PacketBuffer.size();
        if(header->packet_size > Length) {
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
    } while(Offset < Length);
}

//--------------------------------------------------------------------
//  3D point cloud packet Decoder
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

    // critical section
    PacketMutex.lock();
    latest3DdataPacket_.header = pkt->header;
    latest3DdataPacket_.data = pkt->data;
    latest3DdataPacket_.tail = pkt->tail;

    latestTimestamp_.data.sec = latest3DdataPacket_.data.info.stamp.sec;
    latestTimestamp_.data.nsec = latest3DdataPacket_.data.info.stamp.nsec;
    PacketMutex.unlock();
    // end of critical section

    //  if(latest3DdataPacket_.data.state.dirty_index != 0.0) {
    //      // report packets that show a no zero dirty_index
    //      // This has been used to determine that any point
    //      // with a range value of less than 1 meter is
    //      // how dirty_index has been calculated
    //      // i.e. any point less than 1 meter from L2 unit are
    //      // not observable
    //      qDebug() << "DirtyIndex: " << latest3DdataPacket_.data.state.dirty_index;
    // }

    // send out notice of latest L2 time stap
    emit timestampReceived();
    // send out notice that a 3D point cloud packet received
    emit PCL3DReceived();
}

//--------------------------------------------------------------------
//  2D point cloud packet Decoder
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

    // critical section
    PacketMutex.lock();

    latest2DdataPacket_.header = pkt->header;
    latest2DdataPacket_.data = pkt->data;
    latest2DdataPacket_.tail = pkt->tail;

    latestTimestamp_.data.sec = latest2DdataPacket_.data.info.stamp.sec;
    latestTimestamp_.data.nsec = latest2DdataPacket_.data.info.stamp.nsec;

    PacketMutex.unlock();
    // end of critical section

    // send out notice of latest L2 time stap
    emit timestampReceived();
    // send out notice that a 2D point cloud packet received
    emit PCL2DReceived();
}

//--------------------------------------------------------------------
//  IMU packet Decoder
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

    // critical section
    PacketMutex.lock();
    latestImu_ = pkt->data;

    latestTimestamp_.data.sec = latestImu_.info.stamp.sec;
    latestTimestamp_.data.nsec = latestImu_.info.stamp.nsec;
    PacketMutex.unlock();
    // end of critical section

    // send out notice of latest L2 time stap
    emit timestampReceived();
    // send out notice that a IMU packet received
    emit imuReceived();
}

//--------------------------------------------------------------------
//  VERSION packet Decoder
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

    // critical section
    PacketMutex.lock();
    latestVersion_ = pkt->data;
    PacketMutex.unlock();
    // end of critical section

    // send out notice that a VERSION packet received
    emit versionReceived();
}
//--------------------------------------------------------------------
// TIMESTAMP Decoder
// Currently no timestamp packet has been observed being sent
// by the L2
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

    // critical section
    PacketMutex.lock();
    latestTimestamp_ = pkt->data;
    PacketMutex.unlock();
    // end of critical section

    // send out notice that a TIMESTAMP packet received
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

    // critical section
    PacketMutex.lock();
    latestACKdata_ = pkt->data;
    PacketMutex.unlock();
    // end of critical section

    // send out notice that a ACK packet received
    emit ackReceived();
}

//--------------------------------------------------------------------
// Raw Packet Handler
// This is called when the packet type is NOT:
//          ACK
//          VERSION
//          IMU
//          2D PC
//          3D PC
//--------------------------------------------------------------------
void L2lidar::handleRaw(uint32_t packetType,
                             const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    // report this in program output only in debug mode
    QString hex = datagram.toHex(' ').toUpper();
    qDebug() << "RAW packet type:" << packetType
             << "size:" << header->packet_size
             << hex.left(128) << "...";

    totalOther_++;
    totalPackets_++;
}
//====================================================================
// end of received packet handling section
//====================================================================

//--------------------------------------------------------------------
//  ClearCounts()
//  Packet count statistics are accumulated
//  This reset the stats
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

//====================================================================
//  L2 commmands
//  These are commands sent to the L2
//  They are used to:
//      change the state of the L2
//      configure L2
//      reset the L2
//      request information from the L2 (such as Version info)
//
//  note: when constructing the commad packets the 'tail' portion
//   of the packet has 2 fields that are undocumented
//   and their actual purpose is unknown
//   These fields have been set to match what has been observed
//   in commands sent by Unitree L2 software.
//   Some experimentation indicates these fields may not be used
//   the L2b ut are included here just in case.
//====================================================================

//--------------------------------------------------------------------
//  LidarStartRotation
//  sends a run command to the L2
//  if the L2 was in standby then it should start scanning
//  it can take >20 seconds to come up to speed and start
//  sending point cloud data
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
    cmd.tail.msg_type_check = 0;
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
//  LidarStopRotation
//  Send a standby command to the L2
//  This causes the L2 to stop the motors and go into low power mode
//  Issues have been seen with not being able to bring the L2
//  out of standby mode without a power cycle
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
    cmd.data.cmd_value = 1;  // 1 puts the L2 into standby

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));
    cmd.tail.msg_type_check = 0;
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
//  LidarReset
//  This causes the L2 to restart (should be equivalent to power cycle)
//  Some workmode changes are only effective after a restart
//  Note:
//   The L2 restart is immediate and does not send and ACK packet
//   that confirms receipt of the command
//--------------------------------------------------------------------
bool L2lidar::LidarReset(void)
{
    // USER_CMD_RESET_TYPE

    // set header
    LidarUserCtrlCmdPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_USER_CMD_PACKET_TYPE,
                    sizeof(LidarUserCtrlCmdPacket));

    // set data
    cmd.data.cmd_type = USER_CMD_RESET_TYPE;
    cmd.data.cmd_value = 1;

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));
    cmd.tail.msg_type_check = 0;
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
//  LidarGetVersion
//  This sends a request to the L2 for a version packet
//  The are 3 posible results:
//      The command is completely ignored (This can happen early
//          in the startup.
//      An ACK packet is sent with the status: ACK_WAIT_ERROR. This
//          occurs when the L2 is not fully initializes.  It will
//          not be fully initialize until the L2 is no longer
//          in standby and the first point cloud packet is sent.
//          If a standby command is sent after the L2 is full initialized
//          then a version packet will still be sent.
//      A VERSION packet is sent (this class will send a signal
//          to any connected subscribers).  An ACK packet will
//          also be sent by the L2 with the status ACK_SUCCESS
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
    cmd.tail.reserve[0] = 0xff; // known value from recorded command
    cmd.tail.reserve[1] = 0x7f; // known value from recorded command
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
//  SetWorkMode
//  This commands only sets the workmode in the L2.  It does not restart
//  the L2.  This must be done separately for certain workmode changes
//  Note:
//      Immediately effective workmode settings that have observed are:
//          Std/Wide FOV
//          IMU disable/enable
//      Effective after restart or reset
//          2D/3D mode
//          serial/UPD mode
//          start automatically or wait for start command after power on
//
//  NOTE: This uses an undocumented command to perform this function
//  It was discovered using Wireshark to see how the
//  Unitree software sends commands
//  This is how the Unitree software sets workmode
//  The define LIDAR_PARAM_WORK_MODE_TYPE
//  was added to be consistent with the documented commands
//
//--------------------------------------------------------------------
bool L2lidar::SetWorkMode(uint32_t mode)
{
    // set header
    LidarWorkModeConfigPacket cmd;
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
//  LidarSetConfig
//  This command does not communicate with the L2
//  It saves the IP setup required to connect to the
//  L2 through UDP.
//  This will be extended to include the serial com port
//  if UART communications when is implemented
//  This must be set before the L2connect() method is used.
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
//  setL2UDPconfig
//  This commands set the UDP configuration used by the L2 to send
//  and receive packets through the Ehternet connection to the L2
//  Note:
//      The L2 does not use DHCP. It is really configured for
//      peer to peer operation.  This means the host Ethernet conenction
//      should also be manually configured.
//      It does not appear that the L2 uses jumbo packets.
//
//      The factory default when it is shipped are:
//          Lidar IP: 192.168.1.62
//          Lidar port: 6101 (this is the port L2 listens for packets)
//          Host IP: 192.168.1.2
//          Host port: 6201 (this is the port the L2 uses to send packets)
//
//      If you change these parameters you will also likely need to change
//      the port settings on your host to match.
//      These settings take effect on the next power cycle of the L2
//
//--------------------------------------------------------------------
bool L2lidar::setL2UDPconfig(QString hostIP, uint32_t hostPort,
                             QString LidarIP, uint32_t LidarPort)
{
    // convert IP string to numbers
    int L2ip[4] {0,0,0,0};
    int Hip[4] {0,0,0,0};

    // make sure valid ip
    // extract Lidar ip
    QStringList parts = LidarIP.split('.');
    bool result;

    L2ip[0] = parts[0].toUInt(&result);
    if(!result) return false;
    if(L2ip[0]<0 || L2ip[0]>255) return false;

    L2ip[1] = parts[1].toUInt(&result);
    if(!result) return false;
    if(L2ip[1]<0 || L2ip[1]>255) return false;

    L2ip[2] = parts[2].toUInt(&result);
    if(!result) return false;
    if(L2ip[2]<0 || L2ip[2]>255) return false;

    L2ip[3] = parts[3].toUInt(&result);
    if(!result) return false;
    if(L2ip[3]<0 || L2ip[3]>255) return false;

    // extract hist ip
    parts = hostIP.split('.');
    Hip[0] = parts[0].toUInt(&result);
    if(!result) return false;
    if(Hip[0]<0 || Hip[0]>255) return false;

    Hip[1] = parts[1].toUInt(&result);
    if(!result) return false;
    if(Hip[1]<0 || Hip[1]>255) return false;

    Hip[2] = parts[2].toUInt(&result);
    if(!result) return false;
    if(Hip[2]<0 || Hip[2]>255) return false;

    Hip[3] = parts[3].toUInt(&result);
    if(!result) return false;
    if(Hip[3]<0 || Hip[3]>255) return false;

    // Set lidar ip address
    LidarIpAddressConfigPacket config;

    config.data.lidar_ip[0] = L2ip[0];
    config.data.lidar_ip[1] = L2ip[1];
    config.data.lidar_ip[2] = L2ip[2];
    config.data.lidar_ip[3] = L2ip[3];

    config.data.user_ip[0] = Hip[0];
    config.data.user_ip[1] = Hip[1];
    config.data.user_ip[2] = Hip[2];
    config.data.user_ip[3] = Hip[3];

    config.data.lidar_port = LidarPort;
    config.data.user_port = hostPort;

    config.data.gateway[0] = 0;
    config.data.gateway[1] = 0;
    config.data.gateway[2] = 0;
    config.data.gateway[3] = 0;

    config.data.subnet_mask[0] = 255;
    config.data.subnet_mask[1] = 255;
    config.data.subnet_mask[2] = 255;
    config.data.subnet_mask[3] = 0;

    setPacketHeader(&config.header, LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE, sizeof(LidarIpAddressConfigPacket));

    // set tail
    config.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &config.data, sizeof(config.data));
    config.tail.msg_type_check = 0x00000000;
    config.tail.reserve[0] = 0x50; // known value from recorded command
    config.tail.reserve[1] = 0xda; // known value from recorded command
    config.tail.tail[0] = 0x00;
    config.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &config, sizeof(LidarIpAddressConfigPacket))) {
        qDebug() << "Set work mode failed";
        return false;
    }

    return true;
}

//--------------------------------------------------------------------
//  setPacketHeader
//  This is just a helper functin to help construct a packet to be
//  sent to the L2.
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
// SendUDPPacket
//--------------------------------------------------------------------
bool L2lidar::SendUDPpacket(uint8_t *Buffer,uint32_t Len)
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
//  SendUARTPacket
//  This is not yet implememted.  It is here just as part of the skeleton
//  required when it is implemented
//--------------------------------------------------------------------
bool L2lidar::SendUARTpacket(uint8_t *Buffer,uint32_t Len)
{
    return false;
}

//--------------------------------------------------------------------
//  SendPacket
//  Sends a packet to the L2 communication interace
//  This does not construct or check the buffer being sent.
//--------------------------------------------------------------------
bool L2lidar::SendPacket(uint8_t *Buffer,uint32_t Len)
{
    bool result;

    if(!UseSerial) {
        result = SendUDPpacket(Buffer,Len);
    } else {
        result = SendUARTpacket(Buffer,Len);
    }

    return result;
}

//====================================================================
//  L2 connect/disconnect
//  These are the equivalent to opening or closing the L2 communications
//  To open communications with the L2 you must:
//      LidarSetCmdConfig()
//      ConnectL2()
//
//  To close you should:
//      DisconnectL2()
//
//  Note: The interface is automatically closed when the L2lidar class
//  is deleted.
//====================================================================

//--------------------------------------------------------------------
//  ConnectL2
//  Currently only UDP Ethernet communications is supported
//  The UART implementation is only a placeholder for future implementation
//  You should call LidarSetCmdConfig() before calling this.
//  If you do not the factory default settings are used.
//
//  NOTE: If you are using WSL2 on Windows 11 then you are also likely
//  to have configured a virtual ethernet port. Both your actual physical
//  Ethernet port and the Virutal Ethernet ports should be manually
//  configured.  You can not set both to be the same IP address.
//--------------------------------------------------------------------
bool L2lidar::ConnectL2()
{
    if(!UseSerial) {
        // Receive packets from L2 UDP
        if (!L2socket.bind(QHostAddress(src_ip),src_port)) {
            qWarning() << "Failed to bind UDP socket to port" << src_port;
            return false;
        }

        // Connect readyRead signal
        connect(&L2socket, &QUdpSocket::readyRead, this, &L2lidar::readUDPpendingDatagrams);
        return true;
    } else {
        // Receive packets from L2 UART
        // problem with QSerialPort support for 4M baudrate
        // need to find alternative UART support package
        // that will support across multiple platforms
        L2serial.setPortName(SerialPort);
        if (!L2serial.open(QIODevice::ReadWrite)) {
            return false;
        }

        L2serial.setBaudRate(QSerialPort::Baud115200);
        L2serial.setDataBits(QSerialPort::Data8);
        L2serial.setParity(QSerialPort::NoParity);
        L2serial.setStopBits(QSerialPort::OneStop);
        L2serial.setFlowControl(QSerialPort::NoFlowControl);

        // Connect readyRead signal
        connect(&L2serial, &QUdpSocket::readyRead, this, &L2lidar::readUARTpendingDatagrams);

        return true;
    }

}

//--------------------------------------------------------------------
// DisconnectL2
//--------------------------------------------------------------------
void L2lidar::DisconnectL2()
{
    if(!UseSerial){
        // close UDP
        L2socket.close();
    } else {
        // close UART
        L2serial.close();
    }
}
