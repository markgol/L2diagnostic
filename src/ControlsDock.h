//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: ControlsDock.h
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
//  V.02.6  2026-01-13  added button controls dockable dialog
//
//--------------------------------------------------------
#ifndef CONTROLSDOCK_H
#define CONTROLSDOCK_H

#include <QDockWidget>

QT_BEGIN_NAMESPACE
namespace Ui { class ControlsDock; }
QT_END_NAMESPACE

class ControlsDock : public QDockWidget
{
    Q_OBJECT

public:
    explicit ControlsDock(QWidget *parent = nullptr);
    ~ControlsDock();

    // UI state control (called by MainWindow)
    void setConnectState(bool connected);  // true - L2 connected
                                        // false - L2 disconnected

signals:
    void startRotationRequested();
    void stopRotationRequested();
    void L2resetRequested();
    void GetVersionRequested();
    void ConfigRequested();
    void L2connectRequested();
    void L2disconnectRequested();

private:
    Ui::ControlsDock *ui;
};

#endif // CONTROLSDOCK_H
