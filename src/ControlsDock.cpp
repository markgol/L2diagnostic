//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: ControlsDock.cpp
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
#include "ControlsDock.h"
#include "ui_ControlsDock.h"

//--------------------------------------------------------
//
//--------------------------------------------------------
ControlsDock::ControlsDock(QWidget *parent)
    : QDockWidget(parent)
    , ui(new Ui::ControlsDock)
{
    ui->setupUi(this);

    //connections to the buttons to signals
    connect(ui->btnStart, &QPushButton::clicked,
            this, &ControlsDock::startRotationRequested);

    connect(ui->btnStop, &QPushButton::clicked,
            this, &ControlsDock::stopRotationRequested);

    connect(ui->btnReset, &QPushButton::clicked,
            this, &ControlsDock::L2resetRequested);

    connect(ui->btnVersion, &QPushButton::clicked,
            this, &ControlsDock::GetVersionRequested);

    connect(ui->btnConfig, &QPushButton::clicked,
            this, &ControlsDock::ConfigRequested);

    connect(ui->btnL2Connect, &QPushButton::clicked,
            this, &ControlsDock::L2connectRequested);

    connect(ui->btnL2Disconnect, &QPushButton::clicked,
            this, &ControlsDock::L2disconnectRequested);

    setConnectState(false); // L2 is disconnected at start

}

ControlsDock::~ControlsDock()
{
    delete ui;
}

void ControlsDock::setConnectState(bool connected)
{
    // true - L2 connected
    // false - L2 disconnected
    ui->btnL2Connect->setEnabled(!connected);
    ui->btnStart->setEnabled(connected);
    ui->btnStop->setEnabled(connected);
    ui->btnReset->setEnabled(connected);
    ui->btnL2Disconnect->setEnabled(connected);
    ui->btnVersion->setEnabled(connected);

    // this button is always enabled
    ui->btnConfig->setEnabled(true);
}
