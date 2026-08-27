//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: CalibrationDock.cpp
//
//  Purpose:
//  Control calibration process for the L2.
//  Set RangeScale,RangeBias toestimate frist order range calibration
//  Flatten point cloud over small angle to obtain range nonlinearity
//  Compute the line spread function for range versus radial angle
//  Best fit line to line spread function
//  Compute point range corrections from line spread function
//  Compute piecewise cubic spline to fit total range
//  Test calibration
//  Save calibration
//
//  Background:
//  The Unitree L2 4D LiDAR has intrinsic non-linearities in
//  it reported range.  The conversion model for az,el, range provided
//  by Unitree is only first order.
//  The distortion caused by the range non-linearity corrupts
//  the ICP mathcing typically used in odometry.
//  The L2 has this distortion is still present after removing
//  gyroscopic induced coherent vibration by proper mounting
//  and dampening of the L2.
//  The distortion is measureable and repeatable at all slow scan angles.
//  This allows correction of the range value before it is converted
//  into a x,y,z point poistion.
//
//  V2.0.0 RC1 2026-07-31  Added calibration model for the L2
//  V2.0.1  2026-08-24  This is the intial V2.x release
//                      Implemented application of the alpha angle LUT
//                      Added Alpha Angle step size override
//                      Some cleanup of the UI and GUI interactions
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

#include "CalibrationDock.h"
#include "settingINI.h"
#include "Stage1CalRangeDialog.h"
#include "Stage2CalRangeDialog.h"
#include "Stage3CalRangeDialog.h"
#include "Stage4CalRangeClasses.h"

#include <QCloseEvent>
#include <QMessageBox>
#include <QInputDialog>

//--------------------------------------------------------
//  ControlsDock constructor
//--------------------------------------------------------
CalibrationDock::CalibrationDock(L2lidar& lidar, QWidget *parent)
    : QDockWidget(parent),
    ml2lidar(lidar),
    ui(new Ui::CalibrationDock)
{
    ui->setupUi(this);

    //connections to the buttons to signals

    // switch to diagnostics mode
    connect(ui->btnDiagnose, &QPushButton::clicked,
             this, &CalibrationDock::DiagnosticMode);

    // l2 connect button
    connect(ui->btnL2Connect, &QPushButton::clicked,
            this, &CalibrationDock::L2connectRequested);

    // l2 disconnect button
    connect(ui->btnL2Disconnect, &QPushButton::clicked,
            this, &CalibrationDock::L2disconnectRequested);

    // clear point cloud button
    connect(ui->btnClearDisplay, &QPushButton::clicked,
            this, &CalibrationDock::ClearPCwindowRequested);

    //  SavePC clock button
    connect(ui->btnSavePC, &QPushButton::clicked,
            this, &CalibrationDock::SavePC);

    //  LoadPC button
    connect(ui->btnLoadPC, &QPushButton::clicked,
            this, &CalibrationDock::LoadPC);

    //  Browse calibration file button
    connect(ui->btnBrowseCalFile, &QPushButton::clicked,
            this, &CalibrationDock::BrowseCalFile);

    //  Load calibration button
    connect(ui->btnLoadRangeCal, &QPushButton::clicked,
            this, &CalibrationDock::LoadRangeCal);

    //  Clear correction button
    connect(ui->btnClear, &QPushButton::clicked,
            this, &CalibrationDock::ClearRangeCorrection);

    //  Init settings for acquistion button
    connect(ui->btnInit, &QPushButton::clicked,
            this, &CalibrationDock::Init4Cal);

    //  EnableRangeCal checkbox clicked
    connect(ui->cbEnableRangeCal, &QCheckBox::clicked,
            this, &CalibrationDock::RangeCorrectionCheckbox);

    //  Range bias spin
    connect(ui->spinRangeBias, &QSpinBox::valueChanged,
            this, &CalibrationDock::RangeBiasChanged);

    //  Range scale spin
    connect(ui->spinRangeScale, &QDoubleSpinBox::valueChanged,
            this, &CalibrationDock::RangeScaleChanged);

    //  Min Range spin
    connect(ui->spinMinRange, &QSpinBox::valueChanged,
            this, &CalibrationDock::MinRangeChanged);

    //  Max Range spin
    connect(ui->spinMaxRange, &QSpinBox::valueChanged,
            this, &CalibrationDock::MaxRangeChanged);

    //  Theta angle bias spin
    connect(ui->spinThetaBias, &QDoubleSpinBox::valueChanged,
            this, &CalibrationDock::ThetaChanged);

    //  Alpha angle bias spin
    connect(ui->spinAlphaAngle, &QDoubleSpinBox::valueChanged,
            this, &CalibrationDock::AlphaChanged);

    //  Alpha angle step size spin
    connect(ui->spinAlphaStepSize, &QDoubleSpinBox::valueChanged,
            this, &CalibrationDock::AlphaSizeChanged);

    //  Beta angle spin
    connect(ui->spinBetaAngle, &QDoubleSpinBox::valueChanged,
            this, &CalibrationDock::BetaChanged);

    //  Xi angle spin
    connect(ui->spinXiAngle, &QDoubleSpinBox::valueChanged,
            this, &CalibrationDock::XiChanged);

    //CalOVRenable button
    connect(ui->cbCalOveride, &QCheckBox::clicked,
            this, &CalibrationDock::CalOVRenable);

    //  EnableAlphaAngle checkbox clicked
    connect(ui->cbEnableAlphaLUT, &QCheckBox::clicked,
            this, &CalibrationDock::AlphaAngleCheckbox);

    // Range Correction stage buttons
    ui->btnStage1->setEnabled(false);
    ui->btnStage2->setEnabled(false);
    ui->btnStage3->setEnabled(false);
    ui->btnStage4->setEnabled(false);
    ui->btnStage5->setEnabled(false);

    // The range correction stage buttons and checkbox
    connect(ui->btnStage1, &QPushButton::clicked,
            this, &CalibrationDock::Stage1Dialog);

    connect(ui->btnStage2, &QPushButton::clicked,
            this, &CalibrationDock::Stage2Dialog);

    connect(ui->btnStage3, &QPushButton::clicked,
            this, &CalibrationDock::Stage3Dialog);

    connect(ui->btnStage4, &QPushButton::clicked,
            this, &CalibrationDock::ProcessStage4);

    connect(ui->btnStage5, &QPushButton::clicked,
            this, &CalibrationDock::ProcessStage5);
}

//--------------------------------------------------------
//  ControlsDock destructor
//--------------------------------------------------------
CalibrationDock::~CalibrationDock()
{
    if(mstage2) {
        mstage2->close();
        delete mstage2;
    }
    if(mstage3) {
        mstage3->close();
        delete mstage3;
    }
    delete ui;
}

//--------------------------------------------------------
//  setConnectState
//  this disables/enables various button depending on
//  wether the L2 is connected
//--------------------------------------------------------
void CalibrationDock::setConnectState(bool connected)
{
    // true - L2 connected
    // false - L2 disconnected
    ui->btnL2Connect->setEnabled(!connected);
    ui->btnL2Disconnect->setEnabled(connected);
}

//--------------------------------------------------------
//  closeEvent
//--------------------------------------------------------
void CalibrationDock::closeEvent(QCloseEvent* event)
{
    event->accept();
}

//--------------------------------------------------------
//  RangeCalGUI
//--------------------------------------------------------
void CalibrationDock::RangeCalGUI(bool clear, bool visible)
{
    emit CalGUIrequest(clear, visible);
}

//--------------------------------------------------------
//  EnableRangeCorrectionChanged
//--------------------------------------------------------
void CalibrationDock::RangeCorrectionCheckbox()
{
    emit EnableRangeCorrectionChanged();
}

//--------------------------------------------------------
//  RangeCorrectionCheckbox
//--------------------------------------------------------
void CalibrationDock::ClearRangeCorrection()
{
    ml2lidar.ClearRangeCorrection();
    mLastMessage = "Calibration cleared";
    UpdateRangeCalInfo();
    emit ResetConfigScanSettings();
    emit ClearPCwindowRequested();
}

//--------------------------------------------------------
//  Calibration override button
//--------------------------------------------------------
void CalibrationDock::CalOVRenable()
{
    bool override = ui->cbCalOveride->isChecked();
    ml2lidar.EnableCalibrationOVR(override);
}

//--------------------------------------------------------
//  RangeBiasChanged
//--------------------------------------------------------
void CalibrationDock::RangeBiasChanged()
{
    int32_t RangeBias = (int32_t) ui->spinRangeBias->value();
    ml2lidar.SetRangeBiasOVR(RangeBias);
}

//--------------------------------------------------------
//  RangeScaleChanged
//--------------------------------------------------------
void CalibrationDock::RangeScaleChanged()
{
    double RangeScale =  ui->spinRangeScale->value();
    ml2lidar.SetRangeScaleOVR(RangeScale);
}

//--------------------------------------------------------
//  MinRangeChanged
//--------------------------------------------------------
void CalibrationDock::MinRangeChanged()
{
    double MinRange_mm =  ui->spinMinRange->value();
    ml2lidar.SetMinRange_mm(MinRange_mm);
}

//--------------------------------------------------------
//  RangeScaleChanged
//--------------------------------------------------------
void CalibrationDock::MaxRangeChanged()
{
    double MaxRange_mm =  ui->spinMaxRange->value();
    ml2lidar.SetMaxRange_mm(MaxRange_mm);
}

//--------------------------------------------------------
//  Theta angle bias changed
//--------------------------------------------------------
void CalibrationDock::ThetaChanged()
{
    double Theta =  ui->spinThetaBias->value();
    ml2lidar.SetThetaAngleBiasOVR(Theta);
}

//--------------------------------------------------------
//  Alpha Angle bias changed
//--------------------------------------------------------
void CalibrationDock::AlphaChanged()
{
    double Alpha =  ui->spinAlphaAngle->value();
    ml2lidar.SetAlphaAngleBiasOVR(Alpha);
}
//--------------------------------------------------------
//  Alpha Angle bias changed
//--------------------------------------------------------
void CalibrationDock::AlphaSizeChanged()
{
    double Alpha =  ui->spinAlphaStepSize->value();
    ml2lidar.SetAlphaAngleStepOVR(Alpha);
}

//--------------------------------------------------------
//  Beta Angle changed
//--------------------------------------------------------
void CalibrationDock::BetaChanged()
{
    double beta =  ui->spinBetaAngle->value();
    ml2lidar.SetBetaAngleOVR(beta);
}

//--------------------------------------------------------
//  Xi Angle changed
//--------------------------------------------------------
void CalibrationDock::XiChanged()
{
    double Xi =  ui->spinXiAngle->value();
    ml2lidar.SetXiAngleOVR(Xi);
}

//--------------------------------------------------------
//  RangeCorrectionCheckbox
//--------------------------------------------------------
void CalibrationDock::Init4Cal()
{
    // initialize L2 for range correction calibration
    // Turn off IMU adjust
    // Enable Calibration parameters override
    // Set RangeScale to 0.001 (converts mm to meters, no scaling)
    // Set the override parameters
    // Clear any exisiting range correction
    // Send request for L2 version info
    ml2lidar.EnableIMUadjust(false);

    ml2lidar.EnableCalibrationOVR(true);
    ui->cbCalOveride->setChecked(true);

    ml2lidar.EnableRangeCorrection(false);
    ui->cbEnableRangeCal->setChecked(false);

    mSavedRangeScale = ml2lidar.GetRangeScaleOVR();

    ml2lidar.SetRangeBiasOVR(static_cast<double>(ui->spinRangeBias->value()));
    ml2lidar.SetThetaAngleBiasOVR(static_cast<double>(ui->spinThetaBias->value()));
    ml2lidar.SetAlphaAngleBiasOVR(static_cast<double>(ui->spinAlphaAngle->value()));

    ml2lidar.LidarGetVersion();

    ui->btnStage1->setEnabled(true);
    ui->btnStage3->setEnabled(true);
}

//--------------------------------------------------------
//  BrowseCalFile
//--------------------------------------------------------
bool CalibrationDock::BrowseCalFile()
{
    // get current filename ui->editCalFile
    QString CurrentFile = ui->editCalFile->text().trimmed();
    QString file = QFileDialog::getOpenFileName(this,
                                "L2 calibration file", CurrentFile, "L2CalFIle (*.csv)");
    if(file.trimmed()=="")
        return false;
    ui->editCalFile->setText(file);
    return true;
}

//--------------------------------------------------------
//  LoadRangeCal
//--------------------------------------------------------
bool CalibrationDock::LoadRangeCal()
{
    // get current filename ui->editCalFile
    QString CurrentFile = ui->editCalFile->text().trimmed();
    bool loaded = ml2lidar.LoadRangeCalibration(CurrentFile.toStdString());

    // update RangeCalinfoDock
    auto errors = ml2lidar.GetRangeCorrectionErrors();
    auto warnings = ml2lidar.GetRangeCorrectionWarnings();

    mLastMessage = "Load Range Calibration finished\n";

    if(errors.size()==0) {
        mLastMessage =  mLastMessage + "\nErrors: NONE\n";
    } else {
        mLastMessage =  mLastMessage + "\nErrors:\n";
        for(const auto& str : errors) {
            mLastMessage =  mLastMessage + str;
        }
    }

    if(warnings.size()==0) {
        mLastMessage = mLastMessage + "\n\nWarnings: NONE\n";
    } else {
        mLastMessage = mLastMessage + "\n\nWarnings:\n";
        for(const auto& str : warnings) {
            mLastMessage =  mLastMessage + str;
        }
    }
    UpdateRangeCalInfo();

    if(!loaded){
        return false;
    }

    mRangeCal.AlphaAngleBias = ml2lidar.GetAlphaAngleBiasOVR();
    ui->spinAlphaAngle->setValue(mRangeCal.AlphaAngleBias);

    mRangeCal.AlphaAngleStepSize = ml2lidar.GetAlphaAngleStepOVR();
    ui->spinAlphaStepSize->setValue(mRangeCal.AlphaAngleStepSize);

    mRangeCal.ThetaAngleBias = ml2lidar.GetThetaAngleBiasOVR();
    ui->spinThetaBias->setValue(mRangeCal.ThetaAngleBias);

    mRangeCal.BetaAngle = ml2lidar.GetBetaAngleOVR();
    ui->spinBetaAngle->setValue( mRangeCal.BetaAngle);

    mRangeCal.RangeBias = ml2lidar.GetRangeBiasOVR();
    ui->spinRangeBias->setValue(mRangeCal.RangeBias);

    mRangeCal.RangeScale = ml2lidar.GetRangeScaleOVR();
    ui->spinRangeScale->setValue(mRangeCal.RangeScale);

    mRangeCal.XiAngle = ml2lidar.GetXiAngleOVR();
    ui->spinXiAngle->setValue(mRangeCal.XiAngle);

    saveINI("Calibration","RangeCalibrationFilename", CurrentFile);
    emit ResetConfigScanSettings();
    emit ClearPCwindowRequested();
    return true;
}

//--------------------------------------------------------
//  Stage1Dialog
//--------------------------------------------------------
void CalibrationDock::Stage1Dialog()
{
    Stage1CalRangeDialog stage1;

    {
        // initialize entries
        // get device version information
        auto connected = ml2lidar.IsL2connected();
        auto versioninfo = ml2lidar.version();

        // date
        QDateTime date = QDateTime::currentDateTime();
        QString formattedTime = date.toString("yyyy-MM-dd hh:mm:ss");
        mRangeCal.Date = formattedTime.toStdString();
        stage1.SetDate(mRangeCal.Date);

        if(connected) {
            QString Product = QString::fromUtf8((const char *)versioninfo.reserve);
            mRangeCal.Sensor = Product.toStdString();
        }
        stage1.SetSensor(mRangeCal.Sensor);

        QString string = loadINI("Stage1","SensorID",(QString)"L2-1");
        mRangeCal.SensorID = string.toStdString();
        stage1.SetSensorID(mRangeCal.SensorID);

        if(connected){
            QString FWversion = QString().asprintf("FW: %d.%d.%d.%d",
                                                   versioninfo.sw_version[0],
                                                   versioninfo.sw_version[1],
                                                   versioninfo.sw_version[2],
                                                   versioninfo.sw_version[3]);
            mRangeCal.Firmware = FWversion.toStdString();

        }
        stage1.SetFirmware(mRangeCal.Firmware);
        stage1.SetVersion(mRangeCal.Version);
        stage1.SetCreatedBy(mRangeCal.CreatedBy);

        string = loadINI("Stage1","CalibrationDescription",(QString)"Validation test");
        mRangeCal.CalibrationDescription = string.toStdString();
        mRangeCal.MinRange = loadINI("Stage1", "MinRange", 150.0);
        mRangeCal.MinTrustedRange = loadINI("Stage1", "MinTrustedRange", -1.0);
        mRangeCal.MaxRange = loadINI("Stage1", "MaxRange", 40000.0);

        stage1.SetDescription(mRangeCal.CalibrationDescription);
        stage1.SetMinRange_mm(mRangeCal.MinRange);
        stage1.SetMinTrustedRange_mm(mRangeCal.MinTrustedRange);
        stage1.SetMaxRange_mm(mRangeCal.MaxRange);
    }

    if(ml2lidar.IsRangeCorrectionLoaded()) {
        auto CalInfo = ml2lidar.GetRangeCalibrationInfo();
        mRangeCal = CalInfo;
        mRangeCal.NumberOfSegments = 0;
        mRangeCal.MinCalRange = 0.0;
        mRangeCal.MaxCalRange = 0.0;
        mRangeCal.RMSResidual = 0.0;
    }

    if (stage1.exec() == QDialog::Accepted) {
        // save parameters tmp
        ui->btnStage2->setEnabled(true);
        ui->lblStatusStage1->setText("Reviewed");
        // save description and sensor ID for next time
        mRangeCal.CalibrationDescription = stage1.GetDescription();
        mRangeCal.SensorID = stage1.GetSensorID();
        mRangeCal.MinRange = stage1.GetMinRange_mm();
        mRangeCal.MinTrustedRange = stage1.GetMinTrustedRange_mm();
        mRangeCal.MaxRange = stage1.GetMaxRange_mm();
        saveINI("Stage1","SensorID", QString::fromStdString(mRangeCal.SensorID));
        saveINI("Stage1","CalibrationDescription", QString::fromStdString(mRangeCal.CalibrationDescription));
        saveINI("Stage1", "MinRange", mRangeCal.MinRange);
        saveINI("Stage1", "MinTrustedRange", mRangeCal.MinTrustedRange);
        saveINI("Stage1", "MaxRange", mRangeCal.MaxRange);
    }
}

//--------------------------------------------------------
//  Stage2Dialog
//--------------------------------------------------------
void CalibrationDock::CalibrationDock::Stage2Dialog()
{
    if(!ml2lidar.IsL2connected()) {
        ui->lblStatusStage2->setText("L2 not connected");
        return;
    }

    if(mstage2==nullptr) {

        mstage2 = new Stage2CalRangeDialog(ml2lidar);

        connect(mstage2, &Stage2CalRangeDialog::ClearPCwindowRequested,
                this, &CalibrationDock::ClearPCwindowRequested);

        connect(mstage2, &Stage2CalRangeDialog::Stage2SavePC,
                this, &CalibrationDock::Stage2SavePC);

        connect(mstage2, &Stage2CalRangeDialog::Finished,
                this, &CalibrationDock::FinishStage2ACQ);

        mstage2->SetStartAngle(loadINI("RangeCal","StartAngle", 358.0));
        mstage2->SetAngleWidth(loadINI("RangeCal","AngleWidth", 3.0));
        mstage2->SetNumScans(loadINI("RangeCal","NumPoints", (int)20000));
        mstage2->setWindowFlag(Qt::Tool, true);
    }
    mstage2->show();
    mstage2->raise();
    mstage2->activateWindow();
    // mstage2->setVisible(true);
    mstage2->ClearSaved();
    ui->lblStatusStage2->setText("incomplete");
}

//--------------------------------------------------------
//  FinishStage2ACQ
//--------------------------------------------------------
void CalibrationDock::Stage2SavePC()
{
    ui->lblStatusStage2->setText("Saving PC");
    emit SavePC4Stage2();
}

//--------------------------------------------------------
//  Stage2SaveDone
//--------------------------------------------------------
void CalibrationDock::Stage2SaveDone(bool completed)
{
    mstage2->SetAQCsaved(completed);
    if(completed) {
        ui->lblStatusStage2->setText("Saved PC");
    } else {
        ui->lblStatusStage2->setText("Not saved");
    }
}

//--------------------------------------------------------
//  FinishStage2ACQ
//--------------------------------------------------------
void CalibrationDock::FinishStage2ACQ()
{
    if(mstage2->GetACQsaved()) {
        ui->lblStatusStage2->setText("Acqusition Saved");
    }
    // restore normal operation
    ml2lidar.SetScanAngleWidth(mSavedAngleWidth);
    ml2lidar.SetStartScanAngle(mSavedStartAngle);
    ml2lidar.EnableFlattenScan(mSavedFlattened);
    ml2lidar.SetRangeScaleOVR(mSavedRangeScale);
}

//--------------------------------------------------------
//  Stage3Dialog
//--------------------------------------------------------
void CalibrationDock::Stage3Dialog()
{
    if(mstage3==nullptr) {
        mstage3 = new Stage3CalRangeDialog();

        mstage3->SetMinRange_m(mRangeCal.MinRange/1000.0); // Stage 3 works in meters
        mstage3->SetMaxRange_m(mRangeCal.MaxRange/1000.0); // Stage 2 works in meters

        // connect for Range Calibration GUI request
        connect(mstage3, &Stage3CalRangeDialog::CalGUIrequest,
                this, &CalibrationDock::RangeCalGUI);
    }
    mstage3->raise(); // bring to front
    mstage3->setVisible(true);
    mstage3->CalGUIrequest(true,false); // clear GUI, do not show GUI
    ui->lblStatusStage3->setText("incomplete");
}

//--------------------------------------------------------
//  Stage3accepted
//--------------------------------------------------------
void CalibrationDock::Stage3accepted()
{
    // THese are done in mainwindows
       // GetCalibrationSegments();
       // GetExclusionRegions();

    ui->lblStatusStage3->setText("completed"); // mark stage 3 as complete
    ui->btnStage4->setEnabled(true); // stage 4 has valid input
    mstage3->setVisible(false);
    mstage3->Stage3accepted();// notify the stage 3 dialog the GUI was cancelled
}

//--------------------------------------------------------
//  SetCalibrationSegments
//--------------------------------------------------------
void CalibrationDock::SetStage3CalSegments(const std::vector<CalibrationSegment>& segments)
{
    mstage3->SetStage3CalSegments(segments);
}

//--------------------------------------------------------
//  SetExclusionRegions
//--------------------------------------------------------
void CalibrationDock::SetStage3ExclusionRegions(const std::vector<ExclusionRegion>& regions)
{
    mstage3->SetStage3ExclusionRegions(regions);

}


//--------------------------------------------------------
//  Stage3rejected
//--------------------------------------------------------
void CalibrationDock::Stage3rejected()
{
    //  set stage3 status to incomplete (this is done in CalibrationDock)
    ui->lblStatusStage3->setText("incomplete"); // mark stage 3 as incomplete
    ui->btnStage4->setEnabled(false); // stage 4 does not have valid input
    mstage3->Stage3rejected();// notify the stage 3 dialog the GUI was cancelled
    mstage3->setVisible(true);
}

//--------------------------------------------------------
//  ProcessStage4
//--------------------------------------------------------
void CalibrationDock::ProcessStage4()
{
    // get measurements from stage3
    auto const measurements = mstage3->GetCalMeasurements();

    // if stage1 not "reviewed" then present stage 1 dialog
    QString stage1status = ui->lblStatusStage1->text();
    if(stage1status!="Reviewed") {
        Stage1Dialog();
    }

    ui->lblStatusStage4->setText("Processing");

    // Query user for the number of spline segments to use
    // read in last use number of spline segments
    int NumSplineSegments = loadINI("Stage4","NumSplineSegments",10);
    int maxSplineSegments = measurements.size();

    // Query user for number of segments
    bool ok;
    int NumSplines = QInputDialog::getInt(this,
        tr("Enter value"),
        tr("Number of cubic spline steps: "),
        NumSplineSegments, // default value
        4,      // minimum
        maxSplineSegments,    // maximum
        1,      // step
        &ok
        );

    if (!ok) {
        ui->lblStatusStage4->setText("Cancelled");
        return;
    }

    // process measurements
    if(mStage4CalFit.ProcessCalibrationMeasurements(measurements,NumSplines,
                                                    mRangeCal.MinRange/1000.0,
                                                    mRangeCal.MaxRange/1000.0)) {
        saveINI("Stage4","NumSplineSegments",NumSplines);
        ui->lblStatusStage4->setText("Processed");
        ui->btnStage5->setEnabled(true);
    } else {
        ui->lblStatusStage4->setText("FAILED");
        std::string Message  = mStage4CalFit.GetLastErrorMessage();
        QMessageBox msgBox;
        msgBox.setText("Stage4 computation failure");
        msgBox.setInformativeText(QString::fromStdString(Message));
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
}

//--------------------------------------------------------
//  ProcessStage5
//--------------------------------------------------------
void CalibrationDock::ProcessStage5()
{
    // capture current calibration override settings
    mRangeCal.AlphaAngleBias  = ui->spinAlphaAngle->value();
    mRangeCal.AlphaAngleStepSize  = ui->spinAlphaStepSize->value();
    mRangeCal.ThetaAngleBias = ui->spinThetaBias->value();
    mRangeCal.BetaAngle = ui->spinBetaAngle->value();
    mRangeCal.RangeBias = ui->spinRangeBias->value();
    mRangeCal.RangeScale = ui->spinRangeScale->value();
    mRangeCal.XiAngle = ui->spinXiAngle->value();

    // save Alpha LUT
    // This is for future implementation of a non linear
    // Alpha Angle correction if needed
    std::vector<double> AlphaAngleLUT;
    AlphaAngleLUT.clear();

    L2RangeCalibrationWriter writer;
    RangeCalibrationInfo outputInfo = mRangeCal;
    auto candidate = mStage4CalFit.GetCandidate(); // certain measurements are in meters
    auto fitMeasurements = mStage4CalFit.GetMeasurements();

    // merge stage1 RangeCalibrationInfo with stage 4 results
    outputInfo.MinCalRange  =candidate.minCalRange*1000.0; // convert to mm
    outputInfo.MaxCalRange = candidate.maxCalRange*1000.0; // convert to mm
    outputInfo.RMSResidual = candidate.rmsResidual*1000.0; // convert to mm
    outputInfo.NumberOfSegments = candidate.segments.size();

    // get current filename ui->editCalFile
    QString CurrentFile = ui->editCalFile->text().trimmed();
    QString file = QFileDialog::getSaveFileName(this,
                                                "L2 calibration file", CurrentFile, "L2CalFIle (*.csv)");
    if(file.trimmed()=="") {
        ui->lblStatusStage5->setText("Cancelled");
        return;
    }

    if (!writer.SaveCalibrationFile(file.toStdString(),
                                    candidate,
                                    fitMeasurements,
                                    outputInfo,
                                    AlphaAngleLUT)) {
        std::string Message  = writer.GetLastErrorMessage();
        QMessageBox msgBox;
        msgBox.setText("Stage5 save cal file failure");
        msgBox.setInformativeText(QString::fromStdString(Message));
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
        ui->lblStatusStage5->setText("Failed");
        return;
    }

    // set text in ui so that it can be loaded easily
    ui->editCalFile->setText(file);

    ui->lblStatusStage5->setText("Saved");

}

//--------------------------------------------------------
//  AlphaAngleCheckbox
//--------------------------------------------------------
void CalibrationDock::AlphaAngleCheckbox()
{
    if(ui->cbEnableAlphaLUT->isChecked()) {
        ml2lidar.EnableAlphaAngleLUT(true);
    } else {
        ml2lidar.EnableAlphaAngleLUT(false);
    }
}
