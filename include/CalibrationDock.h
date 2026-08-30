//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: CalibrationDock.h
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
//  V2.0.0 RC1 2026-08-18 Added calibration model for the L2
//  V2.0.1  2026-08-24  This is the intial V2.x release
//                      Implemented application of the alpha angle LUT
//                      Added Alpha Angle step size override
//                      Some cleanup of the UI and GUI interactions
//  V2.1.0  2026-08-27  Changed calibration file so that range correction
//                          optional.  This allows just metadata to be saved
//                          which includes the overrride biases.
//                      Changed files/names to reflect generalization
//                          of the calibration file rather than RangeCorrection
//
//--------------------------------------------------------
#ifndef CALIBRATIONDOCK_H
#define CALIBRATIONDOCK_H

#include <QDockWidget>
#include <QFileDialog>
#include "L2lidar.h"
#include "L2calibration.h"
#include "Stage2CalRangeDialog.h"
#include "Stage4CalRangeClasses.h"
#include "Stage5saveCalibration.h"
#include "CalGraphDock.h"
#include "forms/ui_CalibrationDock.h"

QT_BEGIN_NAMESPACE
namespace Ui { class CalibrationDock; }
QT_END_NAMESPACE

class CalibrationDock : public QDockWidget
{
    Q_OBJECT

public:
    explicit CalibrationDock(L2lidar& lidar, QWidget *parent = nullptr);
    ~CalibrationDock();

    void setConnectState(bool connected);

    QString GetRangeCalFile() const
    {
        return ui->editCalFile->text().trimmed();
    }

    void SetRangeCalFile(const QString& filename) const
    {
        ui->editCalFile->setText(filename);
    }

    void EnableRangeCorrection(bool p)
    {
        ui->cbEnableRangeCal->setChecked(p);
    }

    //--------------------------------------------------------
    // Calibration override
    //--------------------------------------------------------
    bool isCalOVRenabled()
    {
        return static_cast<bool>(ui->cbCalOveride->isChecked());
    }

    bool isRangeCorrectionEnabled()
    {
        return static_cast<bool>(ui->cbEnableRangeCal->isChecked());
    }

    double getRangeScale() const
    {
        return static_cast<double>(ui->spinRangeScale->value());
    }

    double getMinRange_mm() const
    {
        return static_cast<double>(ui->spinMinRange->value());
    }

    double getMaxRange_mm() const
    {
        return static_cast<double>(ui->spinMaxRange->value());
    }

    double getRangeBias() const
    {
        return static_cast<double>(ui->spinRangeBias->value());
    }

    double getAlphaBias() const
    {
        return static_cast<double>(ui->spinAlphaAngle->value());
    }

    double getAlphaStep() const
    {
        return static_cast<double>(ui->spinAlphaStepSize->value());
    }

    double getThetaBias() const
    {
        return static_cast<double>(ui->spinThetaBias->value());
    }

    double getBetaAngle() const
    {
        return static_cast<double>(ui->spinBetaAngle->value());
    }

    double getXiAngle() const
    {
        return static_cast<double>(ui->spinXiAngle->value());
    }

    const std::string GetLastMessage() const
    {
        return mLastMessage;
    }

    //--------------------------------------------------------
    // Calibration override
    //--------------------------------------------------------
    void EnableCalOVR(bool p)
    {
        ui->cbCalOveride->setChecked(p);
    }

    void SetRangeScale(double p) const
    {
        ui->spinRangeScale->setValue(p);
    }

    void SetMinRange_mm(double p) const
    {
        ui->spinMinRange->setValue(p+0.5);
    }

    void SetMaxRange_mm(double p) const
    {
        ui->spinMaxRange->setValue(p+0.5);
    }

    void setRangeBias(double p) const
    {
        ui->spinRangeBias->setValue(p);
    }

    void setAlphaBias(double p) const
    {
        ui->spinAlphaAngle->setValue(p);
    }

    void setAlphaStep(double p) const
    {
        ui->spinAlphaStepSize->setValue(p);
    }

    void setThetaBias(double p) const
    {
        ui->spinThetaBias->setValue(p);
    }

    void setBetaAngle(double p) const
    {
        ui->spinBetaAngle->setValue(p);
    }

    void setXiAngle(double p) const
    {
        ui->spinXiAngle->setValue(p);
    }

    void SaveStartAngle(double p) { mSavedStartAngle = p;}
    void SaveAngleWitdh(double p) { mSavedAngleWidth = p;};
    void SaveFlattened(double p) { mSavedFlattened = p;};

    bool IsACQrunning() {
        if(mstage2!=nullptr)
            return mstage2->IsACQrunning();
        return false;
    }
    void Stage2SaveDone(bool completed);

    // get analysis result for GUI
    const std::vector<Stage3BPoint>& GetStage3BPoints() const noexcept
    {
        return mstage3->GetStage3BPoints();
    }

    // used to pass back the cancel but in the GUI for stage 3
    void Stage3rejected();
    void Stage3accepted();
    void SetStage3CalSegments(const std::vector<CalibrationSegment>& segments);
    void SetStage3ExclusionRegions(const std::vector<ExclusionRegion>& regions);

signals:
    void DiagnosticMode();
    void L2connectRequested();
    void L2disconnectRequested();
    void ClearPCwindowRequested();
    void LoadPC();
    void SavePC();
    void SavePC4Stage2();
    void UpdateCalibrationInfo();
    void EnableRangeCorrectionChanged();
    void CalGUIrequest(bool clear, bool visible);
    void ResetConfigScanSettings(); // restore the StartScanAngle,
                                    //SacnAngleWidth, Flatten flag,
                                    //IMUadjust flag, IMUrollPith only flag

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void RangeCorrectionCheckbox();
    // Range Calibration GUI request
    void RangeCalGUI(bool clear, bool visible);

private:
    bool BrowseCalFile();
    bool LoadCal();
    void ClearRangeCorrection();
    void Init4Cal();
    void RangeBiasChanged();
    void MinRangeChanged();
    void MaxRangeChanged();
    void RangeScaleChanged();
    void ThetaChanged();
    void AlphaChanged();
    void AlphaSizeChanged();
    void BetaChanged();
    void XiChanged();
    void CalOVRenable();
    void FinishStage2ACQ();
    void Stage2SavePC();
    void AlphaAngleCheckbox();

    // range correction stages
    void Stage1Dialog(); //btnStage1
    void Stage2Dialog(); //btnStage2
    void Stage3Dialog(); //btnStage3
    void ProcessStage4(); //btnStage4
    void ProcessStage5(); //btnStage5

    Stage2CalRangeDialog *mstage2 {nullptr};
    Stage3CalRangeDialog *mstage3 {nullptr};

    Ui::CalibrationDock *ui;
    std::string mLastMessage = "No messages";
    L2lidar& ml2lidar;

    // Stage1 parmeters
    CalibrationInfo mCalib {
        "Range Correction Calibration spec V2.1.0",
        "yyyy-MM-dd hh:mm:ss",
        "Unitree L2 4D LiDAR",
        "#1",
        "unknown",
        "L2diagnostic V2.1.0",
        "None",
        "No calibration loaded",
        0,
        0.001,
        1.65,
        0.602,
        120.0,
        0.25,
        0.25,
        0,
        150,
        40000,
        0,
        0,
        0.0
    };

    // saved configuration
    double mSavedStartAngle {0.0};
    double mSavedAngleWidth {0.0};
    bool mSavedFlattened {false};
    double mSavedRangeScale {0.000978};

    // Stage3 GUI

    // stage 4 results
    L2RangeCalibrationFit mStage4CalFit;
};

#endif // CALIBRATIONDOCK_H
