//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: Stage3CalRangeDialog.h
//
//
//  Purpose:
//  Stage 3 data analysis and extraction
//
//  V2.0.0 RC1 2026-07-31
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
// This is the Stage 3 data analysis and extraction dialog
//--------------------------------------------------------
#include "Stage3CalRangeDialog.h"
#include "settingINI.h"

Stage3CalRangeDialog::Stage3CalRangeDialog(QWidget* parent)
    : QDialog(parent),
    ui(new Ui::Stage3CalRangeDialog)
{
    ui->setupUi(this);

    // Exit button
    connect(ui->btnExit, &QPushButton::clicked,
            this, &Stage3CalRangeDialog::Exit);

    // Browse button
    connect(ui->btnBrowsePCDfile, &QPushButton::clicked,
            this, &Stage3CalRangeDialog::BrowseFile);

    // load PCD button
    connect(ui->btnLoad, &QPushButton::clicked,
            this, &Stage3CalRangeDialog::LoadPC);

    // Browse Save CSV button
    connect(ui->btnBrowseCSVfile, &QPushButton::clicked,
            this, &Stage3CalRangeDialog::BrowseCSVfile);

    // Analyze button
    connect(ui->btnAnalyze, &QPushButton::clicked,
            this, &Stage3CalRangeDialog::AnalyzePC);

    ui->editPCDfile->setText(loadINI("RangeCalStage3","PCDfilename",QString("")));
    ui->editCSVfile->setText(loadINI("RangeCalStage3","CSVfilename",QString("")));
    ui->spinBinSize->setValue(loadINI("RangeCalStage3","BinSize",0.01));
    ui->spinFilterSigma->setValue(loadINI("RangeCalStage3","FilterSigma",0.04));
    ui->spinFilterRadius->setValue(loadINI("RangeCalStage3","FilterRadius",4.0));
}

//--------------------------------------------------------
// DiagnosticsDock destructor
//--------------------------------------------------------
Stage3CalRangeDialog::~Stage3CalRangeDialog()
{
    delete ui;
}


//--------------------------------------------------------
// BrowseFile
//--------------------------------------------------------
void Stage3CalRangeDialog::BrowseFile()
{
    QString file;
    file = ui->editPCDfile->text();

    file = QFileDialog::getOpenFileName(this, "Load Point Cloud", file, "PointCloud (*.pcd)");
    if(file.trimmed()=="") return;

    saveINI("RangeCalStage3","PCDfilename",file);
    ui->editPCDfile->setText(file);
}

//--------------------------------------------------------
// BrowseCSVfile
//--------------------------------------------------------
void Stage3CalRangeDialog::BrowseCSVfile()
{
    QString file;
    file = ui->editCSVfile->text();

    file = QFileDialog::getSaveFileName(this, "CSV calibration analysis", file, "CSV (*.csv)");
    if(file.trimmed()=="") return;

    saveINI("RangeCalStage3","CSVfilename",file);
    ui->editCSVfile->setText(file);
}

//--------------------------------------------------------
// LoadPC
//--------------------------------------------------------
bool Stage3CalRangeDialog::LoadPC()
{
    QString file;
    file = ui->editPCDfile->text();

    if(file.trimmed()=="") return false;

    if(!readPCDfile(file,&mcloud)){
        QMessageBox msgBox;
        msgBox.setText("Can not load point cloud");
        msgBox.setInformativeText("bad file format");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
        ui->lblNumPoints->setText("nothing loaded");
        return false;
    }

    saveINI("RangeCalStage3","PCDfilename",ui->editPCDfile->text());

    QString ResultString;
    ResultString = ResultString.asprintf("%d",(int)mcloud.size());
    ui->lblNumPoints->setText(ResultString);
    ui->btnAnalyze->setEnabled(true);

    // clear analysis gui, do not show GUI
    emit CalGUIrequest(true, false);
    return true;
}

//--------------------------------------------------------
// AnalyzePC
//--------------------------------------------------------
void Stage3CalRangeDialog::AnalyzePC()
{
    if(mcloud.size()==0) {
        ui->lblNumPoints->setText("nothing loaded");
        return;
    }

    double p = ui->spinBinSize->value();
    mRangeAnalysis.SetBinSize(p);
    saveINI("RangeCalStage3","BinSize",p);

    p = ui->spinFilterSigma->value();
    mRangeAnalysis.SetFilterSigma(p);
    saveINI("RangeCalStage3","FilterSigma",p);

    p = ui->spinFilterRadius->value();
    mRangeAnalysis.SetFilterRadiusSigma(p);
    saveINI("RangeCalStage3","FilterRadius",p);

    // Stage 3A Analyyze
    if(!mRangeAnalysis.AnalyzePointCloud(mcloud, mCal, mAnalysisResult)) {
        return;
    }

    {
        QString filename = ui->editCSVfile->text();
        if(!(filename.trimmed()=="")) {
            QFileInfo info(filename);
            // save analysis
            QString newFile = QDir(info.path()).filePath(
                info.completeBaseName() +
                "-analysis" +
                "." +
                info.completeSuffix());
            mRangeAnalysis.ExportAnalysisCSV(newFile.toStdString(),mAnalysisResult);

            auto binsfound = mAnalysisResult.elevationPeaks.size();
            auto samplesize = mAnalysisResult.samples.size();
            QString results = QString::asprintf("Analysis completed\nSamples processed: %u\nElevation bins found: %u\nAnalysis file generated:\n",
                                                samplesize, binsfound) + newFile;
            ui->lblStatus->setText(results);
        }
    }

    // Stage 3B Evaluation

    mL2RangeExtraction.SetMinRange_m(mMinRange_m); // this is in meters
    mL2RangeExtraction.SetMaxRange_m(mMaxRange_m); // this is in meters

    // use the Stage 3A results in Stage 3B
    mL2RangeExtraction.SetInput(mAnalysisResult);

    if(!mL2RangeExtraction.Evaluate()) {
        return;
    }
    {
        QString filename = ui->editCSVfile->text();
        if(!(filename.trimmed()=="")) {
            QFileInfo info(filename);
            // save analysis
            QString newFile = QDir(info.path()).filePath(
                info.completeBaseName() +
                "3B-evaluated" +
                "." +
                info.completeSuffix());
            mL2RangeExtraction.ExportMeasuredPointsCSV(newFile.toStdString());
        }
    }

    ui->btnAnalyze->setEnabled(false);
    setVisible(false); // set the stage3 dialog invisible

    // keep current points, show GUI
    emit CalGUIrequest(false, true);

    return;
}

//--------------------------------------------------------
// Stage3accected
//--------------------------------------------------------
void Stage3CalRangeDialog::Stage3accepted()
{
    //    OK
    // ↓
    // use the current RangeAnalysisResults

    // ↓
    // Get the GUI segments and exclusions
    mL2RangeExtraction.SetCalibrationSegments(mSegments);
    mL2RangeExtraction.SetExclusionRegions(mExclusionRegions);

    // ↓
    // Set into L2RangeExtraction
    mL2RangeExtraction.SetInput(mAnalysisResult);

    // ↓
    // Evaluate using the segments and exclusion from the GUI
    if(!mL2RangeExtraction.Evaluate()) {
        //disable Stage 4
        //mark stage3 as failed
    }

    // ↓
    // BuildCalibrationMeasurements()

    if (!mL2RangeExtraction.BuildCalibrationMeasurements(mMeasurements)) {
           //disable Stage 4
           //mark stage3 as failed
    }

    // ↓
    //          report results in CSV analysis file
    {
        QString filename = ui->editCSVfile->text();
        if(!(filename.trimmed()=="")) {
            QFileInfo info(filename);
            // save analysis
            QString newFile = QDir(info.path()).filePath(
                         info.completeBaseName() +
                         "CalPoints" +
                         "." +
                         info.completeSuffix());
            mL2RangeExtraction.ExportCalMeasurementsCSV(newFile.toStdString(), mMeasurements);
            auto binsfound = mAnalysisResult.elevationPeaks.size();
            auto samplesize = mAnalysisResult.samples.size();
            QString results = QString::asprintf("Stage3 completed\nSamples processed: %u\nElevation bins found: %u\nAnalysis file generated:\n",
                                                samplesize, binsfound) + newFile;
            ui->lblStatus->setText(results);
        }
    }

    emit CalGUIrequest(false,false); // don't clear GUI, hide GUI

    setVisible(false); // make the stage3 dialog invisible

    // calibration dock dock will enable the stage 4 button

    //  enable the Analyze button for the future
    ui->btnAnalyze->setEnabled(true);

    // save the current stage 3 settings
    saveINI("RangeCalStage3","PCDfilename",ui->editPCDfile->text());
    saveINI("RangeCalStage3","CSVfilename",ui->editCSVfile->text());
    saveINI("RangeCalStage3","BinSize",ui->spinBinSize->text().toDouble());
    saveINI("RangeCalStage3","FilterSigma",ui->spinFilterSigma->text().toDouble());
    saveINI("RangeCalStage3","FilterRadius",ui->spinFilterRadius->text().toDouble());

    return;
}

//--------------------------------------------------------
// Stage3rejected
//--------------------------------------------------------
void Stage3CalRangeDialog::Stage3rejected()
{
    setVisible(true); // make the stage3 dialog visible
    //  set stage3 status to incomplete (this is done in CalibrationDock)
    //  enable the Analyze button
    ui->btnAnalyze->setEnabled(true);
    emit CalGUIrequest(true,false); // clear GUI, hide GUI
    return;
}

//--------------------------------------------------------
// Exit
//--------------------------------------------------------
void Stage3CalRangeDialog::Exit()
{
    emit CalGUIrequest(false,false);
    setVisible(false);
}


