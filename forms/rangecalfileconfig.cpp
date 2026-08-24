#include "rangecalfileconfig.h"
#include "ui_rangecalfileconfig.h"

RangeCalFileConfig::RangeCalFileConfig(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::RangeCalFileConfig)
{
    ui->setupUi(this);
}

RangeCalFileConfig::~RangeCalFileConfig()
{
    delete ui;
}
