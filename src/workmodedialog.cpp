#include "workmodedialog.h"
#include "ui_workmodedialog.h"

WorkmodeDialog::WorkmodeDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::WorkmodeDialog)
{
    ui->setupUi(this);
}

WorkmodeDialog::~WorkmodeDialog()
{
    delete ui;
}
