#ifndef WORKMODEDIALOG_H
#define WORKMODEDIALOG_H

#include <QDialog>

namespace Ui {
class WorkmodeDialog;
}

class WorkmodeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit WorkmodeDialog(QWidget *parent = nullptr);
    ~WorkmodeDialog();

private:
    Ui::WorkmodeDialog *ui;
};

#endif // WORKMODEDIALOG_H
