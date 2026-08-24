#ifndef RANGECALFILECONFIG_H
#define RANGECALFILECONFIG_H

#include <QDialog>

namespace Ui {
class RangeCalFileConfig;
}

class RangeCalFileConfig : public QDialog
{
    Q_OBJECT

public:
    explicit RangeCalFileConfig(QWidget *parent = nullptr);
    ~RangeCalFileConfig();

private:
    Ui::RangeCalFileConfig *ui;
};

#endif // RANGECALFILECONFIG_H
