#ifndef DOUBLE_SLIDER_H
#define DOUBLE_SLIDER_H
#include <QApplication>
#include <QtGui>
#include <QVBoxLayout>
#include <QSlider>
#include <QLabel>

class DoubleSlider : public QSlider 
{
    Q_OBJECT

public:
    DoubleSlider(QWidget *parent = 0) : QSlider(parent) 
    {
        connect(this, SIGNAL(valueChanged(int)),
            this, SLOT(notifyValueChanged(int)));
        
    }

signals:
    void doubleValueChanged(double value);

public slots:   


    void notifyValueChanged(int value) {
        double doubleValue = value / 1000.0;
        emit doubleValueChanged(doubleValue);
    }


};

#endif