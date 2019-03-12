#ifndef PARAMETER_WINDOW_H
#define PARAMETER_WINDOW_H

#include <QWidget>
#include <QMainWindow>
#include <QAction>
#include <QtGui>

#include "mainwindow.h"

class QSlider;
class DoubleSlider;



class ParmWidget : public QWidget
{
    Q_OBJECT

public:
    ParmWidget(MainWindow *mainWnd);
   // ~ParmWidget();
   
protected:
   // void keyPressEvent(QKeyEvent *event);

public slots:    
	void rminChanged(double value);
    void rmaxChanged(double value);
    void scaleChanged(int value);

   
   
private:
	MainWindow *mainWnd;
    DoubleSlider *createRadiusSlider(int value);
    QSlider *createScaleSlider();

    DoubleSlider *rminSlider;
    DoubleSlider *rmaxSlider;
    QSlider *scaleSlider;
  
    QLabel* rminLabel;
    QLabel* rmaxLabel;
    QLabel* scaleLabel;
};

#endif
