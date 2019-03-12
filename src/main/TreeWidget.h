#ifndef TREE_WINDOW_H
#define TREE_WINDOW_H

#include <QWidget>
#include <QMainWindow>
#include <QAction>
#include <QtGui>
#include <QApplication>
#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include "mainwindow.h"

class QSlider;




class TreeWidget : public QWidget
{
    Q_OBJECT

public:
    TreeWidget(MainWindow *mainWnd);
   
protected:
    void keyPressEvent(QKeyEvent *event);

private:
	MainWindow *mainWnd;
	
    QSlider *createSlider();

    QSlider *xSlider;
    QSlider *ySlider;
    QSlider *zSlider;
};

#endif
