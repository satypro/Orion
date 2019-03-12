#include "TreeWidget.h"

TreeWidget::TreeWidget(MainWindow *mainWnd)
{
  this->mainWnd = mainWnd;

  xSlider = createSlider();
  ySlider = createSlider();
  zSlider = createSlider();

    //connect(xSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setXRotation(int)));
   // connect(glWidget, SIGNAL(xRotationChanged(int)), xSlider, SLOT(setValue(int)));
   // connect(ySlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setYRotation(int)));
   // connect(glWidget, SIGNAL(yRotationChanged(int)), ySlider, SLOT(setValue(int)));
   // connect(zSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setZRotation(int)));
   // connect(glWidget, SIGNAL(zRotationChanged(int)), zSlider, SLOT(setValue(int)));

    QHBoxLayout *mainLayout = new QHBoxLayout;
 //   mainLayout->addWidget(glWidget);
    mainLayout->addWidget(xSlider);
    mainLayout->addWidget(ySlider);
    mainLayout->addWidget(zSlider);
    setLayout(mainLayout);

    xSlider->setValue(15 * 110);
    ySlider->setValue(345 * 8.8);
    zSlider->setValue(22 * 55 );
   // setWindowTitle(tr("MT2013008_Assignment2"));
}

QSlider * TreeWidget::createSlider()
{
    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, 360 * 16);
    slider->setSingleStep(16);
    slider->setPageStep(15 * 16);
    slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    return slider;
}

void TreeWidget::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}
