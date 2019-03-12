#include "Strucparam.h"
#include "doubleSlider.h"


Strucparam::Strucparam(MainWindow *mainWnd)
{
  this->mainWnd = mainWnd;

  epsLabel = new QLabel("No Data");
  sigminLabel = new QLabel("No Data");
  sigmaxLabel = new QLabel("No Data");
  scalarMinLabel = new QLabel("No Data");
  scalarMaxLabel = new QLabel("No Data");

  epsSlider = createEpsillonSlider();
  sigminSlider = createSigmaSlider();
  sigmaxSlider = createSigmaSlider();
  scalarMaxSlider = createEpsillonSlider();
  scalarMinSlider = createEpsillonSlider();

  connect(epsSlider, SIGNAL(doubleValueChanged(double)),this,SLOT(epsChanged(double)));
  connect(sigminSlider, SIGNAL(doubleValueChanged(double)),this,SLOT(sigminChanged(double)));
  connect(sigmaxSlider, SIGNAL(doubleValueChanged(double)),this,SLOT(sigmaxChanged(double)));
  connect(scalarMaxSlider, SIGNAL(doubleValueChanged(double)),this,SLOT(scalarMaxChanged(double)));
  connect(scalarMinSlider, SIGNAL(doubleValueChanged(double)),this,SLOT(scalarMinChanged(double)));

  connect(epsSlider, SIGNAL(doubleValueChanged(double)), mainWnd, SLOT(setEps(double)));
  connect(sigminSlider, SIGNAL(doubleValueChanged(double)), mainWnd, SLOT(setSigmaMin(double)));
  connect(sigmaxSlider, SIGNAL(doubleValueChanged(double)), mainWnd, SLOT(setSigmaMax(double)));
  connect(scalarMaxSlider, SIGNAL(doubleValueChanged(double)), mainWnd, SLOT(setScalarMax(double)));
  connect(scalarMinSlider, SIGNAL(doubleValueChanged(double)), mainWnd, SLOT(setScalarMin(double)));
  //connect(glWidget, SIGNAL(rminChanged(double)), xSlider, SLOT(setValue(int)));
  /*connect(ySlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setYRotation(int)));
  connect(glWidget, SIGNAL(yRotationChanged(int)), ySlider, SLOT(setValue(int)));
  connect(zSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setZRotation(int)));
  connect(glWidget, SIGNAL(zRotationChanged(int)), zSlider, SLOT(setValue(int)));*/

    QVBoxLayout *mainLayout = new QVBoxLayout;
    /* mainLayout->addWidget(rminLabel);
 //   mainLayout->addWidget(glWidget);
    mainLayout->addWidget(rminSlider);
    mainLayout->addWidget(rmaxLabel);
    mainLayout->addWidget(rmaxSlider);
    mainLayout->addWidget(epsLabel);
    mainLayout->addWidget(epsSlider);
    mainLayout->addWidget(sigminLabel);
    mainLayout->addWidget(sigminSlider);
    mainLayout->addWidget(sigmaxLabel);
    mainLayout->addWidget(sigmaxSlider);*/
   /* mainLayout->addWidget(epsLabel);
    mainLayout->addWidget(epsSlider);
      mainLayout->addWidget(sigminLabel);
    mainLayout->addWidget(sigminSlider);
    mainLayout->addWidget(sigmaxLabel);
    mainLayout->addWidget(sigmaxSlider);*/
    QHBoxLayout *hmainLayout = new QHBoxLayout;
    hmainLayout->addWidget(epsSlider);
    hmainLayout->addWidget(epsLabel);

    QHBoxLayout *hmainLayout1 = new QHBoxLayout;
    hmainLayout1->addWidget(sigminSlider);
    hmainLayout1->addWidget(sigminLabel);

    QHBoxLayout *hmainLayout2 = new QHBoxLayout;
    hmainLayout2->addWidget(sigmaxSlider);
    hmainLayout2->addWidget(sigmaxLabel);

    QHBoxLayout *hmainLayout3 = new QHBoxLayout;
    hmainLayout3->addWidget(scalarMinSlider);
    hmainLayout3->addWidget(scalarMinLabel);

    QHBoxLayout *hmainLayout4 = new QHBoxLayout;
    hmainLayout4->addWidget(scalarMaxSlider);
    hmainLayout4->addWidget(scalarMaxLabel);
    

    mainLayout->addLayout(hmainLayout);
    mainLayout->addLayout(hmainLayout1);
    mainLayout->addLayout(hmainLayout2);
    mainLayout->addLayout(hmainLayout3);
    mainLayout->addLayout(hmainLayout4);

  
   
    setLayout(mainLayout);

    epsSlider->setValue(650);
    sigminSlider->setValue(200);
    sigmaxSlider->setValue(1700);
    scalarMinSlider->setValue(1);
    scalarMaxSlider->setValue(1000);
  
}


DoubleSlider *Strucparam::createEpsillonSlider()
{
    DoubleSlider *slider = new DoubleSlider();
    slider->setOrientation(Qt::Horizontal);
    slider->setRange(0, 1000);
    slider->setSingleStep(100);
    //slider->setPageStep(15 * 16);
    //slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    return slider;
}

DoubleSlider *Strucparam::createSigmaSlider()
{
    DoubleSlider *slider = new DoubleSlider();
    slider->setOrientation(Qt::Horizontal);
    slider->setRange(0, 2000);
    slider->setSingleStep(1000);
  //  slider->setPageStep(15 * 16);
 //   slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    return slider;
}


void  Strucparam::epsChanged(double value)
{
    epsLabel->setText("eps Value is: "+ QString::number(value) );  
}

void  Strucparam::sigminChanged(double value)
{
    sigminLabel->setText("sigmin value is: "+ QString::number(value) );  
}

void  Strucparam::sigmaxChanged(double value)
{
    sigmaxLabel->setText("sigmax value is: "+ QString::number(value) );  
}

void  Strucparam::scalarMinChanged(double value)
{
    scalarMinLabel->setText("scalarMin = "+ QString::number(value) );
}


void  Strucparam::scalarMaxChanged(double value)
{
    scalarMaxLabel->setText("scalarMax = "+ QString::number(value) );
}

/*

void ParmWidget::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}
*/
