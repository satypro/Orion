/****************************************************************************
**
**
****************************************************************************/

#include <QtGui>
#include <QtOpenGL>

#include "glwidget.h"
#include "mainwindow.h"
#include "ParameterWidget.h"
#include "TreeWidget.h"
#include "Strucparam.h"

/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
MainWindow::MainWindow() 
{

    m_handReduction = new Processing();
    glWidget = new GLWidget(0, this);

    this->resize(800, 800);
    setCentralWidget(glWidget);
    setWindowTitle(tr("Vis tool"));

    initVisActions();
    createActions();
    createMenus();
    createDockWindows();

    //Parameter initialization
    rmin = 0.009;
    rmax = 0.011;
    scale = 3;
    eps = 0.65;
    sigmin = 0.2;
    sigmax = 1.7;
    scalarMin = 0.0;
    scalarMax = 1.0;
    displaymode = 0.0;
    pointmode = 0.0;
    filePath = "NULL";

}
/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::setFilePath()
{
     QString fn = QFileDialog::getOpenFileName(this, tr("Open File..."),
            QString(), tr("Okc Files (*.txt);;CSV Files (*.las);;All Files (*)"));
    // if users clicked the button "Cancel", fn will be null
    if (fn.isNull()) {
        return;
    }

    try
    {
        filePath =fn.toStdString();
        m_handReduction->clear();
        m_handReduction->setFilePath(filePath);
        m_handReduction->fileRead();
        setFeatureMode(1, 1);
        GLupdate();
    } 
    catch (bad_alloc& ba) 
    {
        QMessageBox::critical(this,
                "Failure to request memory",
                "XmdvTool failed to request enough memory to visualize "
                "the current dataset and has to exit.  Please report "
                "this problem to xmdv@cs.wpi.edu with the description "
                "of your dataset and system configuration.  Thank you! "
                "--Xmdv Group");
        exit(1);
    }

    setFeatDispMode(0);

}

/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::performReduction()
{
    
    try
    {
       m_handReduction->reset();
        m_handReduction->setParams((float)rmin, (float)rmax, (float)scale, (float)eps, (float)sigmin, (float)sigmax, (float)scalarMin, (float)scalarMax);
        m_handReduction->classifyStructFeats();
        GLupdate();
    
    } 
    catch (bad_alloc& ba) 
    {
        QMessageBox::critical(this,
                "Failure to request memory",
                "XmdvTool failed to request enough memory to visualize "
                "the current dataset and has to exit.  Please report "
                "this problem to xmdv@cs.wpi.edu with the description "
                "of your dataset and system configuration.  Thank you! "
                "--Xmdv Group");
        exit(1);
    }


}
/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::setRmin(double rmin)
{
    if(rmin != this->rmin)
    {
        this->rmin = rmin;
       // emit rminChanged();
    }

}

/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::setRmax(double rmax)
{
    if(rmax != this->rmax)
    {
        this->rmax = rmax;
       // emit rmaxChanged();
    }
}

/*****************************************************************************************/
void MainWindow::setScale(double scale)
{
    if(scale != this->scale)
    {
        this->scale = scale;
       // emit rmaxChanged();
    }
}

/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::setEps(double eps)
{
    if(eps != this->eps)
    {
        this->eps = eps;
       // emit epsChanged();
    }
}

/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::setSigmaMin(double sigmin)
{   
    if(sigmin != this->sigmin)
    {
        this->sigmin = sigmin;
        //emit sigminChanged();
    }
}

/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::setSigmaMax(double sigmax)
{
    if(sigmax != this->sigmax)
    {
        this->sigmax = sigmax;
        //emit sigmaxChanged();
    }
}

void MainWindow::setScalarMin(double scalarMin)
{
    if(scalarMin != this->scalarMin)
    {
        this->scalarMin = scalarMin;
        m_handReduction->setParams((float)rmin, (float)rmax, (float)scale, (float)eps, (float)sigmin, (float)sigmax, (float)scalarMin, (float)scalarMax);
        GLupdate();
    }
}

void MainWindow::setScalarMax(double scalarMax)
{
    if(scalarMax != this->scalarMax)
    {
        this->scalarMax = scalarMax;
        m_handReduction->setParams((float)rmin, (float)rmax, (float)scale, (float)eps, (float)sigmin, (float)sigmax, (float)scalarMin, (float)scalarMax);
        GLupdate();
    }
}

/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::setDisplayMode(int displaymode)
{
    this->displaymode = displaymode;
}
/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::setPointMode(int pointmode)
{
    this->pointmode = pointmode;
}
/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::about()
{
    QMessageBox::about(this, tr("About Grabber"),
            tr("The <b>Grabber</b> example demonstrates two approaches for "
               "rendering OpenGL into a Qt pixmap."));
}
/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::createActions()
{
     action_Open = new QAction(this);
   // action_Open->setObjectName(QString::fromUtf8("action_Open"));
    action_Open->setText(tr("Open"));
   // action_Open->setIcon(ICON_OPEN);
    action_Open->setShortcut(tr("Ctrl+F"));
    action_Open->setStatusTip(tr("Open file"));
    connect(action_Open, SIGNAL(triggered()), this, SLOT(setFilePath()));

    exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcuts(QKeySequence::Quit);
    connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

    aboutAct = new QAction(tr("&About"), this);
    connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));

    aboutQtAct = new QAction(tr("About &Qt"), this);
    connect(aboutQtAct, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
}
/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::createMenus()
{
    fileMenu = menuBar()->addMenu(tr("&File"));

    fileMenu->addAction(action_Open);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(aboutAct);
    helpMenu->addAction(aboutQtAct);
}



/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::initVisActions() 
{
        // Initialize the action for flat display
    action_ptRedn = new QAction(this);
    action_ptRedn->setObjectName(QString::fromUtf8("Point Reduction"));
  //  action_lasDisplay->setIcon(ICON_HPI);
    action_ptRedn->setToolTip(tr("Point Reduction"));
    action_ptRedn->setStatusTip(tr("Point Reduction"));
    action_ptRedn->setCheckable(true);
    connect(action_ptRedn, SIGNAL(triggered()), this, SLOT(performReduction()));
    action_ptRedn->setChecked(true);


}

/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::initToolBarRight()
{

}

/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/

void MainWindow::setFeatDispMode(int index)
{
    setFeatureMode(index, 0);
    GLupdate();
}

/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
       glWidget->keyPressEvent(e);
       // QWidget::keyPressEvent(e);
}
/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::createDockWindows()
{
    QDockWidget *dock;
    dock = new QDockWidget(tr(""), this);
    dock->setAllowedAreas(Qt::RightDockWidgetArea);
    dock->setFloating(false);
    addDockWidget(Qt::RightDockWidgetArea, dock);

    QDockWidget *dock1;
    dock1 = new QDockWidget(tr(""), this);
    dock1->setAllowedAreas(Qt::RightDockWidgetArea);
    dock1->setFloating(false);
    addDockWidget(Qt::RightDockWidgetArea, dock1);

    QGroupBox *previewGroupBox;
    QGroupBox *generalOptionsGroupBox;


    //Global Parameter - First Box
    previewGroupBox = new QGroupBox(tr("Parameters"));
    setStyleSheet(QString::fromUtf8("QGroupBox { border: 2px solid red; margin-bottom: 7px;margin-right: 7px; padding: 5px} QGroupBox::title {top:7 ex;left: 10px; subcontrol-origin: border}"));

    m_handleParWindow = new ParmWidget(this);
    QGridLayout *previewLayout = new QGridLayout;

    tensorType = new QComboBox();
    // Fill the items of the ComboBox
    tensorType->addItem("3DVT-GET");
    tensorType->addItem("3DVT");
    tensorType->addItem("3DCM");
	tensorType->addItem("2DGET");
    tensorType->addItem("3DMCM");
    tensorType->addItem("Hessian");
    tensorType->addItem("2DCM");
    connect(tensorType, SIGNAL (currentIndexChanged(int)), this, SLOT (setTensorType(int)));

    previewLayout->addWidget(m_handleParWindow);
    previewLayout->addWidget(tensorType);
    previewGroupBox->setLayout(previewLayout);

    //Structural Classification parameters
    generalOptionsGroupBox = new QGroupBox(tr("Structural Classification"));
    QVBoxLayout *previewLayout1 = new QVBoxLayout;
    m_handlestrucwind = new Strucparam(this);

    QPushButton *train_button = new QPushButton(this);
    train_button->setText(tr("Start"));
    train_button->show();
    connect ( train_button, SIGNAL( clicked() ), this, SLOT( performReduction()) );

    ftdisp2 = new QComboBox();
    // Fill the items of the ComboBox
    ftdisp2->addItem("Intensity");
    ftdisp2->addItem("Saliency");
    ftdisp2->addItem("Surface Variation");
    ftdisp2->addItem("ClCsCp");
    ftdisp2->addItem("Curve Graph");
    ftdisp2->addItem("Sum of Eigenvalues");
    ftdisp2->addItem("Planarity");
    ftdisp2->addItem("Anisotropy");
    ftdisp2->addItem("Sphericity");
    ftdisp2->addItem("Triangulation");
    ftdisp2->addItem("Triangulation Points");
    ftdisp2->addItem("DoN");
    ftdisp2->addItem("Contour");
    ftdisp2->addItem("Tensor Lines");
    ftdisp2->addItem("Height");
    ftdisp2->addItem("Linearity");
    ftdisp2->addItem("Omnivariance");
    ftdisp2->addItem("Eigenentropy");
    ftdisp2->addItem("Labels");
  
    connect(ftdisp2, SIGNAL (currentIndexChanged(int)), this, SLOT (setMeatFeatDispMode(int)));

    QHBoxLayout *hmainLayout = new QHBoxLayout;
    hmainLayout->addWidget(m_handlestrucwind);

    QHBoxLayout *hmainLayout1 = new QHBoxLayout;
    hmainLayout1->addWidget(train_button);
    hmainLayout1->addWidget(ftdisp2);

    previewLayout1->addLayout(hmainLayout);
    previewLayout1->addLayout(hmainLayout1);
    generalOptionsGroupBox->setLayout(previewLayout1);


    dock->setWidget(previewGroupBox);
    dock1->setWidget(generalOptionsGroupBox);

   
}
/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::setFeatureMode(int pointMode, int displayMode)
{
    glWidget->setDisplayMode(displayMode);
    glWidget->setPointMode(pointMode);  
}
/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::setMeatFeatDispMode(int index)
{
    setFeatureMode(index, 1);
    GLupdate();
}

/*****************************************************************************************/
void MainWindow::setTensorType(int index)
{
    m_handReduction->setTensorType(index);
}
/*****************************************************************************************/
/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
void MainWindow::GLupdate()
 {

    glWidget->updateGL();
 }
/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
                                critical curve node and critical disc node
Calls			:
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/
 void MainWindow::GLClear()
 {
    glWidget->winClear();
 }
