#include <QApplication>
#include <QMessageBox>
#include <QGLFormat>
#include <QGLWidget>

//#include "main/Globals.h"
#include "main/mainwindow.h"


#include <vector>
#include <stdio.h>
#include <iostream>


//Globals g_globals;
using namespace std;

int main(int argc, char *argv[])
{

/*classifier *handle_classifier;
	handle_classifier = new classifier();

	string filepath = "/home/iiit-b/Desktop/tool/data/fd.okc";

  handle_classifier->setFilePath(filepath);
  handle_classifier->readFeature();

  handle_classifier->intialization();
*/

	QApplication a(argc, argv);

	// Check whether the current system configuration supports the OpenGL
	if (!QGLFormat::hasOpenGL() ) {
		QMessageBox::information(0, "XmdvTool",
				"This system does not support OpenGL rendering.  XmdvTool cannot start.");
		return -1;
	}

	QGLWidget* tryGLWidget = new QGLWidget( (QWidget*)0 );
	if ( !tryGLWidget->isValid() ) {
		QMessageBox::information(0, "XmdvTool",
				"This system does not support OpenGL rendering.  XmdvTool cannot start.");
		delete tryGLWidget;
		return -1;
	}
	delete tryGLWidget;

	MainWindow w;
	w.show();
	a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
	//w.openInitialDatasets();
	return a.exec();

	return 0;


}
