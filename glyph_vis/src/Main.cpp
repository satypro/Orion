
#include "FltkView.h"
#include <iostream>

GuiView *fv;
int mode = -1;
using namespace std;

using namespace std;

int main(int argc, char **argv)
{
	fv = new GuiView();
	//~ std::cout << "hello1" << std::endl;
  fv->mainWindow->end();
  fv->mainWindow->resizable(fv->mainWindow);
  fv ->show();   
  return Fl :: run() ;
  return 0;

	/*std::string filePath;
 	feature _objprocess;//  = new feature();
//	_objprocess
	_objprocess.setFilePath(filePath);
	_objprocess.perform();*/
}


