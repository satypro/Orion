/*
 * FltkForm.h
 */

#ifndef FltkForm_h_
#define FltkForm_h_

#include <FL/Fl.H>
#include <Fl/Fl_Gl_Window.H>
#include "FastTrackball.h"

#include <cstdlib>
#include <iostream>
 #include <teem/ten.h>

#include "vis.h"

 using namespace std;
/*****************************************************************************************

Class Name		:	FltkForm	
Purpose			:	rendering of 3-D point cloud into a normal FLTK window using openGL. 
Created by		:	Beena	
Created date		:	27/12/2013
Modification		:
Modified Function	:

*****************************************************************************************/

class FltkForm : public Fl_Gl_Window
	{
 	protected:
  	//void (*userDrawFunc)();

 	public:
  	FltkForm(int x, int y, int w, int h, const char *l = 0): Fl_Gl_Window(x, y, w, h, l), xShift(0.0), yShift(0.0),size(1.0), _objprocess(new vis[1]),
    		enableTranslation(1), enableZoom(1), enableZoomIn(0), enableZoomOut(0), enableRotation(1), displaymode(1), pointmode(0), curvemode(0), discmode(0), lpd(limnPolyDataNew())
 		{ 

      		this->mode( FL_RGB8 | FL_ALPHA | FL_DOUBLE | FL_DEPTH ) ;
		end() ;
    		}

  	~FltkForm() { }

  void	drawGlyph();
	void drawAxis(void);
  	void draw(void);
  	void getVersions(void) ;
  	void resize(int, int, int, int);
  	void reset(void);
  	void setInitSize() ;
  	int handle(int);
  	int handleMouse(int, int, int, int);
  	int handleMouseWheel(int);
  	int handleKey(int, int);
	int process();
	void setFilePath(std::string filePath) { _filePath.assign(filePath); }
	void render();
	void drawSpectrum();
    void enableLights( void );
  void disableLights( void );
  void getGlyphParam();
  void getGlyphParam2();
	// void drawFunc(void(*fp)()){ userDrawFunc = fp; }
	private:
  vis *_objprocess ;
	std::string _filePath;
	double size;
  	Trackball trackball ;
  	float xShift, yShift ;
  	int oldPosX, oldPosY, newPosX, newPosY ;
  	int enableTranslation, enableZoom, enableZoomIn, enableZoomOut, enableRotation;
	int displaymode, pointmode, curvemode, discmode;
	 double gltrans[16];
    limnPolyData *lpd;
  
};

#endif
