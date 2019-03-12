#include <GL/gl.h>


#include "FltkForm.h"
#include "FltkView.h"

#include "Color.h"

#include <iostream>
/*****************************************************************************************
******************************************************************************************

Global Variables

******************************************************************************************
*****************************************************************************************/
GLdouble x_ang, y_ang ;
int enable_fullscreen = 0 ;
extern GuiView *fv ;
int max_index = 1, max_limit ;

/*****************************************************************************************

Function Name   : FltkForm::setInitSize 
Purpose of Function : sets an orthographic projection for OpenGL display

*****************************************************************************************/

void FltkForm :: setInitSize(void) 
  {
    glOrtho(-2, 2, -2, 2, -200, 200) ;
  }

/*****************************************************************************************
Function Name   : FltkForm::handleMouse 
Purpose of Function : routines for Mouse 

*****************************************************************************************/
void FltkForm :: enableLights( void ) 
{ 

  GLfloat light0_ambient[] =  { 0., 0., 1, 1 } ;
  GLfloat light0_diffuse[] = { 1, 1, 1, 1.0 };
  GLfloat light0_specular[] = { 1, 1, 1, 1.0 };
  GLfloat light0_position[] = { 1.0, 1.0, 1.0, 0.0 };

  glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light0_position);

  GLfloat light1_ambient[] =  { 1.0, 0.2, 0.2, 1 } ;
  GLfloat light1_diffuse[] = { 1,1, 1, 1.0 };
  GLfloat light1_specular[] = { 1, 1, 1, 1.0 };
  GLfloat light1_position[] = { -1.0, -1.0, -1.0, 0.0 };

  glLightfv(GL_LIGHT1, GL_AMBIENT, light1_ambient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
  glLightfv(GL_LIGHT1, GL_SPECULAR, light1_specular);
  glLightfv(GL_LIGHT1, GL_POSITION, light1_position);

  GLfloat light2_ambient[] =  { 0.0, 1.0, 0.2, 1 } ;
  GLfloat light2_diffuse[] = { 1, 1, 1, 1.0 };
  GLfloat light2_specular[] = { 1, 1, 1, 0.0 };
  GLfloat light2_position[] = { 0.0, 0.0, 0.0, 0.0 };

  glLightfv(GL_LIGHT2, GL_AMBIENT, light2_ambient);
  glLightfv(GL_LIGHT2, GL_DIFFUSE, light2_diffuse);
  glLightfv(GL_LIGHT2, GL_SPECULAR, light2_specular);
  glLightfv(GL_LIGHT2, GL_POSITION, light2_position);

  GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat mat_shininess[] = { 80.0 };

  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
  
  glShadeModel (GL_SMOOTH);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_LIGHT2);
  glEnable(GL_DEPTH_TEST);

  glEnable(GL_NORMALIZE); 

  //glColorMaterial( GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glEnable( GL_COLOR_MATERIAL ) ;
}
/*****************************************************************************************
Function Name   : FltkForm::handleMouse 
Purpose of Function : routines for Mouse 

*****************************************************************************************/
void FltkForm :: disableLights( void ) {
  glDisable( GL_LIGHTING ) ;
  glDisable( GL_LIGHT0 ) ; 
  glDisable( GL_COLOR_MATERIAL ) ;
}

/*****************************************************************************************

Function Name		:	FltkForm::process	
Purpose	of Function	:		:

*****************************************************************************************/
int FltkForm ::process()
	{
	//_objprocess[0].clear();
	pointmode = 0;
	_objprocess[0].fileread(_filePath);
	_objprocess[0].drawColorMap(0);

	}

/*****************************************************************************************

Function Name		:	FltkForm::draw	
Purpose	of Function	:	draw the display into a FLTK-Window.
Author/date		:	Beena
Modified by/date	:	27/12/2013
Calls			:	FltkForm::getVersions(), FltkForm::setInitSize(), FltkForm::userDrawFunc(), trackball.rotationMatrix
Input Params		:
Output Params		:
Return			:
Remarks			:

*****************************************************************************************/
void FltkForm ::render()
	{
      _objprocess[0].drawColorMap(pointmode);
	return ;


	}



/*****************************************************************************************

Function Name		:	FltkForm::draw	
Purpose	of Function	:	draw the display into a FLTK-Window.
Author/date		:	Beena
Modified by/date	:	27/12/2013
Calls			:	FltkForm::getVersions(), FltkForm::setInitSize(), FltkForm::userDrawFunc(), trackball.rotationMatrix
Input Params		:
Output Params		:
Return			:
Remarks			:

*****************************************************************************************/
void FltkForm :: draw(void) 
	{
  	if (!valid())
 		{
      		valid( 1 );   
      		glClearColor(1, 1, 1, 1) ;  //0.5 0 0 1
         // glClearColor(0.0, 0.0, 0.0, 1) ; 
      		glEnable( GL_LINE_SMOOTH );
      		// Depth Buffer Setup
      		glClearDepth(1.0f);      
      		// Enables Depth Testing
      		glEnable(GL_DEPTH_TEST); 
         // enableLights();
         // glEnable( GL_LIGHTING ) ;
         // glEnable(GL_NORMALIZE); 
      		// The Type Of Depth Testing To Do
      		glDepthFunc(GL_LEQUAL);  
      		// Really Nice Perspective Calculations
      		glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	
      		getVersions() ;
      		glLoadIdentity( ) ;
      		glViewport( 0, 0, w(), h() );
      		// Orthographic projection
      		glMatrixMode( GL_PROJECTION ) ;
      		glLoadIdentity() ;
      		setInitSize() ;
      		glMatrixMode( GL_MODELVIEW ) ;
      		glLoadIdentity() ;
		  //getGlyphParam();

    		}
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ); 
	//drawSpectrum();
  	glPushMatrix();
  	static float rotation_transform[4][4] ;
  	// TODO Fix trackball.
  	trackball.rotationMatrix( rotation_transform ) ;
  	glScalef( float( size ), float( size ), float( size ) );
  	glTranslatef( xShift/size, -yShift/size, 0 );
  	glMultMatrixf( &rotation_transform[0][0] ) ;
	
  	render(); 
  	glPopMatrix();  
	}

/*****************************************************************************************

Function Name		:	FltkForm::getVersions	
Purpose	of Function	:	display the OpenGL version
Author/date		:
Modified by/date	:	27/12/2013
Calls			:	
Input Params		:
Output Params		:
Return			:
Remarks			:

*****************************************************************************************/

void FltkForm :: getVersions(void) 
	{
  	const char* verstr = (const char*)glGetString(GL_VERSION) ;

  	if (verstr) 
    		std :: cerr << "GL_VERSION : " << verstr << std :: endl ;
  	else
    		std :: cerr << "Error in determining OpenGL version" << std :: endl ;

  	return ;
	}

/*****************************************************************************************

Function Name		:	FltkForm::resize	
Purpose	of Function	:	change size and position of FL_GL_Window(widget).
Author/date		:
Modified by/date	:	27/12/2013
Calls			:	
Input Params		:	X,Y - new position relative to the parent window 
				W - new width of the window, 
				H - new height of the window
Output Params		:
Return			:
Remarks			:

*****************************************************************************************/

void FltkForm :: resize(
	int X,
	int Y,
 	int W,
	int H)
 	{
  	// Let FLTK resize the widget
  	Fl_Gl_Window::resize(X, Y, W, H);
  
  	if(!valid())
 		{
    		glViewport(0, 0, W, H);
    		glEnable(GL_DEPTH_TEST);
  		}
	}

/*****************************************************************************************

Function Name		:	FltkForm::reset	
Purpose	of Function	:	reset the parameters to their default values 
Author/date		:	
Modified by/date	:	27/12/2013
Calls			:	FltkForm::redraw()	
Input Params		:
Output Params		:
Return			:
Remarks			:

*****************************************************************************************/

void FltkForm :: reset(void) 
	{
  	xShift = 0.0 ;
  	yShift = 0.0 ;
  	size = 1.0;
  	oldPosX = oldPosY = newPosX = newPosY = 0;
  	this->redraw();
  	return;
	}



/*****************************************************************************************

Function Name		:	FltkForm::handle	
Purpose	of Function	:	Routines for mouse/keyboard
Author/date		:
Modified by/date	:	27/12/2013
Calls			:	FltkForm::handleMouseWheel, FltkForm::handleMouse, FltkForm::handleKey
Input Params		:	int event - Type of event (Keyboard, Mouse, MouseWheel, etc. )
Output Params		:	0 - sucessfully executed
				1 - not sucessfully
Return			:
Remarks			:

*****************************************************************************************/
int FltkForm :: handle(int event) 
	{
  	switch(event)
 		{
  		case FL_ENTER:
    			Fl::focus() ;
    			return 1 ;
    
  		case FL_MOVE:
  
		case FL_MOUSEWHEEL:
			return handleMouseWheel(event);
  		
		case FL_LEAVE:
    			return 1;
    
  		case FL_PUSH:

  		case FL_RELEASE:

  		case FL_DRAG:
   			return handleMouse(event, Fl::event_button(), Fl::event_x(),Fl::event_y());
    
  		case FL_SHORTCUT:

  		case FL_KEYBOARD:
    			Fl::focus() ;
			return handleKey(event, Fl::event_key());
    
  		default:
    			return Fl_Window::handle(event);
  		}
  	return 0;
	}

/*****************************************************************************************

Function Name		:	FltkForm::handleKey	
Purpose	of Function	:	routines for keyboard
Author/date		:
Modified by/date	:	27/12/2013
Calls			:	FltkForm::redraw(), fv->mainWindow->fullscreen(), fv->mainWindow->fullscreen_off	
Input Params		:	int event - type of Keyboard event
				int key
Output Params		:	1 - sucessfully
				0 - not sucessfully
Return			:

Remarks			:

*****************************************************************************************/

int FltkForm :: handleKey(int event, int key) 
{
  switch(key)
	 	{		
      case 'p':

        case 'P':
          pointmode = 0 ;
          this->redraw() ;
          return 1 ;

        case 'c':

        case 'C':
          pointmode = 1 ;
          this->redraw() ;
          return 1 ;

        case 's':

        case 'S':
          pointmode = 2 ;
          this->redraw() ;
          return 1 ;

        case 'u':

        case 'U':
          pointmode = 3 ;
          this->redraw() ;
          return 1 ;

        case 't':

        case 'T':
          pointmode = 4 ;
          this->redraw() ;
          return 1 ;

        case 'x':

        case 'X':
          pointmode = 5 ;
          this->redraw() ;
          return 1 ;


		case 'r':

  		case 'R':
    			enableRotation = 1-enableRotation ;
    			this->redraw() ;
    			return 1 ;
    
  		case 'f':
  
		case 'F':
    			enable_fullscreen = 1-enable_fullscreen ;
    			if (enable_fullscreen)
			 	fv->mainWindow->fullscreen() ;
    			else
 				fv->mainWindow->fullscreen_off(100,100,600,600) ;
    			return 1 ;
    
  		case '=':
    			if (enableZoom)
 				{
      				size = (size < 28.0) ? size+0.2 : 28 ;
      				this->redraw();
    				}
    			return 1;

  		case '-':
    			if (enableZoom) 
				{
      				size = (size > 0.6) ? size-0.1 : 0.6 ;
      				this->redraw();
    				}
    			return 1;
    
  		case FL_Escape:
    			exit(0) ;
    
  		case FL_Up:
    			max_index = (max_index < max_limit) ?
      			max_index+1 : max_limit ;
    			this->redraw() ;
    			return 1 ;

  		case FL_Down:
    			max_index = (max_index > 1) ? max_index-1 : 1 ;
    			this->redraw() ;
    			return 1 ;

  		default:
      			return 0;
    		}
	}

/*****************************************************************************************

Function Name		:	FltkForm::handleMouseWheel	
Purpose	of Function	:	routines for MouseWheel movement
Author/date		:
Modified by/date	:	27/12/2013
Calls			:	FltkForm::redraw()	
Input Params		:	int event - type of MouseWheel event
Output Params		:	0 - sucessfully
Return			:
Remarks			:

*****************************************************************************************/
int FltkForm :: handleMouseWheel(
	int event)
 	{
  	int ret = 0;
  	if(!enableZoomOut && !enableZoomIn)
 		return 1 ;

  	if(event == FL_MOUSEWHEEL && enableZoomOut == 1)
    		{
      		size = (size < 18.0) ? size+ 0.1 : 5.0 ;
      		this->redraw();
    		}
   
  	if ( event == FL_MOUSEWHEEL && enableZoomIn == 1)
    		{
      		size = (size > 0.2) ? size-0.1 : 0.2 ;
      		this->redraw();
    		}

  	return 0;

	}

/*****************************************************************************************

Function Name		:	FltkForm::handleMouse	
Purpose	of Function	:	routines for Mouse
Author/date		:
Modified by/date	:	27/12/2013
Calls			:	FltkForm::redraw(), fv->mainWindow->fullscreen(), fv->mainWindow->fullscreen_off	
Input Params		:	int event - type of Mouse event
				int button - Type of Mouse button
				xx, yy - Mouse position
Output Params		:	1 - sucessfully
				0 - not sucessfully
Return			:
Remarks			:

*****************************************************************************************/
int FltkForm :: handleMouse(
	int event, 
	int button, 
	int xx, 
	int yy) 
	{
  
  	int ret = 0;
  	
	switch(button)
 		{

    		// Rotation
    		case 1:
      			if (!enableRotation) 
				break ;
      			ret = 1 ;
      			if ( event == FL_PUSH )
				{
	  			// Get the initial mouse position
	  			// and make sure the trackball stops.
	  			x_ang = xx ;
	  			y_ang = yy ;
	  			trackball.rotate( 0,0,0,0 ) ;
				}
      			else if (( event == FL_DRAG ) )
				{
	  			if ( enableRotation == 1 )
	    				trackball.rotate((2.0 * x_ang - w()) / float(w()),(h() - 2.0 * y_ang) / float(h()),
			     			        (2.0 * xx - w()) / float(w()), (h() - 2.0 * yy) / float(h()));
	  			x_ang = xx ; y_ang = yy ;
	  			this->redraw();
				}
      			break ;

      		// Translation 
    		case 3:
      			if (!enableTranslation) 
				break ;
      			ret = 1;
      			if (event == FL_PUSH) 
				{

	  			oldPosX = newPosX = xx ;
	  			oldPosY = newPosY = yy ;
      				}
      			else if ((event == FL_DRAG) || (event == FL_RELEASE)) 
				{
				newPosX = xx ;
				newPosY = yy ;
				if (enableTranslation) 
					{
	  				// ortho_width and ortho_height come from 
	  				// the orthographic projection dimensions. 
	  				// If they should change, the developer
	  				// should update these two variables.
	  				const float ortho_width = 2 ;
	  				//const float ortho_height = 2 ;
	  
	  				xShift += ortho_width*(newPosX - oldPosX)/double(w()) ;
	      				yShift += ortho_width*(newPosY - oldPosY)/double(h()) ;
	    				}
	  			oldPosX = xx ;
	  			oldPosY = yy ;
	  			this->redraw();
				}
      			break;
    		default:
      			ret = 0;
      			break;
    		}
  	return ret;
	}

