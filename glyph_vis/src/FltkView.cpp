// generated by Fast Light User Interface Designer (fluid) version 1.0300

#include "FltkView.h"
#include <iostream>

Fl_Menu_Item GuiView::menu_menu[] = {
 {"File", 0,  0, 0, 64, FL_NORMAL_LABEL, 0, 14, 0},
 {"Open", 0,  (Fl_Callback*)scb_open, 0, 0, FL_NORMAL_LABEL, 0, 14, 0},
 {"Quit", 0,  (Fl_Callback*)scb_quit, 0, 0, FL_NORMAL_LABEL, 0, 14, 0},
 {0,0,0,0,0,0,0,0,0},
 {"Help", 0,  0, 0, 0, FL_NORMAL_LABEL, 0, 14, 0},
 {0,0,0,0,0,0,0,0,0}
};
Fl_Menu_Item* GuiView::file = GuiView::menu_menu + 0;
Fl_Menu_Item* GuiView::open = GuiView::menu_menu + 1;
Fl_Menu_Item* GuiView::quit = GuiView::menu_menu + 2;

GuiView::GuiView() {
  { mainWindow = new Fl_Double_Window(930, 645, "Visualization Tool");
    mainWindow->box(FL_UP_BOX);
    mainWindow->labelsize(11);
    mainWindow->user_data((void*)(this));
    { menu = new Fl_Menu_Bar(1, 1, 924, 23);
      menu->box(FL_ENGRAVED_BOX);
      menu->labelsize(11);
      menu->menu(menu_menu);
    } // Fl_Menu_Bar* menu
    { curve = new FltkForm(160, 27, 615, 615, "Curve");
      curve->box(FL_ENGRAVED_BOX);
      curve->color(FL_BACKGROUND_COLOR);
      curve->selection_color(FL_BACKGROUND_COLOR);
      curve->labeltype(FL_NO_LABEL);
      curve->labelfont(0);
      curve->labelsize(16);
      curve->labelcolor((Fl_Color)2);
      curve->align(Fl_Align(33));
      curve->when(FL_WHEN_RELEASE);
    } // FltkForm* curve
    Fl::scheme("plastic");
    mainWindow->end();
  } // Fl_Double_Window* mainWindow
}

void GuiView::show() {
  mainWindow->show();
}

void GuiView::scb_open(Fl_Widget *o, void *v) {
  ((GuiView *)v)->cb_menu_open();
}

void GuiView::scb_quit(Fl_Widget *o, void *v) {
  exit(0);
}

void GuiView::cb_menu_open() {
  Fl_File_Chooser chooser("../data/", "(*.{las,ply,txt})", Fl_File_Chooser::SINGLE, "Title Of Chooser"); 
  chooser.show();
  while(chooser.shown()) 
   { Fl::wait(); }
  
  std::string FileName ;
  extern GuiView *fv;
  FileName = (char *) chooser.value();
  extern int mode;
  mode = 1;
  fv->curve->setFilePath(FileName);
  //extern void Fileread(std::string FileName);
  //Fileread( FileName);
  fv->curve->process();
  fv->curve->redraw();
}
