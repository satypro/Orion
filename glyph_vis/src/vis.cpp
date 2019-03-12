#include "vis.h"
#include <GL/gl.h>
#include <iostream>
#include <algorithm>
#include "Color.h"


bool compareFeat(double i, double j) { return i<j; }
/*****************************************************************************************

Function Name		:	Processing::scale	
Purpose	of Function	:	
Input Params		:
Input/output Params	:
Output Params		:
*****************************************************************************************/

bool vis::fileread(std::string sFilePath)
{
	bool error;
	FILE *fp = fopen(sFilePath.c_str(), "r");

  	if(fp == NULL) 
	{
   		printf("Invalid File path. Please select correct file\n");
   		return false;
  	}

  	char buffer[1000];
  //	error = fgets(buffer, 300, fp);
  	error = fgets(buffer, 30, fp);
    sscanf(buffer,"%d ", &cloudSize_);

   // error = fgets(buffer, 30, fp);
    //sscanf(buffer,"%d ", &totDataField_);

    //error = fgets(buffer, 30, fp);
    //sscanf(buffer,"%f ", &sigma_);

    error = fgets(buffer, 30, fp);
    sscanf(buffer,"%f ", &radius_);

    cloud_.resize(cloudSize_);

    cout<<"cloud size "<<cloudSize_<<endl;
    //cout<<"DataField size "<<totDataField_<<endl;
    //cout<<"sigma "<<sigma_<<endl;
    cout<<"radius "<<radius_<<endl;

    curvVal.resize(cloudSize_);

    for(int i = 0; i < cloudSize_; i ++)
   		{
    		error =  fgets(buffer, 800, fp);
    	sscanf(buffer,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &cloud_[i].XYZ[0],
            &cloud_[i].XYZ[1], &cloud_[i].XYZ[2], &cloud_[i].eval[0], 
          &cloud_[i].eval[1], &cloud_[i].eval[2], &cloud_[i].evec[0], &cloud_[i].evec[1], &cloud_[i].evec[2], &cloud_[i].evec[3],
          &cloud_[i].evec[4], &cloud_[i].evec[5], &cloud_[i].evec[6], &cloud_[i].evec[7], &cloud_[i].evec[8], &cloud_[i].UV[0],
          &cloud_[i].UV[1], &cloud_[i].ABC[0], &cloud_[i].ABC[1], &cloud_[i].ABC[2], &cloud_[i].clcpcs[0], 
          &cloud_[i].clcpcs[1], &cloud_[i].clcpcs[2]);

   		}

     int l_p, s_p, p_p;
     
       l_p = 0;
       s_p = 0;
       p_p = 0;
  for(size_t i =0; i < cloudSize_; i++)
   {


        if(cloud_[i].clcpcs[0] >= cloud_[i].clcpcs[1] && cloud_[i].clcpcs[0] >= cloud_[i].clcpcs[2])
            p_p++;

        if(cloud_[i].clcpcs[1] >= cloud_[i].clcpcs[0] && cloud_[i].clcpcs[1] >= cloud_[i].clcpcs[2])
            l_p++;

        if(cloud_[i].clcpcs[2] >= cloud_[i].clcpcs[0] && cloud_[i].clcpcs[2] >= cloud_[i].clcpcs[1])
            s_p++;

   }

  cout<<"total Point "<<cloudSize_<<endl;
  cout<<"Point "<<p_p<<endl;
  cout<<"line "<<l_p<<endl;
  cout<<"surface "<<s_p<<endl;

}

/*****************************************************************************************

Function Name   : Processing::scale 
Purpose of Function : 
Input Params    :
Input/output Params :
Output Params   :
*****************************************************************************************/
bool vis::cloudDisplay()
{
  if(cloud_.size() == 0 ) 
  {
    cout<<"Point cloud size is zero"<<endl;
    return false;
  }

  glPointSize(2.0);

  Color currentcolor ;

  glBegin(GL_POINTS);

    for (size_t i = 0 ; i < cloud_.size() ; i++)
    {
      //glColor3d(cloud_[i].clcpcs[1], cloud_[i].clcpcs[2], cloud_[i].clcpcs[0]) ;
     
      glColor3d(cloud_[i].clcpcs[1], cloud_[i].clcpcs[2], cloud_[i].clcpcs[0]) ;
      glVertex3d(cloud_[i].XYZ[0], cloud_[i].XYZ[1], cloud_[i].XYZ[2]);     
    }

  glEnd(); 

}

/*****************************************************************************************

Function Name   : Processing::scale 
Purpose of Function : 
Input Params    :
Input/output Params :
Output Params   :
*****************************************************************************************/
bool vis::colorClCpCs()
{
  if(cloud_.size() == 0 ) 
  {
    cout<<"Point cloud size is zero"<<endl;
    return false;
  }
  glPointSize(2.0);
  Color currentcolor ;

  glBegin(GL_POINTS);

  for (size_t i = 0 ; i < cloud_.size() ; i++)
  {
    if( cloud_[i].XYZ[1] >1.65 && cloud_[i].XYZ[1] <1.85 && cloud_[i].XYZ[0] <0.61 && cloud_[i].XYZ[0] > 0.30)

    {
    glColor3d(cloud_[i].clcpcs[1], cloud_[i].clcpcs[2], cloud_[i].clcpcs[0]) ;
    //glColor3d(currentcolor.red(), currentcolor.green(), currentcolor.blue()) ;
    //glColor3d(cloud_[i].clcpcs[0], cloud_[i].clcpcs[2], cloud_[i].clcpcs[1]) ;
    //glColor3d(cloud_[i].clcpcs[2], cloud_[i].clcpcs[0], cloud_[i].clcpcs[1]) ;
    glVertex3d(cloud_[i].XYZ[0], cloud_[i].XYZ[1], cloud_[i].XYZ[2]);     
  }
}
  
  glEnd();  
}
/*****************************************************************************************

Function Name   : Processing::scale 
Purpose of Function : 
Input Params    :
Input/output Params :
Output Params   :
*****************************************************************************************/
void vis::drawSingleGlyph(int idx)
{
Color currentcolor ;
  double absevals[3];
  int i,j, k;
  limnPolyData *lpd = limnPolyDataNew();
  double eps=1e-4;
  int glyphRes=20; 
  unsigned int zone;
  double gltrans[16];

  limnPolyDataSpiralBetterquadric(lpd, (1 << limnPolyDataInfoNorm), cloud_[idx].ABC[0], cloud_[idx].ABC[1], cloud_[idx].ABC[2], 0.0, 2*glyphRes, glyphRes);
  limnPolyDataVertexNormals(lpd);

  for (k=0; k<3; k++)
    absevals[k]=fabs(cloud_[idx].eval[k]);

 double trans[16]={absevals[0]*cloud_[idx].evec[0], absevals[1]*cloud_[idx].evec[3],
                    absevals[2]*cloud_[idx].evec[6], 0,
                    absevals[0]*cloud_[idx].evec[1], absevals[1]*cloud_[idx].evec[4],
                    absevals[2]*cloud_[idx].evec[7], 0,
                    absevals[0]*cloud_[idx].evec[2], absevals[1]*cloud_[idx].evec[5],
                    absevals[2]*cloud_[idx].evec[8], 0,
                    0, 0, 0, 1};

double uv[2];
uv[0] = cloud_[idx].UV[0];
uv[1] = cloud_[idx].UV[1];
 zone=tenGlyphBqdZoneUv(uv);

  if (0==zone || 5==zone || 6==zone || 7==zone || 8==zone) {
    /* we need an additional rotation */
    double ZtoX[16]={ 0,0,1,0,
                      0,1,0,0,
                     -1,0,0,0,
                      0,0,0,1 };
    ell_4m_mul_d(trans, trans, ZtoX);
  }

  ELL_4M_TRANSPOSE(gltrans, trans);

   glPushMatrix() ;
  glTranslatef(( cloud_[idx].XYZ[0]), ( cloud_[idx].XYZ[1]), (cloud_[idx].XYZ[2]));

  glScalef(0.02, 0.02, 0.02);
  //  glScalef(0.002, 0.002, 0.002);
  // glScalef(0.006, 0.006, 0.006);
  //glScalef(0.0015, 0.0015, 0.0015);
  glMultMatrixd(gltrans);

 
  glBegin(GL_TRIANGLE_STRIP);

   //glColor3d(0.08, 0.20, 0.02) ;
 //  glColor3d(cloud_[idx].clcpcs[0], cloud_[idx].clcpcs[1], cloud_[idx].clcpcs[2]) ;
glColor3d(cloud_[idx].clcpcs[1], cloud_[idx].clcpcs[2], cloud_[idx].clcpcs[0]) ;
  for(i = 0; i < lpd->indxNum; i++)
  {
    j = lpd->indx[i]*4;
    glNormal3f(lpd->xyzw[j], lpd->xyzw[j+1], lpd->xyzw[j+2]) ;
     glVertex3f(lpd->xyzw[j], lpd->xyzw[j +1],lpd->xyzw[j +2] );
  }
  glEnd();

  glPopMatrix();



}
/*****************************************************************************************

Function Name   : Processing::scale 
Purpose of Function : 
Input Params    :
Input/output Params :
Output Params   :
*****************************************************************************************/
void vis::drawGlyph()
{
  if(cloud_.size() == 0 ) 
  {
    cout<<"Point cloud size is zero"<<endl;
    return ;
  }

  float minXYZ2 = 1000.0;
  float minXYZ1 = 1000.0;
  float minXYZ0 = 1000.0;
  float maxXYZ2 = -1.0;
  float maxXYZ0 = -1.0;
  float maxXYZ1 = -1.0;
  for (size_t i = 0 ; i <cloud_.size(); i++)
  {
	if(cloud_[i].XYZ[2] > maxXYZ2)
		maxXYZ2 = cloud_[i].XYZ[2];
	if(cloud_[i].XYZ[2] < minXYZ2)
		minXYZ2 = cloud_[i].XYZ[2];
	if(cloud_[i].XYZ[1] > maxXYZ1)
		maxXYZ1 = cloud_[i].XYZ[1];
	if(cloud_[i].XYZ[1] < minXYZ1)
		minXYZ1 = cloud_[i].XYZ[1];
	if(cloud_[i].XYZ[0] > maxXYZ0)
		maxXYZ0 = cloud_[i].XYZ[0];
	if(cloud_[i].XYZ[0] < minXYZ0)
		minXYZ0 = cloud_[i].XYZ[0];
    //~ if( cloud_[i].XYZ[2] > 0.0 && cloud_[i].XYZ[1] < 6 )// && cloud_[i].XYZ[1] > 0.0)// && cloud_[i].XYZ[1] < 0.6)// && cloud_[i].XYZ[0] > 0.8 && cloud_[i].XYZ[0] < 1.2)
    //~ {
      drawSingleGlyph(i);
    //~ }

  }
  std::cout<< "2: " << minXYZ2 << "," << maxXYZ2 << "1: " << minXYZ1<<","<<maxXYZ1<<";0:"<<minXYZ0<<","<<maxXYZ0<<std::endl;

}
/*****************************************************************************************

Function Name   : Processing::scale 
Purpose of Function : 
Input Params    :
Input/output Params :
Output Params   :
*****************************************************************************************/
void vis::drawColorMap(int type)
{
  //cout<<"_tempTensorAtt "<<_tempTensorAtt.size()<<endl;

  //cout<<"_tensorAtt "<<_tensoratt->size()<<endl;
  if(cloud_.size()  == 0)
    return;

  else
  {
     cout<<"type "<<type<<endl;

     if(type == 0)
     {
        //cout<<"point cloud"<<endl;
        cloudDisplay();
     }
     else if(type == 1)
     {
        cout << "ClCsCp color map" << endl; 
          colorClCpCs();
     }

    else if(type == 2)
     {
        cout << "SaliencyVals color map" << endl; 
       // colorClCpCs();
        drawGlyph();
        //colorSaliencyVals();
        //drawCriticalGlyph();
     }
     else
        cout << "Wrong parameter for color map" << endl;

  }

   return;  
}
