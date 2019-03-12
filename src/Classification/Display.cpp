#include "Display.h"
#include "Color.h"
#include <GL/gl.h>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>

/*****************************************************************************************

Function Name		:	PointClassificaton::probMeasure	
Purpose	of Function	:	
Calls			:	PointClassificaton::CurvatureEstimation
Input Params		:	
Output Params		:	T *curve, T *disc, T *spherical
Return			:	int
Remarks			:

*****************************************************************************************/

static int buildingNo = 1;
#define TENSORLINE_IMPLEMENTATION_TYPE 1    // 0-> Streamline implementation ;; 1-> Weighted Neighbors

bool DisplayPoints::lasDisplay()
{
	if(_inCloud->points.size() <= 0)
		return false;

    float minX = 100;
    float minY = 100;
    float minZ = 100;
    float maxX = -100;
    float maxY = -100;
    float maxZ = -100;

    // Intensity Map
    glBegin(GL_POINTS);

    for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
            double temp = _intensity[i];
            glColor3d(temp, temp, temp) ;
//            if(_probval[i].label==5)
                glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
            if(_inCloud->points[i].x < minX)
                minX = _inCloud->points[i].x;
            if(_inCloud->points[i].y < minY)
                minY = _inCloud->points[i].y;
            if(_inCloud->points[i].z < minZ)
                minZ = _inCloud->points[i].z;
            if(_inCloud->points[i].x > maxX)
                maxX = _inCloud->points[i].x;
            if(_inCloud->points[i].y > maxY)
                maxY = _inCloud->points[i].y;
            if(_inCloud->points[i].z > maxZ)
                maxZ = _inCloud->points[i].z;
        }
    glEnd();

//    std::cout << "(" << minX  << ","  <<  maxX  <<  ") "  <<  "("  <<  minY  << ","  <<  maxY  <<  ") "  <<  "("  <<  minZ  << ","  <<  maxZ  <<  ") " << std::endl;
	return true;
}

// 0 -> horizontalgable 1-> Vertical gable
void DisplayPoints::smoothTensorLines(std::string filename, int roof_topology_type) {
    // Read tensorline data from file and smoothen it
    ifstream tensorlinesFile;
    tensorlinesFile.open(filename.c_str());
    int numLines;
    int ndx,id;
    std::vector< std::vector<Point> > tls;
    std::vector<Point> temp;
//    cout << "numline = " << numLines <<endl;

//    while(numLines--) {
//        tensorlinesFile >> ndx >> id;
//        if(id==-1) {
//            tls.push_back(temp);
//            temp.clear();
//        } else{
//            Point a(_inCloud->points[id].x,_inCloud->points[id].y,_inCloud->points[id].z);
//            temp.push_back(a);
//        }
//    }

    std::string word;
    while(tensorlinesFile >> word) {

        if(word.compare("POSITION:")==0) {
            tls.push_back(temp);
            temp.clear();
            tensorlinesFile >> word;
        } else {
            int p = atoi( word.c_str() );
            temp.push_back(Point(_inCloud->points[p].x,_inCloud->points[p].y,_inCloud->points[p].z));
        }
    }
    tls.push_back(temp);
    temp.clear();

    tensorlinesFile.close();

    cout << "Building no = " << buildingNo << endl;

    Point a0,a1,b0,b1,c0,c1,d0,d1,e0,e1,f0,f1,g0,g1;
    for(int i=0;i<tls.size();i++) {
        if(tls[i].size()<2){continue;}
        else {
            Line3 line;
            linear_least_squares_fitting_3(tls[i].begin(),tls[i].end(),line, CGAL::Dimension_tag<0>());

            float error = 0;
            for(int k=0;k<tls[i].size();k++) {
                error += squared_distance(tls[i][k],line);
            }
            errorTL.push_back(error);
            cout << error << endl;

            Point p = line.projection(tls[i][0]);
            Point q = line.projection(tls[i][1]);
//            Point p = tls[i][0];
//            Point q = tls[i][tls[i].size()-1];
//            cout << p << " -------------- " << q << endl;
            Line temp;
            temp.p[0] = p.x(); temp.p[1] = p.y(); temp.p[2] = p.z();
            temp.q[0] = q.x(); temp.q[1] = q.y(); temp.q[2] = q.z();

            if(i==1) {
                a0 = Point(p);
                a1 = Point(q);
            }else if(i==2){
                b0 = Point(p);
                b1 = Point(q);
            }else if(i==3){
                c0 = Point(p);
                c1 = Point(q);
            }else if(i==4){
                d0 = Point(p);
                d1 = Point(q);
            }else if(i==5){
                e0 = Point(p);
                e1 = Point(q);
            }else if(i==6){
                f0 = Point(p);
                f1 = Point(q);
            }else if(i==7){
                g0 = Point(p);
                g1 = Point(q);
            }

            _smoothTensorLines.push_back(temp);
        }
    }
    buildingNo++;

    // Horizontal gable
    if(roof_topology_type==0) {
        Point p1((c0.x()+e1.x()+d0.x())/3,(c0.y()+e1.y()+d0.y())/3,(c0.z()+e1.z()+d0.z())/3);
        Point p2((c1.x()+g1.x()+b0.x())/3,(c1.y()+g1.y()+b0.y())/3,(c1.z()+g1.z()+b0.z())/3);
        Point p3((f0.x()+e0.x())/2,(f0.y()+e0.y())/2,(f0.z()+e0.z())/2);
        Point p4((f1.x()+g0.x())/2,(f1.y()+g0.y())/2,(f1.z()+g0.z())/2);
        Point p5((d1.x()+a0.x())/2,(d1.y()+a0.y())/2,(d1.z()+a0.z())/2);
        Point p6((b1.x()+a1.x())/2,(b1.y()+a1.y())/2,(b1.z()+a1.z())/2);

        Line tempLine;
        tempLine.p[0] = p5.x(); tempLine.p[1] = p5.y(); tempLine.p[2] = p5.z();
        tempLine.q[0] = p6.x(); tempLine.q[1] = p6.y(); tempLine.q[2] = p6.z();
        _smoothTensorLinesCorrected.push_back(tempLine);
        tempLine.p[0] = p6.x(); tempLine.p[1] = p6.y(); tempLine.p[2] = p6.z();
        tempLine.q[0] = p2.x(); tempLine.q[1] = p2.y(); tempLine.q[2] = p2.z();
        _smoothTensorLinesCorrected.push_back(tempLine);
        tempLine.p[0] = p2.x(); tempLine.p[1] = p2.y(); tempLine.p[2] = p2.z();
        tempLine.q[0] = p1.x(); tempLine.q[1] = p1.y(); tempLine.q[2] = p1.z();
        _smoothTensorLinesCorrected.push_back(tempLine);
        tempLine.p[0] = p1.x(); tempLine.p[1] = p1.y(); tempLine.p[2] = p1.z();
        tempLine.q[0] = p5.x(); tempLine.q[1] = p5.y(); tempLine.q[2] = p5.z();
        _smoothTensorLinesCorrected.push_back(tempLine);
        tempLine.p[0] = p3.x(); tempLine.p[1] = p3.y(); tempLine.p[2] = p3.z();
        tempLine.q[0] = p1.x(); tempLine.q[1] = p1.y(); tempLine.q[2] = p1.z();
        _smoothTensorLinesCorrected.push_back(tempLine);
        tempLine.p[0] = p3.x(); tempLine.p[1] = p3.y(); tempLine.p[2] = p3.z();
        tempLine.q[0] = p4.x(); tempLine.q[1] = p4.y(); tempLine.q[2] = p4.z();
        _smoothTensorLinesCorrected.push_back(tempLine);
        tempLine.p[0] = p4.x(); tempLine.p[1] = p4.y(); tempLine.p[2] = p4.z();
        tempLine.q[0] = p2.x(); tempLine.q[1] = p2.y(); tempLine.q[2] = p2.z();
        _smoothTensorLinesCorrected.push_back(tempLine);

        // Add two roof planes
        // After triangulation, project points 4 and
        // 6 to the planes of tri-123 and tri-125,
        // respectively.
        Triangle3 t1(p1,p2,p3);
        Triangle3 t2(p1,p2,p5);

        Point p4_projected = t1.supporting_plane().projection(p4);
        Point p6_projected = t2.supporting_plane().projection(p6);

        Rect roof1,roof2;
        roof1.p[0] = p1.x();roof1.p[1] = p1.y();roof1.p[2] = p1.z();
        roof1.q[0] = p2.x();roof1.q[1] = p2.y();roof1.q[2] = p2.z();
        roof1.r[0] = p4_projected.x();roof1.r[1] = p4_projected.y();roof1.r[2] = p4_projected.z();
        roof1.s[0] = p3.x();roof1.s[1] = p3.y();roof1.s[2] = p3.z();
        roof2.p[0] = p1.x();roof2.p[1] = p1.y();roof2.p[2] = p1.z();
        roof2.q[0] = p2.x();roof2.q[1] = p2.y();roof2.q[2] = p2.z();
        roof2.r[0] = p6_projected.x();roof2.r[1] = p6_projected.y();roof2.r[2] = p6_projected.z();
        roof2.s[0] = p5.x();roof2.s[1] = p5.y();roof2.s[2] = p5.z();
        _roofPlanes.push_back(roof1);
        _roofPlanes.push_back(roof2);

        tempLine.p[0] = p5.x(); tempLine.p[1] = p5.y(); tempLine.p[2] = p5.z();
        tempLine.q[0] = p6_projected.x(); tempLine.q[1] = p6_projected.y(); tempLine.q[2] = p6_projected.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);
        tempLine.p[0] = p6_projected.x(); tempLine.p[1] = p6_projected.y(); tempLine.p[2] = p6_projected.z();
        tempLine.q[0] = p2.x(); tempLine.q[1] = p2.y(); tempLine.q[2] = p2.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);
        tempLine.p[0] = p2.x(); tempLine.p[1] = p2.y(); tempLine.p[2] = p2.z();
        tempLine.q[0] = p1.x(); tempLine.q[1] = p1.y(); tempLine.q[2] = p1.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);
        tempLine.p[0] = p1.x(); tempLine.p[1] = p1.y(); tempLine.p[2] = p1.z();
        tempLine.q[0] = p5.x(); tempLine.q[1] = p5.y(); tempLine.q[2] = p5.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);
        tempLine.p[0] = p3.x(); tempLine.p[1] = p3.y(); tempLine.p[2] = p3.z();
        tempLine.q[0] = p1.x(); tempLine.q[1] = p1.y(); tempLine.q[2] = p1.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);
        tempLine.p[0] = p3.x(); tempLine.p[1] = p3.y(); tempLine.p[2] = p3.z();
        tempLine.q[0] = p4_projected.x(); tempLine.q[1] = p4_projected.y(); tempLine.q[2] = p4_projected.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);
        tempLine.p[0] = p4_projected.x(); tempLine.p[1] = p4_projected.y(); tempLine.p[2] = p4_projected.z();
        tempLine.q[0] = p2.x(); tempLine.q[1] = p2.y(); tempLine.q[2] = p2.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);


        for(int i=0;i<tls.size();i++) {
            if(tls[i].size()<2){continue;}
            else {
                Line3 line;
                if(i==1) {
                    line = Line3(p5,p6_projected);
                }else if(i==2){
                    line = Line3(p6_projected,p2);
                }else if(i==3){
                    line = Line3(p2,p1);
                }else if(i==4){
                    line = Line3(p1,p5);
                }else if(i==5){
                    line = Line3(p3,p1);
                }else if(i==6){
                    line = Line3(p3,p4_projected);
                }else if(i==7){
                    line = Line3(p4_projected,p2);
                }
                float error = 0;
                for(int k=0;k<tls[i].size();k++) {
                    error += squared_distance(tls[i][k],line);
                }
//                cout << error << endl;
            }
        }

    } else if(roof_topology_type==1) {
        // Vertical Gable
        Point p1((a0.x()+e0.x()+d1.x())/3,(a0.y()+e0.y()+d1.y())/3,(a0.z()+e0.z()+d1.z())/3);
        Point p2((c1.x()+g1.x()+d0.x())/3,(c1.y()+g1.y()+d0.y())/3,(c1.z()+g1.z()+d0.z())/3);
        Point p3((f0.x()+e1.x())/2,(f0.y()+e1.y())/2,(f0.z()+e1.z())/2);
        Point p4((f1.x()+g0.x())/2,(f1.y()+g0.y())/2,(f1.z()+g0.z())/2);
        Point p5((b0.x()+a1.x())/2,(b0.y()+a1.y())/2,(b0.z()+a1.z())/2);
        Point p6((b1.x()+c0.x())/2,(b1.y()+c0.y())/2,(b1.z()+c0.z())/2);

        Line tempLine;
        tempLine.p[0] = p5.x(); tempLine.p[1] = p5.y(); tempLine.p[2] = p5.z();
        tempLine.q[0] = p6.x(); tempLine.q[1] = p6.y(); tempLine.q[2] = p6.z();
        _smoothTensorLinesCorrected.push_back(tempLine);
        tempLine.p[0] = p6.x(); tempLine.p[1] = p6.y(); tempLine.p[2] = p6.z();
        tempLine.q[0] = p2.x(); tempLine.q[1] = p2.y(); tempLine.q[2] = p2.z();
        _smoothTensorLinesCorrected.push_back(tempLine);
        tempLine.p[0] = p2.x(); tempLine.p[1] = p2.y(); tempLine.p[2] = p2.z();
        tempLine.q[0] = p1.x(); tempLine.q[1] = p1.y(); tempLine.q[2] = p1.z();
        _smoothTensorLinesCorrected.push_back(tempLine);
        tempLine.p[0] = p1.x(); tempLine.p[1] = p1.y(); tempLine.p[2] = p1.z();
        tempLine.q[0] = p5.x(); tempLine.q[1] = p5.y(); tempLine.q[2] = p5.z();
        _smoothTensorLinesCorrected.push_back(tempLine);
        tempLine.p[0] = p3.x(); tempLine.p[1] = p3.y(); tempLine.p[2] = p3.z();
        tempLine.q[0] = p1.x(); tempLine.q[1] = p1.y(); tempLine.q[2] = p1.z();
        _smoothTensorLinesCorrected.push_back(tempLine);
        tempLine.p[0] = p3.x(); tempLine.p[1] = p3.y(); tempLine.p[2] = p3.z();
        tempLine.q[0] = p4.x(); tempLine.q[1] = p4.y(); tempLine.q[2] = p4.z();
        _smoothTensorLinesCorrected.push_back(tempLine);
        tempLine.p[0] = p4.x(); tempLine.p[1] = p4.y(); tempLine.p[2] = p4.z();
        tempLine.q[0] = p2.x(); tempLine.q[1] = p2.y(); tempLine.q[2] = p2.z();
        _smoothTensorLinesCorrected.push_back(tempLine);


        // Add two roof planes
        // After triangulation, project points 4 and
        // 6 to the planes of tri-123 and tri-125,
        // respectively.
        Triangle3 t1(p1,p2,p3);
        Triangle3 t2(p1,p2,p5);

        Point p4_projected = t1.supporting_plane().projection(p4);
        Point p6_projected = t2.supporting_plane().projection(p6);

        Rect roof1,roof2;
        roof1.p[0] = p1.x();roof1.p[1] = p1.y();roof1.p[2] = p1.z();
        roof1.q[0] = p2.x();roof1.q[1] = p2.y();roof1.q[2] = p2.z();
        roof1.r[0] = p4_projected.x();roof1.r[1] = p4_projected.y();roof1.r[2] = p4_projected.z();
        roof1.s[0] = p3.x();roof1.s[1] = p3.y();roof1.s[2] = p3.z();
        roof2.p[0] = p1.x();roof2.p[1] = p1.y();roof2.p[2] = p1.z();
        roof2.q[0] = p2.x();roof2.q[1] = p2.y();roof2.q[2] = p2.z();
        roof2.r[0] = p6_projected.x();roof2.r[1] = p6_projected.y();roof2.r[2] = p6_projected.z();
        roof2.s[0] = p5.x();roof2.s[1] = p5.y();roof2.s[2] = p5.z();
        _roofPlanes.push_back(roof1);
        _roofPlanes.push_back(roof2);

        tempLine.p[0] = p5.x(); tempLine.p[1] = p5.y(); tempLine.p[2] = p5.z();
        tempLine.q[0] = p6_projected.x(); tempLine.q[1] = p6_projected.y(); tempLine.q[2] = p6_projected.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);
        tempLine.p[0] = p6_projected.x(); tempLine.p[1] = p6_projected.y(); tempLine.p[2] = p6_projected.z();
        tempLine.q[0] = p2.x(); tempLine.q[1] = p2.y(); tempLine.q[2] = p2.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);
        tempLine.p[0] = p2.x(); tempLine.p[1] = p2.y(); tempLine.p[2] = p2.z();
        tempLine.q[0] = p1.x(); tempLine.q[1] = p1.y(); tempLine.q[2] = p1.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);
        tempLine.p[0] = p1.x(); tempLine.p[1] = p1.y(); tempLine.p[2] = p1.z();
        tempLine.q[0] = p5.x(); tempLine.q[1] = p5.y(); tempLine.q[2] = p5.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);
        tempLine.p[0] = p3.x(); tempLine.p[1] = p3.y(); tempLine.p[2] = p3.z();
        tempLine.q[0] = p1.x(); tempLine.q[1] = p1.y(); tempLine.q[2] = p1.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);
        tempLine.p[0] = p3.x(); tempLine.p[1] = p3.y(); tempLine.p[2] = p3.z();
        tempLine.q[0] = p4_projected.x(); tempLine.q[1] = p4_projected.y(); tempLine.q[2] = p4_projected.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);
        tempLine.p[0] = p4_projected.x(); tempLine.p[1] = p4_projected.y(); tempLine.p[2] = p4_projected.z();
        tempLine.q[0] = p2.x(); tempLine.q[1] = p2.y(); tempLine.q[2] = p2.z();
        _smoothTensorLinesPlanarityCorrected.push_back(tempLine);

        for(int i=0;i<tls.size();i++) {
            if(tls[i].size()<2){continue;}
            else {
                Line3 line;
                if(i==1){
                    line = Line3(p1,p5);
                }else if(i==2) {
                    line = Line3(p5,p6);
                }else if(i==3){
                    line = Line3(p6,p2);
                }else if(i==4){
                    line = Line3(p2,p1);
                }else if(i==5){
                    line = Line3(p3,p1);
                }else if(i==6){
                    line = Line3(p3,p4);
                }else if(i==7){
                    line = Line3(p4,p2);
                }
                float error = 0;
                for(int k=0;k<tls[i].size();k++) {
                    error += squared_distance(tls[i][k],line);
                }
//                cout << error << endl;
            }
        }
    }
}


void DisplayPoints::roofRMSEerror() {
    // Roof error calculation
    float areaMinX  = 100,areaMinY=100,areaMaxX=-100,areaMaxY=-100;
    for(size_t i = 0; i <_inCloud->points.size(); i++){
        areaMinX = min(areaMinX ,_inCloud->points[i].x);
        areaMinY = min(areaMinY ,_inCloud->points[i].y);
        areaMaxX = max(areaMaxX ,_inCloud->points[i].x);
        areaMaxY = max(areaMaxY ,_inCloud->points[i].y);
    }
    float denom = max((areaMaxX-areaMinX),(areaMaxY-areaMinY));

    for(int i=0;i<_roofPlanes.size();i+=2) {
        float minX=100,minY=100,maxX=-100,maxY=-100;
        Rect roof1 = _roofPlanes[i];
        Rect roof2 = _roofPlanes[i+1];
        Plane roofPlane1(Point(roof1.p[0],roof1.p[1],roof1.p[2]), Point(roof1.q[0],roof1.q[1],roof1.q[2]), Point(roof1.r[0],roof1.r[1],roof1.r[2]));
        Plane roofPlane2(Point(roof2.p[0],roof2.p[1],roof2.p[2]), Point(roof2.q[0],roof2.q[1],roof2.q[2]), Point(roof2.r[0],roof2.r[1],roof2.r[2]));
        minX = min(roof1.p[0],min(roof1.q[0],min(roof1.r[0],min(roof1.s[0],min(roof2.p[0],min(roof2.q[0],min(roof2.r[0],roof2.s[0])))))));
        minY = min(roof1.p[1],min(roof1.q[1],min(roof1.r[1],min(roof1.s[1],min(roof2.p[1],min(roof2.q[1],min(roof2.r[1],roof2.s[1])))))));
        maxX = max(roof1.p[0],max(roof1.q[0],max(roof1.r[0],max(roof1.s[0],max(roof2.p[0],max(roof2.q[0],max(roof2.r[0],roof2.s[0])))))));
        maxY = max(roof1.p[1],max(roof1.q[1],max(roof1.r[1],max(roof1.s[1],max(roof2.p[1],max(roof2.q[1],max(roof2.r[1],roof2.s[1])))))));
        float error = 0;
        int total = 0;
        for (size_t i = 0; i <_inCloud->points.size(); i++) {
            if(_inCloud->points[i].x >= minX && _inCloud->points[i].x <= maxX && _probval[i].label==5) {
                error += min(squared_distance(Point(_inCloud->points[i].x,_inCloud->points[i].y,_inCloud->points[i].z),roofPlane1), squared_distance(Point(_inCloud->points[i].x,_inCloud->points[i].y,_inCloud->points[i].z),roofPlane2));
                total++;
            }
        }
        float rmse = sqrt(error/total);
        cout << "RMSE = " << rmse << endl;
        cout << "% RMSE = " << rmse/denom << endl;
    }
}

void DisplayPoints::displayBoundary() {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3f(0,1,0) ;
    glBegin(GL_QUADS) ;
    glVertex3f(_inCloud->points[0].x,_inCloud->points[0].y,_inCloud->points[0].z);
    glVertex3f(_inCloud->points[3637].x,_inCloud->points[3637].y,_inCloud->points[3637].z);
    glVertex3f(_inCloud->points[3577].x,_inCloud->points[3577].y,_inCloud->points[3577].z);
    glVertex3f(_inCloud->points[1315].x,_inCloud->points[1315].y,_inCloud->points[1315].z);
    glEnd() ;

    glLineWidth(6.0f);
    glBegin(GL_LINES) ;
    glVertex3f(_inCloud->points[0].x,_inCloud->points[0].y,_inCloud->points[0].z);
    glVertex3f(_inCloud->points[9].x,_inCloud->points[9].y,_inCloud->points[9].z);
    glVertex3f(_inCloud->points[1315].x,_inCloud->points[1315].y,_inCloud->points[1315].z);
    glVertex3f(_inCloud->points[9].x,_inCloud->points[9].y,_inCloud->points[9].z);

    glVertex3f(_inCloud->points[3577].x,_inCloud->points[3577].y,_inCloud->points[3577].z);
    glVertex3f(_inCloud->points[1218].x,_inCloud->points[1218].y,_inCloud->points[1218].z);
    glVertex3f(_inCloud->points[3637].x,_inCloud->points[3637].y,_inCloud->points[3637].z);
    glVertex3f(_inCloud->points[1218].x,_inCloud->points[1218].y,_inCloud->points[1218].z);

    glVertex3f(_inCloud->points[9].x,_inCloud->points[9].y,_inCloud->points[9].z);
    glVertex3f(_inCloud->points[2642].x,_inCloud->points[2642].y,_inCloud->points[2642].z);
    glVertex3f(_inCloud->points[2642].x,_inCloud->points[2642].y,_inCloud->points[2642].z);
    glVertex3f(_inCloud->points[1218].x,_inCloud->points[1218].y,_inCloud->points[1218].z);
    glEnd() ;


}

/*****************************************************************************************

Function Name		:	PointClassificaton::probMeasure
Purpose	of Function	:	
Calls			:	PointClassificaton::CurvatureEstimation
Input Params		:	
Output Params		:	T *curve, T *disc, T *spherical
Return			:	int
Remarks			:

*****************************************************************************************/

bool DisplayPoints::plyDisplay()
{

	if(_inCloud->points.size() <= 0)
		return false;

	glBegin(GL_POINTS);

  	glColor3f(0.5f, 0.5f, 0.5f);

	for (size_t i = 0; i <_inCloud->points.size(); i++)
	{ 
        glColor3f(0.5f, 0.5f, 0.5f);
      	glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
	}
  	glEnd();

	return true;
}

/*****************************************************************************************
*****************************************************************************************/
bool DisplayPoints::renderStructFDs(int pointMode)
{
	if(_inCloud->points.size() <= 0 || _probval.size()<=0)
		return false;

        if(pointMode == 1)
	{
		idxPtCloudFeat();   ////reduced point cloud with feature value
	}
        else if(pointMode == 2)
	{
		renderCurvature();
	}
        else if(pointMode == 3)
	{
		csclcpDisplay();
    }
    else if(pointMode == 4)
    {
        drawLineFatures();
    }
    else if(pointMode == 5)
    {
        sumeigen_Display();
    }
    else if(pointMode == 6)
    {
        planarityDisplay();
    }
    else if(pointMode == 7)
    {
        anisotropyDisplay();
    }
    else if(pointMode == 8)
    {
        sphericityDisplay();
    }
    else if(pointMode == 9)
    {
        lineWireframe();
    }
    else if(pointMode == 10)
    {
        triangulation_pointset();
    }
    else if(pointMode == 11)
    {
        donDisplay();
    }
    else if(pointMode == 12)
    {
        contours();
    }
    else if(pointMode == 13)
    {
        tensorLines();
    }
    else if(pointMode == 14)
    {
        heightMap();
    }
    else if(pointMode == 15)
    {
        linearityDisplay();
    }
    else if(pointMode == 16)
    {
        omnivarianceDisplay();
    }
    else if(pointMode == 17)
    {
        eigenentropyDisplay();
    }
    else if(pointMode == 18)
    {
        labelsDisplay();
    }

	return true;
}
/*****************************************************************************************
*****************************************************************************************/
void DisplayPoints::idxPtCloudFeat()
{

   	glBegin(GL_POINTS);
	
    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        glColor3f(_probval[i].prob[1], _probval[i].prob[2], _probval[i].prob[0]);
      	glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

  	glEnd();
	return;
}
/*****************************************************************************************
*****************************************************************************************/
void DisplayPoints::renderCurvature()
{

	Color currentcolor ;
   	glBegin(GL_POINTS);
	
  	for(size_t i = 0; i <_inCloud->points.size(); i++)
	{ 
            currentcolor = currentcolor.findColor(_probval[i].featStrength[2], 0.0, 1.0) ;
            glColor3d(currentcolor.red(), currentcolor.green(), currentcolor.blue()) ;
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
	}

  	glEnd();
	return;
}
/*****************************************************************************************
*****************************************************************************************/
void DisplayPoints::csclcpDisplay()
{
    glPointSize(4.0);
   	glBegin(GL_POINTS);

  	for (size_t i = 0; i <_inCloud->points.size(); i++)
	{ 	
        float cl,cs,cp;
        cs = _probval[i].csclcp[2];
        cl = _probval[i].csclcp[1];
        cp = _probval[i].csclcp[0];
         glColor3f(cl, cs, cp);
//          if(_probval[i].label==5)
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
	}

  	glEnd();
	return;

}


void DisplayPoints::sumeigen_Display()
{
    glBegin(GL_POINTS);

    float max = -1.0;
    float min = 10000.0;
    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(_probval[i].sum_eigen > max && _probval[i].sum_eigen!=-1)
            max = _probval[i].sum_eigen;
        if(_probval[i].sum_eigen < min && _probval[i].sum_eigen!=-1)
            min = _probval[i].sum_eigen;
    }

    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        float val = (_probval[i].sum_eigen - min) / (max - min);
        if(_probval[i].sum_eigen==-1)
            glColor3f(0.0, 1.0, 0.0) ;
        else
            glColor3f(val, 0.0, val) ;
        if(val>=scalarMin && val<=scalarMax)
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

    glEnd();

    return;
}


void DisplayPoints::planarityDisplay()
{
    glBegin(GL_POINTS);

    float max = -1.0;
    float min = 10000.0;
    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(_probval[i].planarity > max)
            max = _probval[i].planarity;
        if(_probval[i].planarity < min)
            min = _probval[i].planarity;
    }

    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        float val = _probval[i].csclcp[1];
            if(val==-1)
                glColor3d(0.0, 1.0, 0.0) ;
            else
                glColor3d(val, 0.0, val) ;
        if(val>=scalarMin && val<=scalarMax)
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

    glEnd();

    return;
}

void DisplayPoints::donDisplay()
{
    glPointSize(4.0);
    glBegin(GL_POINTS);

    float max = -1.0;
    float min = 10000.0;
    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(_probval[i].don > max)
            max = _probval[i].don;
        if(_probval[i].don < min)
            min = _probval[i].don;
    }

    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        float val = (_probval[i].don - min)/(max - min);
        if(val==-1)
            glColor3f(0.0, 1.0, 0.0) ;
        else
            glColor3f(val, 0.0, val) ;

        if(val>=scalarMin && val<=scalarMax)
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

    glEnd();

    return;
}


void DisplayPoints::anisotropyDisplay()
{
    glPointSize(6.0);
    glBegin(GL_POINTS);

    int count = 0;
    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        float val = _probval[i].anisotropy;

        if(val == 0)
            count++;
        if(val==-1)
            glColor3f(1.0, 0.0, 0.0) ;
        else
            glColor3f(val, 0.0, val) ;
        if(val>=scalarMin && val<=scalarMax)
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

    glEnd();
    return;
}

void DisplayPoints::sphericityDisplay()
{
    glBegin(GL_POINTS);

    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        float val = _probval[i].sphericity;
        if(val==-1)
            glColor3f(0.0, 1.0, 0.0) ;
        else
            glColor3f(0.0, val, val) ;

        if(val>=scalarMin && val<=scalarMax)
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

    glEnd();
    return;
}
/*****************************************************************************************
Overwritten by Kreylo's graph in Processing.cpp
*****************************************************************************************/
void DisplayPoints::drawLineFatures()
{
    glBegin(GL_POINTS);

    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(_ftPtsProp[i] == 1)
        {
            glColor3f(0, 0, 1);
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
        }
        else if(_ftPtsProp[i] == 2)
        {
            glColor3f(1, 0, 0);
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
        }
        else  if(_ftPtsProp[i] == 3)
        {
            glColor3f(0, 1, 0);
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
        }
    }

    glEnd();
    return;

}

/*****************************************************************************************
*****************************************************************************************/

void DisplayPoints::drawSpectrum(void)
{
  	Color currentcolor ;
  //	double vmin, vmax, v;

  	glBegin(GL_QUAD_STRIP) ;

  	for (int i = -3 ; i <= 4 ; i++) 
	{
  		double k = i + 3;
  		currentcolor = currentcolor.findColor(k/7.0, 0.0, 1.0) ;
  		glColor3d(currentcolor.red(), currentcolor.green(), currentcolor.blue()) ;
  		glVertex3d(1.70, (double)i/4.0, 0.0) ;
  		glVertex3d(1.80, (double)i/4.0, 0.0) ;
  	}

	glEnd() ;

}

/*****************************************************************************************
 * Triangulation Display
*****************************************************************************************/

void DisplayPoints::lineWireframe()
{
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(1.5);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_TRIANGLES) ;
    glColor3f(1,0,0) ;

    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(_probval[i].label==5){
        for (size_t j = 0; j <_probval[i].triangles.size(); j++)
        {
            if(_probval[_probval[i].triangles[j].pid].csclcp[1] > _probval[_probval[i].triangles[j].pid].csclcp[0] && _probval[_probval[i].triangles[j].pid].csclcp[1] > _probval[_probval[i].triangles[j].pid].csclcp[2])
                glColor3f(_probval[_probval[i].triangles[j].pid].csclcp[1],0,0);
            else
                glColor3f(_probval[_probval[i].triangles[j].pid].csclcp[1], _probval[_probval[i].triangles[j].pid].csclcp[2], _probval[_probval[i].triangles[j].pid].csclcp[0]);
            glVertex3f(_probval[i].triangles[j].p[0],_probval[i].triangles[j].p[1],_probval[i].triangles[j].p[2]);

            if(_probval[_probval[i].triangles[j].qid].csclcp[1] > _probval[_probval[i].triangles[j].qid].csclcp[0] && _probval[_probval[i].triangles[j].qid].csclcp[1] > _probval[_probval[i].triangles[j].qid].csclcp[2])
                glColor3f(_probval[_probval[i].triangles[j].qid].csclcp[1],0,0);
            else
                glColor3f(_probval[_probval[i].triangles[j].qid].csclcp[1], _probval[_probval[i].triangles[j].qid].csclcp[2], _probval[_probval[i].triangles[j].qid].csclcp[0]);

            glVertex3f(_probval[i].triangles[j].q[0],_probval[i].triangles[j].q[1],_probval[i].triangles[j].q[2]);

            if(_probval[_probval[i].triangles[j].rid].csclcp[1] > _probval[_probval[i].triangles[j].rid].csclcp[0] && _probval[_probval[i].triangles[j].rid].csclcp[1] > _probval[_probval[i].triangles[j].rid].csclcp[2])
                glColor3f(_probval[_probval[i].triangles[j].rid].csclcp[1],0,0);
            else
                glColor3f(_probval[_probval[i].triangles[j].rid].csclcp[1], _probval[_probval[i].triangles[j].rid].csclcp[2], _probval[_probval[i].triangles[j].rid].csclcp[0]);
            glVertex3f(_probval[i].triangles[j].r[0],_probval[i].triangles[j].r[1],_probval[i].triangles[j].r[2]);
        }
        }
    }

    glEnd() ;
}

void DisplayPoints::contours()
{
    glPointSize(1.0);
    glBegin(GL_POINTS);
    for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
            double temp = _intensity[i];
            glColor3d(temp, temp, temp) ;
            if(_probval[i].label == 5)
                glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
        }
    glEnd();

    glEnable(GL_LINE_SMOOTH);
    glLineWidth(1.5);
    glBegin(GL_LINES) ;

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_LINES) ;
    glColor3f(1,0,0) ;

    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        for (size_t j = 0; j <_probval[i].triangles.size(); j++)
        {
            if(_probval[i].triangles[j].hasCL &&  (_probval[_probval[i].triangles[j].pid].label == 5 || _probval[_probval[i].triangles[j].qid].label == 5  || _probval[_probval[i].triangles[j].rid].label == 5) )
            {
                glVertex3f(_probval[i].triangles[j].CL_p1[0],_probval[i].triangles[j].CL_p1[1],_probval[i].triangles[j].CL_p1[2]);
                glVertex3f(_probval[i].triangles[j].CL_p2[0],_probval[i].triangles[j].CL_p2[1],_probval[i].triangles[j].CL_p2[2]);
            }
        }
    }

    glEnd() ;
    displayBoundary();
}

void DisplayPoints::writeTostl()
{
    std::ofstream myfile;
    myfile.open ("triangulation.txt");
//    myfile << "solid model" << std::endl;

//    for (size_t i = 0; i <_inCloud->points.size(); i++)
//    {
//        for (size_t j = 0; j <_probval[i].triangles.size(); j++)
//        {
//            myfile << "\t\touter loop" << std::endl;
//            myfile << "\t\t\tvertex " << _probval[i].triangles[j].p[0] << " " << _probval[i].triangles[j].p[1] << " " << _probval[i].triangles[j].p[2] << std::endl;
//            myfile << "\t\t\tvertex " << _probval[i].triangles[j].q[0] << " " << _probval[i].triangles[j].q[1] << " " << _probval[i].triangles[j].q[2] << std::endl;
//            myfile << "\t\t\tvertex " << _probval[i].triangles[j].r[0] << " " << _probval[i].triangles[j].r[1] << " " << _probval[i].triangles[j].r[2] << std::endl;
//            myfile << "\t\tendloop" << std::endl;
//            myfile << "\tendfacet" << std::endl;
//        }
//    }
    int tNdx = 0;
    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        for (size_t j = 0; j <_probval[i].triangles.size(); j++)
        {
            myfile << tNdx <<" " << _probval[i].triangles[j].pid << " " << _probval[i].triangles[j].qid << " " << _probval[i].triangles[j].rid << std::endl;
            tNdx++;
        }
    }
    myfile.close();
}

void DisplayPoints::eigenentropyDisplay()
{
    glPointSize(6.0);
    glBegin(GL_POINTS);

    float max = -1.0;
    float min = 10000.0;
    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(_probval[i].eigenentropy > max)
            max = _probval[i].eigenentropy;
        if(_probval[i].eigenentropy < min)
            min = _probval[i].eigenentropy;
    }

    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        float val = (_probval[i].eigenentropy - min) / (max - min);
        glColor3f(val, 0.0, val) ;

        if(val>=scalarMin && val<=scalarMax)
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

    glEnd();

    return;
}

void DisplayPoints::omnivarianceDisplay()
{
    glPointSize(6.0);
    glBegin(GL_POINTS);

    float max = -1.0;
    float min = 10000.0;
    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(_probval[i].omnivariance > max)
            max = _probval[i].omnivariance;
        if(_probval[i].omnivariance < min)
            min = _probval[i].omnivariance;
    }

    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        float val = (_probval[i].omnivariance - min) / (max - min);
        glColor3f(val, 0.0, val) ;

        if(val>=scalarMin && val<=scalarMax)
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

    glEnd();

    return;
}

void DisplayPoints::linearityDisplay()
{
    // Line saliency Heat map
    glPointSize(6.0);
    glBegin(GL_POINTS);

    float max = -1.0;
    float min = 10000.0;
    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(_probval[i].csclcp[1] > max)
            max = _probval[i].csclcp[1];
        if(_probval[i].csclcp[1] < min)
            min = _probval[i].csclcp[1];
    }

    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        float val = (_probval[i].csclcp[1] - min) / (max - min);
        glColor3f(val, 0.0, val) ;

        if(val>=scalarMin && val<=scalarMax)
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

    glEnd();

    return;
}

void DisplayPoints::heightMap()
{
    // Height Map
    float maxH = -1.0;
    float minH = 100.0;
    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(_inCloud->points[i].z > maxH)
            maxH = _inCloud->points[i].z;
        if(_inCloud->points[i].z < minH)
            minH = _inCloud->points[i].z;
    }

    glPointSize(6.0);
    glBegin(GL_POINTS);
    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        float val = (_inCloud->points[i].z - minH)/(maxH-minH);
        glColor3f(val,val,val) ;

        if(val>=scalarMin && val<=scalarMax)
            glVertex3f(_inCloud->points[i].x,_inCloud->points[i].y,_inCloud->points[i].z);
    }
    glEnd() ;
}

void DisplayPoints::tensorLines()
{
    // csclcpDisplay();

    if(TENSORLINE_IMPLEMENTATION_TYPE == 0) {
        // Streamline
        glPointSize(1.0);
        glBegin(GL_POINTS);
        for(size_t i = 0; i <_inCloud->points.size(); i++)
            {
                double temp = _intensity[i];
                glColor3d(temp, temp, temp) ;
                if(_probval[i].label == 5)
                    glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
            }
        glEnd();

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glColor3f(1,0,0) ;

        glEnable(GL_LINE_SMOOTH);
        glLineWidth(1.5);
        glBegin(GL_LINES) ;

        for (size_t i = 0; i <_inCloud->points.size(); i++)
        {
            for (size_t j = 0; j <_probval[i].tensor_line.size(); j++)
            {
                if(_probval[_probval[i].tensor_line[j].ndxq].label == 5)
                {
                    glVertex3f(_probval[i].tensor_line[j].p[0],_probval[i].tensor_line[j].p[1],_probval[i].tensor_line[j].p[2]);
                    glVertex3f(_probval[i].tensor_line[j].q[0],_probval[i].tensor_line[j].q[1],_probval[i].tensor_line[j].q[2]);
                }
            }
        }

        glEnd() ;
    }
    else if(TENSORLINE_IMPLEMENTATION_TYPE==1) {
        glPointSize(1.0);
        glBegin(GL_POINTS);
        for(size_t i = 0; i <_inCloud->points.size(); i++)
            {
            double temp = _intensity[i];
            glColor3d(temp, temp, temp) ;
            if(_probval[i].label == 5)
                    glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
            }
        glEnd();

//        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

//        for (size_t i = 0; i <_roofPlanes.size(); i++)
//        {
//            Rect rect = _roofPlanes[i];
//            glBegin(GL_QUADS);
//                    glColor3d(1,1,0);
//                    glVertex3f(rect.p[0],rect.p[1],rect.p[2]);
//                    glVertex3f(rect.q[0],rect.q[1],rect.q[2]);
//                    glVertex3f(rect.r[0],rect.r[1],rect.r[2]);
//                    glVertex3f(rect.s[0],rect.s[1],rect.s[2]);
//            glEnd();
//        }

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        glEnable(GL_LINE_SMOOTH);
        glLineWidth(4.0);
        glBegin(GL_LINES) ;

        glColor3f(1,0,0) ;
        for (size_t i = 0; i <_smoothTensorLines.size(); i++)
        {
            Line line = _smoothTensorLines[i];
            glVertex3f(line.p[0],line.p[1],line.p[2]);
            glVertex3f(line.q[0],line.q[1],line.q[2]);
        }

        glColor3f(0,0,1) ;
        for (size_t i = 0; i <_smoothTensorLinesCorrected.size(); i++)
        {
            Line line = _smoothTensorLinesCorrected[i];
            glVertex3f(line.p[0],line.p[1],line.p[2]);
            glVertex3f(line.q[0],line.q[1],line.q[2]);
        }
        glColor3f(1,0,1) ;
        for (size_t i = 0; i <_smoothTensorLinesPlanarityCorrected.size(); i++)
        {
            Line line = _smoothTensorLinesPlanarityCorrected[i];
            glVertex3f(line.p[0],line.p[1],line.p[2]);
            glVertex3f(line.q[0],line.q[1],line.q[2]);
        }
        glEnd() ;

    }
}

void DisplayPoints::triangulation_pointset()
{
    glPointSize(6.0);
    glBegin(GL_POINTS);
    for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
            double temp = _intensity[i];
            glColor3d(temp, temp, temp) ;
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
        }
    glEnd();

    glPointSize(6.0);
    glBegin(GL_POINTS) ;

    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        for (size_t j = 0; j <_probval[i].triangles.size(); j++)
        {
            glColor3f(_probval[_probval[i].triangles[j].pid].csclcp[1], _probval[_probval[i].triangles[j].pid].csclcp[2], _probval[_probval[i].triangles[j].pid].csclcp[0]);
            glVertex3f(_probval[i].triangles[j].p[0],_probval[i].triangles[j].p[1],_probval[i].triangles[j].p[2]);

            glColor3f(_probval[_probval[i].triangles[j].qid].csclcp[1], _probval[_probval[i].triangles[j].qid].csclcp[2], _probval[_probval[i].triangles[j].qid].csclcp[0]);
            glVertex3f(_probval[i].triangles[j].q[0],_probval[i].triangles[j].q[1],_probval[i].triangles[j].q[2]);

            glColor3f(_probval[_probval[i].triangles[j].rid].csclcp[1], _probval[_probval[i].triangles[j].rid].csclcp[2], _probval[_probval[i].triangles[j].rid].csclcp[0]);
            glVertex3f(_probval[i].triangles[j].r[0],_probval[i].triangles[j].r[1],_probval[i].triangles[j].r[2]);
        }
    }
    glEnd() ;
}

void DisplayPoints::labelsDisplay()
{
    glBegin(GL_POINTS) ;
    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(_probval[i].label==0)
            glColor3f(1,1,0.5) ;
        else if(_probval[i].label==1)
            glColor3f(0,1,1) ;
        else if(_probval[i].label==2)
            glColor3f(1,1,1) ;
        else if(_probval[i].label==3)
            glColor3f(1,1,0) ;
        else if(_probval[i].label==4)
            glColor3f(0,1,0.5) ;
        else if(_probval[i].label==5)
            glColor3f(0,0,1) ;
        else if(_probval[i].label==6)
            glColor3f(0,0.5,1) ;
        else if(_probval[i].label==7)
            glColor3f(0.5,1,0) ;
        else if(_probval[i].label==8)
            glColor3f(0,1,0) ;
        glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }
    glEnd() ;
}
