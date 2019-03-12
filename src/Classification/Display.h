#ifndef DISPLAYPOINT_FILE_H
#define DISPLAYPOINT_FILE_H

#include <stdio.h>
#include <vector>
#include <string>
#include "data_type.h"
#include <pcl/io/pcd_io.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>

typedef double                      FT;
typedef CGAL::Simple_cartesian<FT>  K;
typedef K::Line_3                   Line3;
typedef K::Plane_3                  Plane;
typedef K::Point_3                  Point;
typedef K::Triangle_3               Triangle3;
/*****************************************************************************************
******************************************************************************************

Global Variables 

******************************************************************************************
*****************************************************************************************/

using namespace std;

class DisplayPoints
{
public:
	typedef	std::vector<ftType> PtContainer;
	typedef	std::vector<ftType> InetensityContainer;
	typedef std::vector<probfeatnode> FeatValContainer;
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudType;


        DisplayPoints(CloudType *cloud, vector<float>*intensity, vector <probfeatnode> *probval, vector <probfeatnode> *probval_tmp, vector <unsigned int> *ftPtsProp)
           :_inCloud(*cloud),_intensity(*intensity), _probval(*probval), _probval_tmp(*probval_tmp), _ftPtsProp(*ftPtsProp)
	{}
	~DisplayPoints();
;

	void drawSpectrum(void);
	
	bool lasDisplay();
	
	bool plyDisplay();

    void writeTostl();

    bool renderStructFDs(int pointMode);

    float scalarMin, scalarMax;

    void displayBoundary();

    void smoothTensorLines(string filename, int roof_topology_type);
    void roofRMSEerror();

    vector <Line> _smoothTensorLines;
    vector <Line> _smoothTensorLinesCorrected;
    vector <Line> _smoothTensorLinesPlanarityCorrected;
    vector <Rect> _roofPlanes;

private:
	CloudType &_inCloud;
	vector<float> &_intensity;
  	vector <probfeatnode> &_probval;
    vector <probfeatnode> &_probval_tmp;
    vector <unsigned int> &_ftPtsProp;

    vector <float> errorTL;
    vector <float> errorCTL;

    void renderCurvature();
	
	void idxPtCloudFeat();

	void csclcpDisplay();

    void sumeigen_Display();

    void planarityDisplay();

    void anisotropyDisplay();

    void sphericityDisplay();

    void donDisplay();

    void lineWireframe();

    void drawLineFatures();

    void contours();

    void eigenentropyDisplay();

    void omnivarianceDisplay();

    void linearityDisplay();

    void heightMap();

    void tensorLines();

    void triangulation_pointset();

    void labelsDisplay();


};


#endif
