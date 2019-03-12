#ifndef PROCESSING_FILE_H
#define PROCESSING_FILE_H

#include "tensor3dVoting.h"
#include "tensor2d.h"
#include "diffusednormalvoting.h"
#include "modcovariancematrix.h"
#include "DiffusedOptimal.h"
#include "CoVarianceOptimal.h"
#include "covariancematrix.h"
#include "covariancematrix2d.h"
#include "hessian.h"
#include "boundarytensor.h"
#include <stdio.h>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include "OctreeSerial.h"
#include "utils.h"
#include "data_type.h"
#include "Display.h"
#include "SearchNeighbour.h"
#include "SearchNeighbourFactory.h"
#include "SearchStructure.h"
#include "ProcessRequest.h"

using namespace std;

/*****************************************************************************************

Class Name		:	Processing	
Purpose			:	Implementation of Kreylo's paper
Created by		:	Beena
Created date		:	
Modification		:
Modified Function	:

*****************************************************************************************/

class Processing 
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudType;
    typedef pcl::PointCloud<pcl::PointXYZ> cl;
    typedef pcl::PointXYZ PointT;
    typedef pcl::octree::OctreeContainerPointIndices LeafContainerT;
    typedef pcl::octree::OctreeContainerEmpty BranchContainerT;

    Processing() ;
    ~Processing();
	/*New Method*/
	void buildStructure(ProcessRequest request);
	void processCloud(ProcessRequest request);
	void processPoint(ProcessRequest request, int index);
	bool fileRead(std::string cloudFile, std::string labelFile, std::string optimalFile);
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
	void Processing::processResult();
	/*End New Method*/

    void setParams(float rmin, float rmax, float scale, float eps, float sigmin, float sigmax, float scalarMin, float scalarMax) ;
    void setFilePath(std::string filePath) ;
    bool fileRead();
    void classifyStructFeats();
    void computeEigenValues(float rmin, float rmax, float rmaxpt, float epsi, float scale);
    bool structFeatClassification();
    void clear();
    void reset();
    int getDataSize();
    void setDisplayMode(int displaymode, int pointmode);
    void display();
    void computeFDs();
    void buildOctree();
    void scale();
    void setTensorType(int type);

    void curveGraphExtraction();

    void displayGraph();
  
  private:
    
    string _filePath, _FileExtension, _fileNo;
    ftType _rmin, _rmax,  _rmaxpt,  _epsi,  _scale, _scalarMin, _scalarMax;

    float _sigmax, _sigmin,  _rcurve, _rsurface;
    int _displaymode, _pointmode, _tensorType;

    CloudType _inCloud;
    vector <float> _intensity;
    vector <int> labels;
	vector <int> optimalScale;
    OctreeXYZ<PointT,LeafContainerT, BranchContainerT > * _octree;

    vector <probfeatnode> _probval;
    vector <probfeatnode> _probval_tmp;     // If using two tensors tensors
    vector <tensorType> _accum;     // aggregated tensor

    vector <unsigned int> _ftPtsProp;

    tensor3dvoting *_ptclassifyObj;
    tensor2d *_ptclassifyObj2D;
	DiffusedOptimal * _ptclassifyObj_Optimal_DNV;
	CoVarianceOptimal * _ptclassifyObj_Optimal_CoVariance;
    DiffusedNormalVoting * _ptclassifyObj_DNV;
    ModCovarianceMatrix *_ptclassifyObj_MCM;
    CovarianceMatrix *_ptclassifyObj_CM;
    CovarianceMatrix2D *_ptclassifyObj_CM2D;
    Hessian *_ptclassifyObj_Hessian;
    BoundaryTensor *_ptclassifyObj_Boundary;
    Utils * _utils;

    DisplayPoints *_displayObj;

    vector<gnode> _graph;
    vector<idxType> _graphIdx;
    vector<int>_label;
	SearchNeighbour* search;
};

#endif
