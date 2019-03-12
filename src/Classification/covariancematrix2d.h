#ifndef COVARIANCEMATRIX2D_H
#define COVARIANCEMATRIX2D_H

#include <stdio.h>
#include <vector>
#include "OctreeSerial.h"

#include "data_type.h"


using namespace std;
using namespace Eigen;

class CovarianceMatrix2D
{
public:
    typedef Eigen::Matrix<double, 3, 1> ColumnVector;
    typedef Eigen::Matrix<double, 1 , 3> RowVector;
    typedef pcl::PointXYZ PointT;
    typedef pcl::octree::OctreeContainerPointIndices LeafContainerT;
    typedef pcl::octree::OctreeContainerEmpty BranchContainerT;


    CovarianceMatrix2D(){}

    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void setOctree(OctreeXYZ<PointT,LeafContainerT, BranchContainerT > * octree);

    void setParams(float rmin, float rmax, float rmaxpt, float epsi, float scale);

    void prob_measure(vector <probfeatnode> *probval);



private:

    bool setAccumzero();

    bool voteAccum(float radius);

    bool probMeasure(vector<probfeatnode>&probfeatVal);

    void writeGlyphVars(float radius);

    void computeSaliencyVals(glyphVars &glyphtemp, int idx);

    void glyphAnalysis(glyphVars &glyphtemp);

    bool eigendecomposition(glyphVars &glyphtemp, size_t idx);

    float _rmin, _rmax, _rmaxpt, _epsi, _scale;

    float _sigma;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _inCloud;

    OctreeXYZ<PointT,LeafContainerT, BranchContainerT > * _octree;

    vector < tensorType	> _accum;

    vector<glyphVars> _glyphData;

    vector < tensorType	> averaged_tensors;
};


#endif // COVARIANCEMATRIX2D_H
