#ifndef LINEEXTRACTION_H
#define LINEEXTRACTION_H



#include <stdio.h>
#include <string.h>
#include <vector>
#include <iostream>
#include "data_type.h"
#include <pcl/io/pcd_io.h>


using namespace std;

/*****************************************************************************************

Class Name		:	FileRead
Purpose			:	read the input file and store data(x,y,z coordinates and intensity values) in the vectors data structure
Created by		:	Beena
Created date		:
Modification		:
Modified Function	:

*****************************************************************************************/
class graph
{
public:

    typedef std::vector<idxType> IndxCont;
    typedef std::vector<pcl::PointXYZ> PtCont;
    typedef vector< vector <idxType > > centroidType;

    graph(){}
    ~graph(){}

    void constructGraph(vector<gnode> &graph, vector<idxType> &graphIdx);

    void setParams(float radius, float rmaxpt);

    void setNode(std::vector<node> &seeds);

    void setCriticalNode(std::vector<node> &crticalNode);

    void setInputcloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void setSaliencyMaps(vector <probfeatnode> &probval);

private:
    bool r_search(std::vector<node> &seed, std::vector<node> &criticalSeed, IndxCont *neighborIndices, IndxCont *neighborSize, ftType radius, idxType rmaxpts, vector<myfloat3> &maxevecs);

    inline float get_val(myfloat3 u, myfloat3 v, myfloat3 evecs);

    int check(vector<gnode>*graph, myfloat3 evecs, int index);

    inline void get_minidx(myfloat3 &u, myfloat3 &v, myfloat3 &evecs, vector<tempminval> *tempmin, idxType j);


    bool get_index(std::vector<node> &seed, std::vector<node> &crit_seed, IndxCont &ng_idxs, myfloat3 evecs, idxType idx, myfloat3 v, idxType totCount, idxType loc, tempminval *min_idx);


    bool getminWidx(std::vector<node> &seed, std::vector<node> &crit_seed, IndxCont &ng_sz, IndxCont &ng_idxs, myfloat3 evecs, idxType idx, idxType *min_idx);

    bool propagate(std::vector<node> &seed, std::vector<node> &criticalSeed, idxType minIndex, idxType i, vector<gnode> *graph, vector<idxType> *graphIdx);

    int connectedge
    (
    std::vector<node> &seed,
    std::vector<node> &criticalSeed,
    IndxCont &neighborSize,
    IndxCont &neighborIndices,
    myfloat3 evecs,
    idxType idx,
    int checkerror,
    vector<gnode> *graph,
    vector<idxType> *graphIdx,
    idxType *min_idx,
    idxType *min_idx1
    );

    float getDistw(std::vector<node> &seed, std::vector<node> &criticalSeed, myfloat3 v, idxType minIndex);

    bool get_removedindices(std::vector<node> &seed, std::vector<node> &criticalSeed, IndxCont &neighborSize,
    IndxCont &neighborIndices, idxType index, idxType minIndex, myfloat3 evecs, IndxCont *removedIndices);

    void edgelinking(std::vector<node> &seed, std::vector<node> &criticalSeed, IndxCont &neighborSize, IndxCont &neighborIndices, vector<myfloat3> &maxevecs, vector<gnode> *graph, vector<idxType> *graphIdx);

    bool r_NearestCriticalNode(std::vector<node> *criticalSeed, IndxCont *neighborIndices, IndxCont *neighborSize, ftType radius, idxType rmaxpts);

    void r_clusterCriticalNode(std::vector<node> *criticalSeed, centroidType *centroid, float radius);

    idxType get_nearestCentroid(vector<idxType> &centroid, idxType idx, std::vector<node> *criticalSeed);

    void connectCentroidalNode(centroidType &centroid, idxType k, idxType i, vector<gnode> &graph, vector<idxType> &graphIdx);

    bool connectCriticalPoints(IndxCont &ng_sz, IndxCont &ng_idx, vector<gnode> &graph, vector<idxType> &graphIdx);

    void findDistinctElement(vector<idxType> *graphIdx);

    float _radius, _rmaxpt;

  //  vector<gnode> _graph;

  //  vector<idxType> _graphIdx;

    std::vector<node> _node;

    std::vector<node> _critcalNode;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _inCloud;

    vector <probfeatnode> _probval;

};

#endif // LINEEXTRACTION_H
