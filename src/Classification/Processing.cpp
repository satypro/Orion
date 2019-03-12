#include "ClassifiersFactory.h"
#include "Processing.h"
#include "FileRead.h"
#include "ClassifierTypes.h"
#include "SearchStructure.h"
#include "SearchType.h"
#include <iostream>
#include <fstream>
#include "graph.h"
#include <GL/gl.h>

#define LABEL_FILE "data/label/d4-alg2l.txt"
#define OPTIMAL_FILE "data/label/optimal.txt";

bool comparede(double i, double j) { return i<j; }

bool myfunction (std::pair<int, double>i, std::pair<int, double> j)
{

return ( (i.second < j.second));
}


//bool compareint(int i, int j) { return i<j; }

/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode	
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
				critical curve node and critical disc node
Calls			:	
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/

Processing::Processing(): _inCloud(new cl)
{ 
    _filePath = "NULL";
    _FileExtension = "NULL";
    _fileNo = "NULL";
    _ptclassifyObj = new tensor3dvoting();
    _ptclassifyObj2D = new tensor2d();
    _ptclassifyObj_DNV = new DiffusedNormalVoting();
    _ptclassifyObj_CM = new CovarianceMatrix();
    _ptclassifyObj_MCM = new ModCovarianceMatrix();
    _ptclassifyObj_CM2D = new CovarianceMatrix2D();
    _ptclassifyObj_Hessian = new Hessian();
    _ptclassifyObj_Boundary = new BoundaryTensor();
	_ptclassifyObj_Optimal_DNV = new DiffusedOptimal();
	_ptclassifyObj_Optimal_CoVariance = new CoVarianceOptimal();

    _utils = new Utils();

    _displayObj = new DisplayPoints(&_inCloud, &_intensity, &_probval, &_probval_tmp, &_ftPtsProp);

    _octree = new OctreeXYZ<PointT,LeafContainerT, BranchContainerT >(0.0005);  //resolution = 0.0005

    _rmin = 0.1;
    _rmax = 0.2;


   _rmaxpt = 500;
   _epsi = 0.5;
 //  _scale = 4.0;

   _scale = 3.0;
  
   _sigmax = 1.7;
   _sigmin = 0.2;
   _scalarMax = 1.0;
   _scalarMin = 0.0;
   _tensorType = 0;

  	
}
/*****************************************************************************************
Function Name		:	FeatureGraph::SelectSeedNode	
Purpose	of Function	:	classify the 3D points into various classes- curve node, disc node
				critical curve node and critical disc node
Calls			:	
Input Params		:	T *curve, T *disc, T *spherical
Output Params		:	S *curveNode, S *discNode, S *critCurveNode, S *critDiscNode
Return			:
Remarks			:
*****************************************************************************************/

void Processing::setParams(float rmin, float rmax, float scale, float eps, float sigmin, float sigmax, float scalarMin, float scalarMax)
{ 
   /*_rmin = rmin;
   _rmax = rmax;
  
   _epsi = eps;
   _sigmax = sigmax;
   _sigmin = sigmin;

   _epsi = 0.5;

    _scale = 3.0;
   _rmaxpt = 500;*/
//    _rmin = 0.02;
//    _rmax = 0.04;

//    _rmin = 0.008;
//    _rmax = 0.012;
    _rmin = rmin;
    _rmax = rmax;

          _rmaxpt = 500;
          _epsi = 0.5;
//          _scale = 5.0;

          _scale = scale;

          std::cout << "scale = " << scale << " _rmin = " << _rmin << " _rmax= " << _rmax << std::endl;

          _sigmax = 1.7;
          _sigmin = 0.2;

          _scalarMax = scalarMax;
          _scalarMin = scalarMin;

          _displayObj->scalarMin = scalarMin;
          _displayObj->scalarMax = scalarMax;
}
/*****************************************************************************************

*****************************************************************************************/

void Processing::setFilePath(std::string filePath) 
{ 
	_filePath.assign(filePath); 
}

/*****************************************************************************************

*****************************************************************************************/
bool Processing::fileRead()
{
	bool nErrorCode;

	if(_filePath == "NULL")
	{
   		std::cout<<"Invalid File Path"<<std::endl;
   		return false;
   	}
 	else
	{
        FileRead *readObject = new FileRead;
		nErrorCode = readObject->read(_filePath, _inCloud, _intensity);
		_FileExtension = readObject->getFileType(_filePath);

        string labelFile = LABEL_FILE;
        readObject->readLabels(labelFile, labels);
		
		string optimalFile = OPTIMAL_FILE;
		readObject->readLabels(optimalFile, optimalScale);

		delete readObject;
	}

   /* COmmented By Satendra 
    
   if(_FileExtension == "las" || _FileExtension == "ply" || _FileExtension == "off" || _FileExtension == "xyz")
            scale();
	buildOctree();*/

 
		/*New Code Entry*/
		Configuration* config = new Configuration();
		config->SetValue("Resolution", "125");
		SearchStructure structure = static_cast<SearchStructure>(OctTree); // KD-Tree or Oct-Tree
		search = SearchNeighbourFactory::GetNeighbourSearchDataStructure(structure);
		search->build(_inCloud, config);
		/*End New Code Entry*/

		cout << "total size " << _inCloud->points.size() << endl;
	return nErrorCode;
}

bool Processing::fileRead(string cloudFile, string labelFile, string optimalFile)
{
	bool nErrorCode;
	_filePath = cloudFile;

	if (_filePath == "NULL")
	{
		std::cout << "Invalid File Path" << std::endl;
		return false;
	}
	else
	{
		FileRead *readObject = new FileRead;
		nErrorCode = readObject->read(_filePath, _inCloud, _intensity);
		_FileExtension = readObject->getFileType(_filePath);

		string labelFile = labelFile;
		readObject->readLabels(labelFile, labels);

		string optimalFile = optimalFile;
		readObject->readLabels(optimalFile, optimalScale);

		delete readObject;
	}

	if (_FileExtension == "las" || _FileExtension == "ply" || _FileExtension == "off" || _FileExtension == "xyz")
		//scale();

	//buildOctree();

		cout << "total size " << _inCloud->points.size() << endl;

	return nErrorCode;
}

/*****************************************************************************************
*****************************************************************************************/
void Processing::scale()
{
    if(_inCloud->points.size() == 0)
        return;


    float minX, minY, minZ, maxX, maxY, maxZ; //,  minIntensity, maxIntensiy,
    float DelX, DelY, DelZ;

    float minxr, maxxr, minyr, maxyr, minzr, maxzr, delxr, delyr, delzr;
    float scale_fac = 2.0;

    vector<double> tempData;

    tempData.resize(_inCloud->points.size());

    for(size_t i = 0; i < _inCloud->points.size(); i++)
        tempData[i] = _inCloud->points[i].x;

    minX = *std::min_element(tempData.begin(), tempData.end(), comparede);
    maxX = *std::max_element(tempData.begin(), tempData.end(), comparede);

    for(size_t i = 0; i < _inCloud->points.size(); i++)
        tempData[i] = _inCloud->points[i].y;

    minY = *std::min_element(tempData.begin(), tempData.end(), comparede);
    maxY = *std::max_element(tempData.begin(), tempData.end(), comparede);

    for(size_t i = 0; i < _inCloud->points.size(); i++)
        tempData[i] = _inCloud->points[i].z;

    minZ = *std::min_element(tempData.begin(), tempData.end(), comparede);
    maxZ = *std::max_element(tempData.begin(), tempData.end(), comparede);

    tempData.clear();

    DelX = maxX - minX;
    DelY = maxY - minY;
    DelZ = maxZ - minZ;

    if(DelX >= DelY && DelX >= DelZ)
    {
            minxr = 0.0;
            maxxr = 1.0 * scale_fac;

            minyr = 0;
            maxyr = DelY/DelX * scale_fac;

            minzr = 0.0;
            maxzr =  (DelZ/DelX) * scale_fac;
    }
    else if(DelY >= DelX && DelY >= DelZ)
    {
            minxr = 0;
            maxxr = DelX/DelY * scale_fac;

            minyr = 0.0;
            maxyr = 1.0 * scale_fac;

            minzr = 0.0;
            maxzr = (DelZ/DelY) * scale_fac;
    }
    else if(DelZ >= DelX && DelZ >= DelY)
    {
            //cout<<"notify it "<<endl;
            minxr = 0.0;
            maxxr = DelX/DelZ * scale_fac;

            minyr = 0.0;
            maxyr = DelY/DelZ * scale_fac;

            minzr = 0.0;
            maxzr = 1.0 * scale_fac;
    }

    delxr = maxxr - minxr;
    delyr = maxyr - minyr;
    delzr = maxzr - minzr;

    cout<<"DelX "<<DelX<<endl;
    cout<<"DelY "<<DelY<<endl;
    cout<<"DelZ "<<DelZ<<endl;

    for(size_t i =0; i < _inCloud->points.size(); i++)
    {
        _inCloud->points[i].x = delxr *(_inCloud->points[i].x  - minX)/DelX;
        _inCloud->points[i].y = delyr * (_inCloud->points[i].y - minY)/DelY;
        _inCloud->points[i].z = delzr * (_inCloud->points[i].z - minZ)/DelZ;
    }

}
/*****************************************************************************************
*****************************************************************************************/
void Processing::buildOctree()
{
    _octree->setInputCloud (_inCloud);
    _octree->addPointsFromInputCloud ();

//    _displayObj->smoothTensorLines("sorted_tensorlines/sorted_tensorlines_bldg1_lminmax.txt",0);
//    _displayObj->smoothTensorLines("sorted_tensorlines/sorted_tensorlines_bldg2_lminmax.txt",0);
//    _displayObj->smoothTensorLines("sorted_tensorlines/sorted_tensorlines_bldg3_lminmax.txt",0);
//    _displayObj->smoothTensorLines("sorted_tensorlines/sorted_tensorlines_bldg4_lminmax.txt",0);
//    _displayObj->smoothTensorLines("sorted_tensorlines/sorted_tensorlines_bldg5_lminmax.txt",0);
//    _displayObj->smoothTensorLines("sorted_tensorlines/sorted_tensorlines_bldg6_lminmax.txt",0);
//    _displayObj->smoothTensorLines("sorted_tensorlines/sorted_tensorlines_bldg7_lminmax.txt",1);

    //_displayObj->smoothTensorLines("sorted_tensorlines/sorted_tensorlines_bldg8_lminmax.txt",1);
    //_displayObj->smoothTensorLines("sorted_tensorlines/sorted_tensorlines_bldg9_lminmax.txt",1);
    //_displayObj->smoothTensorLines("sorted_tensorlines/sorted_tensorlines_bldg10_lminmax.txt",1);
    //_displayObj->smoothTensorLines("sorted_tensorlines/sorted_tensorlines_bldg11_lminmax.txt",1);
    //_displayObj->smoothTensorLines("sorted_tensorlines/sorted_tensorlines_bldg12_lminmax.txt",1);

    ofstream fout("linesegments.txt");
    for(int i=0;i<_displayObj->_smoothTensorLinesCorrected.size();i++) {
        Line l = _displayObj->_smoothTensorLinesCorrected[i];
        fout<< l.p[0] << " " << l.p[1] << " " << l.p[2] << " " << l.q[0] << " " << l.q[1] << " " << l.q[2] <<"\n";
    }
    fout.close();

    ofstream fout2("lineSegmentsCorrected.txt");
    for(int i=0;i<_displayObj->_smoothTensorLinesPlanarityCorrected.size();i++) {
        Line l = _displayObj->_smoothTensorLinesPlanarityCorrected[i];
        fout2<< l.p[0] << " " << l.p[1] << " " << l.p[2] << " " << l.q[0] << " " << l.q[1] << " " << l.q[2] <<"\n";
    }
    fout2.close();

//    ofstream fout2("roofplanes.txt");
//    for(int i=0;i<_displayObj->_roofPlanes.size();i++) {
//        Rect rect = _displayObj->_roofPlanes[i];
//        fout2 << rect.p[0] << " " << rect.p[1] << " " << rect.p[2] << " "
//                      << rect.q[0] << " " << rect.q[1] << " " << rect.q[2] << " "
//                      << rect.r[0] << " " << rect.r[1] << " " << rect.r[2] << " "
//                       << rect.s[0] << " " << rect.s[1] << " " << rect.s[2] <<"\n";
//    }
//    fout2.close();
}

/*****************************************************************************************
*****************************************************************************************/
void Processing::classifyStructFeats()
{
	if(_sigmin <= 0.0 || _rmin <= 0.0 || _rmax <= 0.0 || _rmin >= _rmax || _epsi <= 0.0 || _sigmax <=0.0 || _sigmin >= _sigmax )
	{
		cout<<"invalid configuration parameters for classification module"<<endl;
		return;
	}

	// Process New Request Technique....
	ProcessRequest request;
	request.cloudFileName = "data/label/d4-alg2l.txt";
	request.optimalFileName = "data/label/optimal.txt";
	request.labelFileName = "data/3d_4.txt";

	request.classifierType = static_cast<int>(C_3DVT);
	request.dataStructure = static_cast<int>(OctTree);
	request.epsi = _epsi;
	request.rmin = _rmin;
	request.rmax = _rmax;
	request.rmaxpt = _rmaxpt;
	request.scale = _scale;
	request.searchType = static_cast<int>(Radius);
	processCloud(request);






	//_probval.resize(_inCloud->points.size());
	//for (size_t i = 0; i < _inCloud->points.size(); i++)
	//{
	//	_probval[i].optimalScale = optimalScale[i];
	//}

 //   switch(_tensorType) {
 //       case 0: // 3DVT-GET
 //           _ptclassifyObj_DNV->setInputCloud(_inCloud);
 //           _ptclassifyObj_DNV->setOctree(_octree);
 //           _ptclassifyObj_DNV->setParams(_rmin, _rmax, _rmaxpt,  _epsi, _scale);
 //           _ptclassifyObj_DNV->prob_measure(&_probval, &_accum);

 //           _ptclassifyObj_Boundary->setInputCloud(_inCloud);
 //           _ptclassifyObj_Boundary->setOctree(_octree);
 //           _ptclassifyObj_Boundary->setParams(_rmin, _rmax, _rmaxpt,  _epsi, _scale);
 //           _ptclassifyObj_Boundary->prob_measure(&_probval_tmp);

 //           // Update saliency of _probval by combining two tensors
 //           for (size_t i = 0; i <_inCloud->points.size(); i++)
 //           {
 //               if(_probval_tmp[i].csclcp[2] > _probval_tmp[i].csclcp[1])
 //               {
 //                   _probval[i].csclcp[0] = _probval_tmp[i].csclcp[2];
 //                   _probval[i].csclcp[1] = _probval_tmp[i].csclcp[1];
 //                   _probval[i].csclcp[2] = _probval_tmp[i].csclcp[0];

 //                   float lamda2 = _probval[i].eigenvalues[2];
 //                   float lamda1 = 0.5 * lamda2 * (3 / _probval[i].csclcp[0] - 1);
 //                   float lamda0 = lamda1;
 //                   _probval[i].anisotropy = (lamda1 -lamda2) / lamda0;
 //                   _probval[i].sphericity = (lamda0 -lamda2) / lamda0;

 //                   _utils->correctMisclassifiedPoints(_probval[i].eigenvalues,_probval[i].evecs,
 //                                                      _probval_tmp[i].csclcp[2], _probval_tmp[i].csclcp[1],_accum[i]);
 //               }
 //           }
 //           _ptclassifyObj_DNV->filterPointTypes(_rmax,_probval, &_accum);
 //           _utils->eigen_decomposition_to_file(_accum);
 //           break;
 //       case 1: //3DVT
	//		
	//		_ptclassifyObj_Optimal_DNV->setInputCloud(_inCloud);
	//		_ptclassifyObj_Optimal_DNV->setOctree(_octree);
	//		_ptclassifyObj_Optimal_DNV->setParams(_rmin, _rmax, _rmaxpt, _epsi, _scale);
	//		_ptclassifyObj_Optimal_DNV->prob_measure(&_probval, &_accum, optimalScale);
	//		_utils->eigen_decomposition_to_file(_accum);
	//		
	//			
	//		/*
 //           _ptclassifyObj_DNV->setInputCloud(_inCloud);
 //           _ptclassifyObj_DNV->setOctree(_octree);
 //           _ptclassifyObj_DNV->setParams(_rmin, _rmax, _rmaxpt,  _epsi, _scale);
 //           _ptclassifyObj_DNV->prob_measure(&_probval, &_accum);
 //           _utils->eigen_decomposition_to_file(_accum);
	//		*/
 //           break;
 //       case 2: //3DCM
	//		
	//		/*
	//		_ptclassifyObj_Optimal_CoVariance->setInputCloud(_inCloud);
	//		_ptclassifyObj_Optimal_CoVariance->setOctree(_octree);
	//		_ptclassifyObj_Optimal_CoVariance->setParams(_rmin, _rmax, _rmaxpt, _epsi, _scale);
	//		_ptclassifyObj_Optimal_CoVariance->prob_measure(&_probval, &_accum, optimalScale);
	//		_utils->eigen_decomposition_to_file(_accum);
	//		*/

 //           
	//		_ptclassifyObj_CM->setInputCloud(_inCloud);
 //           _ptclassifyObj_CM->setOctree(_octree);
 //           _ptclassifyObj_CM->setParams(_rmin, _rmax, _rmaxpt,  _epsi, _scale);
 //           _ptclassifyObj_CM->prob_measure(&_probval,&_accum);
 //           _utils->eigen_decomposition_to_file(_accum);
	//		
 //           break;
 //       case 3: //2DGET
 //           _ptclassifyObj_Boundary->setInputCloud(_inCloud);
 //           _ptclassifyObj_Boundary->setOctree(_octree);
 //           _ptclassifyObj_Boundary->setParams(_rmin, _rmax, _rmaxpt,  _epsi, _scale);
 //           _ptclassifyObj_Boundary->prob_measure(&_probval);
 //           break;
 //       case 4: //3DMCM
 //          _ptclassifyObj_MCM->setInputCloud(_inCloud);
 //          _ptclassifyObj_MCM->setOctree(_octree);
 //          _ptclassifyObj_MCM->setParams(_rmin, _rmax, _rmaxpt,  _epsi, _scale);
 //          _ptclassifyObj_MCM->prob_measure(&_probval);
 //          break;
 //       case 5: //Hessian
 //           _ptclassifyObj_Hessian->setInputCloud(_inCloud);
 //           _ptclassifyObj_Hessian->setOctree(_octree);
 //           _ptclassifyObj_Hessian->setParams(_rmin, _rmax, _rmaxpt,  _epsi, _scale);
 //           _ptclassifyObj_Hessian->prob_measure(&_probval);
 //           break;
 //       case 6: // 2DCM
 //           _ptclassifyObj_CM2D->setInputCloud(_inCloud);
 //           _ptclassifyObj_CM2D->setOctree(_octree);
 //           _ptclassifyObj_CM2D->setParams(_rmin, _rmax, _rmaxpt,  _epsi, _scale);
 //           _ptclassifyObj_CM2D->prob_measure(&_probval);
 //           break;
 //   }

 //   _utils->setInputCloud(_inCloud);
 //   _utils->setOctree(_octree);
 //   _utils->setParams(_rmin, _rmax, _rmaxpt,  _epsi, _scale);

 //   _utils->computeDoNs_MSO(_probval, _rmin, _rmax);

 //   _utils->triangulate(_rmin,_probval);
 //   _utils->generateTensorLines3(_probval);
 //   cout << "generated tensor lines" << endl;


 //       // Add labels
 //       for (size_t i = 0; i <_inCloud->points.size(); i++)
 //       {
 //           _probval[i].label = labels[i];
 //       }

 //       cout << "added labels" << endl;
 //       _displayObj->roofRMSEerror();

 //       structFeatClassification();
 //       curveGraphExtraction();

 //       ofstream fout("prob.txt");
 //       fout<<_inCloud->points.size()<<"\n";
 //       fout<<_rmin<<"\n";
 //       fout<<_rmax<<"\n";
 //       fout<<_scale<<"\n";
 //       fout<<_epsi<<"\n";

 //      for(size_t i =0; i < _inCloud->points.size(); i++)
 //      {
 //          fout<< _inCloud->points[i].x << " " << _inCloud->points[i].y <<" "<< _inCloud->points[i].z <<" "<<_intensity[i]    <<" ";
 //          fout<< _probval[i].prob[0]   << " " << _probval[i].prob[1]   <<" "<< _probval[i].prob[2]   <<" ";
	//	   fout<< _probval[i].csclcp[0] << " " << _probval[i].csclcp[1] <<" "<< _probval[i].csclcp[2] << " " << _ftPtsProp[i] << " ";
	//	   fout<< _probval[i].planarity << " " << _probval[i].anisotropy << " " << _probval[i].sphericity << " ";
	//	   fout<< _probval[i].linearity << " " << _probval[i].omnivariance << " " << _probval[i].eigenentropy <<  "\n";

 //      }
 //      fout.close();
	//   cout << "at end of pprocessing" << endl;
}


void Processing::processResult()
{
	_utils->setInputCloud(_inCloud);
	_utils->setOctree(_octree);
	_utils->setParams(_rmin, _rmax, _rmaxpt, _epsi, _scale);

	_utils->computeDoNs_MSO(_probval, _rmin, _rmax);

	_utils->triangulate(_rmin, _probval);
	_utils->generateTensorLines3(_probval);
	cout << "generated tensor lines" << endl;

	//// Add labels
	//for (size_t i = 0; i < _inCloud->points.size(); i++)
	//{
	//	_probval[i].label = labels[i];
	//}

	cout << "added labels" << endl;
	_displayObj->roofRMSEerror();

	structFeatClassification();
	curveGraphExtraction();

	ofstream fout("prob.txt");
	fout << _inCloud->points.size() << "\n";
	fout << _rmin << "\n";
	fout << _rmax << "\n";
	fout << _scale << "\n";
	fout << _epsi << "\n";

	for (size_t i = 0; i < _inCloud->points.size(); i++)
	{
		fout << _inCloud->points[i].x << " " << _inCloud->points[i].y << " " << _inCloud->points[i].z << " " << _intensity[i] << " ";
		fout << _probval[i].prob[0] << " " << _probval[i].prob[1] << " " << _probval[i].prob[2] << " ";
		fout << _probval[i].csclcp[0] << " " << _probval[i].csclcp[1] << " " << _probval[i].csclcp[2] << " " << _ftPtsProp[i] << " ";
		fout << _probval[i].planarity << " " << _probval[i].anisotropy << " " << _probval[i].sphericity << " ";
		fout << _probval[i].linearity << " " << _probval[i].omnivariance << " " << _probval[i].eigenentropy << "\n";

	}
	fout.close();
	cout << "at end of pprocessing" << endl;
}


/******************************************************************************************/
bool Processing::structFeatClassification()
{
    size_t i, pts_sz = _inCloud->points.size();
	
	if(_probval.size() == 0 || pts_sz != _probval.size())
	{
		cout<<"there are no points to classify "<<endl;
                return false;
	}

        _ftPtsProp.resize(pts_sz);

        size_t l_c = 0, l_s =  0, l_p =0;

        float min_cl = 100;
        float min_cp = 100;
        float min_cs = 100;
        float min_anisotropy = 100;
        float min_sphericity = 100;
        float min_don = 100;
        float max_cl = -100;
        float max_cp = -100;
        float max_cs = -100;
        float max_anisotropy = -100;
        float max_sphericity = -100;
        float max_don = -100;
        int numRoofPoints = 0;
        int numRoofTriangulationPoints = 0;

        for(i = 0; i < pts_sz; i++)
        {
            if(_probval[i].label==5)
                numRoofPoints++;
            if(_probval[i].label==5 && _probval[i].triangles.size()>0)
                numRoofTriangulationPoints++;

            if(_probval[i].csclcp[0]<min_cs)
                min_cs = _probval[i].csclcp[0];
            if(_probval[i].csclcp[0]>max_cs)
                max_cs = _probval[i].csclcp[0];

            if(_probval[i].csclcp[1]<min_cl)
                min_cl = _probval[i].csclcp[1];
            if(_probval[i].csclcp[1]>max_cl)
                max_cl = _probval[i].csclcp[1];

            if(_probval[i].csclcp[2]<min_cp)
                min_cp = _probval[i].csclcp[2];
            if(_probval[i].csclcp[2]>max_cp)
                max_cp = _probval[i].csclcp[2];

            if(_probval[i].anisotropy<min_anisotropy)
                min_anisotropy = _probval[i].anisotropy;
            if(_probval[i].anisotropy>max_anisotropy)
                max_anisotropy = _probval[i].anisotropy;

            if(_probval[i].sphericity<min_sphericity)
                min_sphericity = _probval[i].sphericity;
            if(_probval[i].sphericity>max_sphericity)
                max_sphericity = _probval[i].sphericity;

            if(_probval[i].don<min_don)
                min_don = _probval[i].don;
            if(_probval[i].don>max_don)
                max_don = _probval[i].don;
			// Change done by SATENDRA
			if (_probval[i].csclcp[0] >= _probval[i].csclcp[1] && _probval[i].csclcp[0] >= _probval[i].csclcp[2])
			{
				_ftPtsProp[i] = 0;
			}
			else
			{
				if (_probval[i].csclcp[0] >= _probval[i].csclcp[1] && _probval[i].csclcp[0] >= _probval[i].csclcp[2])
				{
					_ftPtsProp[i] = 1;
					l_s++;
				}

				else if (_probval[i].csclcp[1] >= _probval[i].csclcp[0] && _probval[i].csclcp[1] >= _probval[i].csclcp[2])
				{
					_ftPtsProp[i] = 2;
					l_c++;
				}

				else if (_probval[i].csclcp[2] >= _probval[i].csclcp[1] && _probval[i].csclcp[2] >= _probval[i].csclcp[0])
				{
					_ftPtsProp[i] = 3;
					l_p++;
				}
				else
					_ftPtsProp[i] = 0;
			}
        }

        cout<<"number of line-type features "<<l_c<<endl;
        cout<<"number of  critical features "<<l_s<<endl;
        cout<<"number of surface-type features "<<l_p<<endl;
        cout <<"min cl = " << min_cl << " max cl = "<< max_cl << endl;
        cout <<"min cp = " << min_cp << " max cp = "<< max_cp << endl;
        cout <<"min cs = " << min_cs << " max cs = "<< max_cs << endl;
        cout <<"min anisotropy = " << min_anisotropy << " max anisotropy = "<< max_anisotropy << endl;
        cout <<"min sphericity = " << min_sphericity << " max sphericity = "<< max_sphericity << endl;
        cout <<"min don = " << min_don << " max don = "<< max_don << endl;
        cout << "# roof points = " << numRoofPoints << " # triangulation roof points = " << numRoofTriangulationPoints << endl;

        return true;
}

/*****************************************************************************************
*****************************************************************************************/
int Processing::getDataSize()
{
	return _inCloud->points.size();
}

/*****************************************************************************************
*****************************************************************************************/
void Processing::reset()
{
	_probval.clear();
    _accum.clear();
	_ftPtsProp.clear();
}

/*****************************************************************************************
*****************************************************************************************/
void Processing::clear()
{
	_inCloud->points.clear();

	_intensity.clear();

    for(size_t i =0; i < _graph.size(); i++)
    _graph[i].edge.clear();

    _graph.clear();
    _graphIdx.clear();

	reset();

}
/*****************************************************************************************
*****************************************************************************************/
void Processing::setDisplayMode(int displaymode, int pointmode)
 {
 	_displaymode = displaymode;
 	_pointmode = pointmode;
 }


/*****************************************************************************************/
void Processing::setTensorType(int type)
 {
    _tensorType= type;
    std::cout << "setting tensor type = " << type << std::endl;
 }

/*****************************************************************************************
 * Links to UI. Overrides displaymode 4 (Line features) with Curve graph
*****************************************************************************************/
void Processing::display()
{
    if(_displaymode == 0 || (_displaymode == 1 && _pointmode == 0))
	{

           // if((!_FileExtension.compare("las") || !_FileExtension.compare("txt")))
                    _displayObj->lasDisplay();
          //  else if (!_FileExtension.compare("ply"))
                 //   _displayObj->plyDisplay();

	}
    else if(_displaymode == 1 && _pointmode == 4)
        {
           displayGraph();
           _displayObj->displayBoundary();
        }
    else if(_displaymode == 1)
     _displayObj->renderStructFDs(_pointmode);
}

/*****************************************************************************************
 * Displays Kreylo's curve graph
*****************************************************************************************/

void Processing::displayGraph()
{
    if(_graph.size() == 0)
        return;

 glPointSize(1.0);
    glBegin(GL_POINTS);

    for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
        if(labels[i] == 5)
        {
            double temp = _intensity[i];
            glColor3d(temp, temp, temp) ;
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
        }
        }
    glEnd();


    glLineWidth(1.5);
    glBegin(GL_LINES);

    for (size_t i = 0 ; i < _graph.size() ; i++)
    {
        int m = _graph[i].nd.idx;
        if(labels[m] == 5)
        {


        for(size_t j = 0; j < _graph[i].edge.size(); j++)
        {
            int n = _graph[i].edge[j].idx;

           // if(label[n] == 5)
            {

            glColor3d(1.0, 0.0, 0.0) ;

            glVertex3f(_inCloud->points[m].x, _inCloud->points[m].y, _inCloud->points[m].z);

            glVertex3f(_inCloud->points[n].x, _inCloud->points[n].y, _inCloud->points[n].z);
            }
        }
        }
    }

    glEnd() ;

    return ;
}

/*****************************************************************************************
 * Implemetation of Kreylo's curve graph
*****************************************************************************************/

void Processing::curveGraphExtraction()
{
    std::vector<std::pair<int, double> > featureNode;
    std::vector<std::pair<int, double> > critFeatureNode;

    for(size_t i =0; i < _inCloud->points.size(); i++)
    {
         if(_probval[i].csclcp[0] > _probval[i].csclcp[1] && _probval[i].csclcp[0] > _probval[i].csclcp[2])
         {
             critFeatureNode.push_back(std::make_pair(i, _probval[i].csclcp[0]));


         }
         if(_probval[i].csclcp[1] >= _probval[i].csclcp[0] && _probval[i].csclcp[1] >= _probval[i].csclcp[2])
         {
             int idx = i;
             double val =  _probval[i].csclcp[1];
             featureNode.push_back(std::make_pair(idx, val));

         }

    }


    std::sort (featureNode.begin(), featureNode.end(), myfunction);

    std::sort (critFeatureNode.begin(), critFeatureNode.end(), myfunction);

    cout<<"sorting done "<<endl;

    std::vector<node> curveSeeds;

    std::vector<node> critCurveSeeds;

   for(size_t i =0; i < featureNode.size(); i++ )
    {
        node temp;
        temp.idx = featureNode[i].first;
        temp.status = false;

        curveSeeds.push_back(temp);

    }

    for(size_t i =0; i < critFeatureNode.size(); i++ )
    {
        node temp;
        temp.idx = critFeatureNode[i].first;
        temp.status = false;
        critCurveSeeds.push_back(temp);
    }

    critFeatureNode.clear();

    featureNode.clear();


    graph curveGraphHandle;

    curveGraphHandle.setParams(_rmax, _rmaxpt);
    curveGraphHandle.setNode(curveSeeds);
    curveGraphHandle.setCriticalNode(critCurveSeeds);
    curveGraphHandle.setInputcloud(_inCloud);
    curveGraphHandle.setSaliencyMaps(_probval);
    curveGraphHandle.constructGraph(_graph, _graphIdx);

    cout<<"graph extraction done "<<endl;

    ofstream fout("Area4 edgeInfo.txt");
    fout<< _graph.size()<<endl;

   for (size_t i = 0 ; i < _graph.size() ; i++)
   {
       int m = _graph[i].nd.idx;
       fout<<m<<" "<<_graph[i].edge.size();

       for(size_t j = 0; j < _graph[i].edge.size(); j++)
       {
           int n = _graph[i].edge[j].idx;

           fout<<" "<<n;
       }

       fout<<"\n";
   }
   fout.close();
}

void Processing::buildStructure(ProcessRequest request)
{
	// get all the file paths and pass it to the fileRead
	fileRead(request.cloudFileName, request.labelFileName, request.optimalFileName);

	Configuration* config = new Configuration();
	config->SetValue("Resolution", "125");
	SearchStructure structure = static_cast<SearchStructure>(request.dataStructure); // KD-Tree or Oct-Tree
	search = SearchNeighbourFactory::GetNeighbourSearchDataStructure(structure);
	search->build(_inCloud, config);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::getCloud()
{
	return _inCloud;
}

void Processing::processCloud(ProcessRequest request)
{
	ClassifierTypes classifierType = static_cast<ClassifierTypes>(request.classifierType); // Descriptor type
	Classifier* classifier = ClassifiersFactory::GetClassifier(classifierType);
	classifier->setSearch(search);
	classifier->setInputCloud(_inCloud);

	//vector <probfeatnode> _probval;
	//vector <tensorType> _accum; // aggregated tensor

	//for loop each will be passed with the configuration required...
	classifier->process(request, &_probval, &_accum);
 	processResult();
}

// This is the request to process against each point
// It can take the selection here for the discriptor type for each point
// But the Search Structure whether Kdtree or Octree will be fixed though.
void Processing::processPoint(ProcessRequest request, int index)
{
	ClassifierTypes classifierType = static_cast<ClassifierTypes>(request.classifierType); // Descriptor type
	Classifier* classifier = ClassifiersFactory::GetClassifier(classifierType);
	classifier->setSearch(search);
	classifier->setInputCloud(_inCloud);

	probfeatnode _probval;
	tensorType _accum; // aggregated tensor

	classifier->processPoint(request, &_probval, &_accum, index);
}