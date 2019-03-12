#ifndef MYOCTREE_READ_H
#define MYOCTREE_READ_H


#include <pcl/gpu/containers/device_array.h>
#include "data_type.h"
#include <pcl/gpu/octree/octree.hpp>
class octree_pointcloud
	{
	public:
	typedef std::vector<pcl::PointXYZ> PtContainer;
	typedef std::vector<idxType> IndicesContainer;
	
	octree_pointcloud() {}

	//octree_pointcloud(float radius, idxType maxNeigh) : _radius(radius),  _maxNeigh(maxNeigh) {}
	~octree_pointcloud() {}
	bool rnearest_search( PtContainer &queries, IndicesContainer &indices, IndicesContainer *sizes, IndicesContainer *data);
	void build_octree(PtContainer &points);
	
	void setradius(float radius, idxType rmaxpts)
	{
	_radius = radius;
	_maxNeigh = rmaxpts;
	}


	private:
	float _radius;
	idxType  _maxNeigh;
	pcl::gpu::Octree octree_device; 
	};

#endif
