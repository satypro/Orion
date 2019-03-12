#include "octree_pointcloud.h"


/*****************************************************************************************

Function Name		:	PointClassificaton::outlierRemoval	
Purpose	of Function	:	Outlier Removal
Author/date		:	Beena
Modified by/date	:
Calls			:		
Input Params		:
Output Params		:	std::vector<int> sizes, std::vector<int> data
Return			:
Remarks			:

*****************************************************************************************/
void octree_pointcloud::build_octree(PtContainer &points)
{
	pcl::gpu::Octree::PointCloud cloud_device;
  	//prepare device cloud
  	cloud_device.upload(points); 
  	/** \brief Sets cloud for which octree is built on GPU */ 
    octree_device.setCloud(cloud_device);
 	/** \brief Performs parallel octree building on GPU */
 	octree_device.build();
}

/*****************************************************************************************

Function Name		:	PointClassificaton::outlierRemoval	
Purpose	of Function	:	Outlier Removal
Author/date		:	Beena
Modified by/date	:
Calls			:		
Input Params		:
Output Params		:	std::vector<int> sizes, std::vector<int> data
Return			:
Remarks			:

*****************************************************************************************/
bool octree_pointcloud::rnearest_search( PtContainer &queries, IndicesContainer &indices, IndicesContainer *sizes, IndicesContainer *data)
{
	pcl::gpu::Octree::Queries queries_device;
  	pcl::gpu::Octree::Indices qindices;
	
	//upload queries
  	queries_device.upload(queries); 
  	qindices.upload(indices);

	//prepare output buffers on device
  	pcl::gpu::NeighborIndices result_device(queries_device.size(), _maxNeigh);
 	//search GPU shared with indices   
 	 octree_device.radiusSearch(queries_device, qindices, _radius, _maxNeigh, result_device);
 	//download results
	result_device.sizes.download(*sizes);
 	result_device.data.download(*data);

        return true;
	
}

