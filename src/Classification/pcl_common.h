#include "cutil_math.h"
#include <limits>
#include <float.h>
//#include <pcl/cuda/common/eigen.h>

/*****************************************************************************************

*****************************************************************************************/
  inline __host__ __device__ bool isMuchSmallerThan (float x, float y)
    {
      float prec_sqr = FLT_EPSILON * FLT_EPSILON; // copied from <eigen>/include/Eigen/src/Core/NumTraits.h
      return x * x <= prec_sqr * y * y;
    }
/*****************************************************************************************


*****************************************************************************************/
inline __host__ __device__ float3 unitOrthogonal (const float3& src)
    {
      float3 perp;
      /* Let us compute the crossed product of *this with a vector
       * that is not too close to being colinear to *this.
       */
  
      /* unless the x and y coords are both close to zero, we can
       * simply take ( -y, x, 0 ) and normalize it.
       */
      if((!isMuchSmallerThan(src.x, src.z))
      || (!isMuchSmallerThan(src.y, src.z)))
      {   
        float invnm = 1.0f / sqrtf (src.x*src.x + src.y*src.y);
        perp.x = -src.y*invnm;
        perp.y = src.x*invnm;
        perp.z = 0.0f;
      }   
      /* if both x and y are close to zero, then the vector is close
       * to the z-axis, so it's far from colinear to the x-axis for instance.
       * So we take the crossed product with (1,0,0) and normalize it.
       */
      else
      {   
        float invnm = 1.0f / sqrtf (src.z*src.z + src.y*src.y);
        perp.x = 0.0f;
        perp.y = -src.z*invnm;
        perp.z = src.y*invnm;
      }   
  
      return perp;
    }

/*****************************************************************************************

*****************************************************************************************/
    inline __host__ __device__ void computeRoots2 (const float& b, const float& c, float3& roots)
  	{
  		roots.x = 0.0f;
  		float d = b * b - 4.0f * c;
  		if (d < 0.0f) // no real roots!!!! THIS SHOULD NOT HAPPEN!
  			d = 0.0f;
  
  		float sd = sqrt (d);
  		
  		roots.z = 0.5f * (b + sd);
  		roots.y = 0.5f * (b - sd);
  	}

/*****************************************************************************************


*****************************************************************************************/
inline __host__ __device__ void swap (float& a, float& b)
    {
      float c(a); a=b; b=c;
    }


/*****************************************************************************************



*****************************************************************************************/
// template<typename Matrix, typename Roots>
    inline __host__ __device__ void
    computeRoots (const CovarianceMatrix& m, float3& roots)
    {
      // The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0. The
      // eigenvalues are the roots to this equation, all guaranteed to be
      // real-valued, because the matrix is symmetric.
      float c0 = m.data[0].x*m.data[1].y*m.data[2].z
                  + 2.0f * m.data[0].y*m.data[0].z*m.data[1].z
                         - m.data[0].x*m.data[1].z*m.data[1].z
                         - m.data[1].y*m.data[0].z*m.data[0].z
                         - m.data[2].z*m.data[0].y*m.data[0].y;
      float c1 = m.data[0].x*m.data[1].y -
                  m.data[0].y*m.data[0].y +
                  m.data[0].x*m.data[2].z -
                  m.data[0].z*m.data[0].z +
                  m.data[1].y*m.data[2].z -
                  m.data[1].z*m.data[1].z;
      float c2 = m.data[0].x + m.data[1].y + m.data[2].z;
  
  
   if (fabs(c0) < FLT_EPSILON) // one root is 0 -> quadratic equation
   computeRoots2 (c2, c1, roots);
   else
   {
   const float s_inv3 = 1.0f/3.0f;
   const float s_sqrt3 = sqrtf (3.0f);
   // Construct the parameters used in classifying the roots of the equation
   // and in solving the equation for the roots in closed form.
   float c2_over_3 = c2 * s_inv3;
   float a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
   if (a_over_3 > 0.0f)
   a_over_3 = 0.0f;
  
   float half_b = 0.5f * (c0 + c2_over_3 * (2.0f * c2_over_3 * c2_over_3 - c1));
  
   float q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
   if (q > 0.0f)
   q = 0.0f;
  
   // Compute the eigenvalues by solving for the roots of the polynomial.
   float rho = sqrtf (-a_over_3);
   float theta = std::atan2 (sqrtf (-q), half_b) * s_inv3;
   float cos_theta = cos (theta);
   float sin_theta = sin (theta);
   roots.x = c2_over_3 + 2.f * rho * cos_theta;
   roots.y = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
   roots.z = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);
  
   // Sort in increasing order.
   if (roots.x >= roots.y)
   swap (roots.x, roots.y);
   if (roots.y >= roots.z)
   {
   swap (roots.y, roots.z);
   if (roots.x >= roots.y)
   swap (roots.x, roots.y);
   }
  
   if (roots.x <= 0.0f) // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
   computeRoots2 (c2, c1, roots);
   }
    }
  

/*****************************************************************************************

*****************************************************************************************/
 inline __host__ __device__ void 
    eigen33 (const CovarianceMatrix& mat, CovarianceMatrix& evecs, float3& evals)
    {
      evals = evecs.data[0] = evecs.data[1] = evecs.data[2] = make_float3 (0.0f, 0.0f, 0.0f);
  
      // Scale the matrix so its entries are in [-1,1].  The scaling is applied
      // only when at least one matrix entry has magnitude larger than 1.
  
      //Scalar scale = mat.cwiseAbs ().maxCoeff ();
      float3 scale_tmp = fmaxf (fmaxf (fabs (mat.data[0]), fabs (mat.data[1])), fabs (mat.data[2]));
      float scale = fmaxf (fmaxf (scale_tmp.x, scale_tmp.y), scale_tmp.z);
      if (scale <= FLT_MIN)
      	scale = 1.0f;
      
      CovarianceMatrix scaledMat;
      scaledMat.data[0] = mat.data[0] / scale;
      scaledMat.data[1] = mat.data[1] / scale;
      scaledMat.data[2] = mat.data[2] / scale;
  
      // Compute the eigenvalues
      computeRoots (scaledMat, evals);
      
  		if ((evals.z-evals.x) <= FLT_EPSILON)
  		{
  			// all three equal
  			evecs.data[0] = make_float3 (1.0f, 0.0f, 0.0f);
  			evecs.data[1] = make_float3 (0.0f, 1.0f, 0.0f);
  			evecs.data[2] = make_float3 (0.0f, 0.0f, 1.0f);
  		}
  		else if ((evals.y-evals.x) <= FLT_EPSILON)
  		{
  			// first and second equal
  			CovarianceMatrix tmp;
  			tmp.data[0] = scaledMat.data[0];
  			tmp.data[1] = scaledMat.data[1];
  			tmp.data[2] = scaledMat.data[2];
  
        tmp.data[0].x -= evals.z;
        tmp.data[1].y -= evals.z;
        tmp.data[2].z -= evals.z;
  
  			float3 vec1 = cross (tmp.data[0], tmp.data[1]);
  			float3 vec2 = cross (tmp.data[0], tmp.data[2]);
  			float3 vec3 = cross (tmp.data[1], tmp.data[2]);
  
  			float len1 = dot (vec1, vec1);
  			float len2 = dot (vec2, vec2);
  			float len3 = dot (vec3, vec3);
  
  			if (len1 >= len2 && len1 >= len3)
  			 	evecs.data[2] = vec1 / sqrtf (len1);
  			else if (len2 >= len1 && len2 >= len3)
  		 		evecs.data[2] = vec2 / sqrtf (len2);
  			else
  				evecs.data[2] = vec3 / sqrtf (len3);
  		
  			evecs.data[1] = unitOrthogonal (evecs.data[2]); 
  			evecs.data[0] = cross (evecs.data[1], evecs.data[2]);
  		}
  		else if ((evals.z-evals.y) <= FLT_EPSILON)
  		{
  			// second and third equal
  			CovarianceMatrix tmp;
  			tmp.data[0] = scaledMat.data[0];
  			tmp.data[1] = scaledMat.data[1];
  			tmp.data[2] = scaledMat.data[2];
        tmp.data[0].x -= evals.x;
        tmp.data[1].y -= evals.x;
        tmp.data[2].z -= evals.x;
  
  			float3 vec1 = cross (tmp.data[0], tmp.data[1]);
  			float3 vec2 = cross (tmp.data[0], tmp.data[2]);
  			float3 vec3 = cross (tmp.data[1], tmp.data[2]);
  
  			float len1 = dot (vec1, vec1);
  			float len2 = dot (vec2, vec2);
  			float len3 = dot (vec3, vec3);
  
  			if (len1 >= len2 && len1 >= len3)
  			 	evecs.data[0] = vec1 / sqrtf (len1);
  			else if (len2 >= len1 && len2 >= len3)
  		 		evecs.data[0] = vec2 / sqrtf (len2);
  			else
  				evecs.data[0] = vec3 / sqrtf (len3);
  		
  			evecs.data[1] = unitOrthogonal (evecs.data[0]);
  			evecs.data[2] = cross (evecs.data[0], evecs.data[1]);
  		}
  		else
  		{
  			CovarianceMatrix tmp;
  			tmp.data[0] = scaledMat.data[0];
  			tmp.data[1] = scaledMat.data[1];
  			tmp.data[2] = scaledMat.data[2];
        tmp.data[0].x -= evals.z;
        tmp.data[1].y -= evals.z;
        tmp.data[2].z -= evals.z;
  
  			float3 vec1 = cross (tmp.data[0], tmp.data[1]);
  			float3 vec2 = cross (tmp.data[0], tmp.data[2]);
  			float3 vec3 = cross (tmp.data[1], tmp.data[2]);
  
  			float len1 = dot (vec1, vec1);
  			float len2 = dot (vec2, vec2);
  			float len3 = dot (vec3, vec3);
  
  			float mmax[3];
  		  unsigned int min_el = 2;
  		  unsigned int max_el = 2;
  		  if (len1 >= len2 && len1 >= len3)
  		  {
  		    mmax[2] = len1;
  		    evecs.data[2] = vec1 / sqrtf (len1);
  		  }
  		  else if (len2 >= len1 && len2 >= len3)
  		  {
  		    mmax[2] = len2;
  		    evecs.data[2] = vec2 / sqrtf (len2);
  		  }
  		  else
  		  {
  		    mmax[2] = len3;
  		    evecs.data[2] = vec3 / sqrtf (len3);
  		  }
  
  			tmp.data[0] = scaledMat.data[0];
  			tmp.data[1] = scaledMat.data[1];
  			tmp.data[2] = scaledMat.data[2];
        tmp.data[0].x -= evals.y;
        tmp.data[1].y -= evals.y;
        tmp.data[2].z -= evals.y;
  
  			vec1 = cross (tmp.data[0], tmp.data[1]);
  			vec2 = cross (tmp.data[0], tmp.data[2]);
  			vec3 = cross (tmp.data[1], tmp.data[2]);
  
  			len1 = dot (vec1, vec1);
  			len2 = dot (vec2, vec2);
  			len3 = dot (vec3, vec3);
  		  if (len1 >= len2 && len1 >= len3)
  		  {
  		    mmax[1] = len1;
  		    evecs.data[1] = vec1 / sqrtf (len1);
  		    min_el = len1 <= mmax[min_el]? 1: min_el;
  		    max_el = len1 > mmax[max_el]? 1: max_el;
  		  }
  		  else if (len2 >= len1 && len2 >= len3)
  		  {
  		    mmax[1] = len2;
  		    evecs.data[1] = vec2 / sqrtf (len2);
  		    min_el = len2 <= mmax[min_el]? 1: min_el;
  		    max_el = len2 > mmax[max_el]? 1: max_el;
  		  }
  		  else
  		  {
  		    mmax[1] = len3;
  		    evecs.data[1] = vec3 / sqrtf (len3);
  		    min_el = len3 <= mmax[min_el]? 1: min_el;
  		    max_el = len3 > mmax[max_el]? 1: max_el;
  		  }
  		  
  			tmp.data[0] = scaledMat.data[0];
  			tmp.data[1] = scaledMat.data[1];
  			tmp.data[2] = scaledMat.data[2];
        tmp.data[0].x -= evals.x;
        tmp.data[1].y -= evals.x;
        tmp.data[2].z -= evals.x;
  
  			vec1 = cross (tmp.data[0], tmp.data[1]);
  			vec2 = cross (tmp.data[0], tmp.data[2]);
  			vec3 = cross (tmp.data[1], tmp.data[2]);
  
  			len1 = dot (vec1, vec1);
  			len2 = dot (vec2, vec2);
  			len3 = dot (vec3, vec3);
  		  if (len1 >= len2 && len1 >= len3)
  		  {
  		    mmax[0] = len1;
  		    evecs.data[0] = vec1 / sqrtf (len1);
  		    min_el = len3 <= mmax[min_el]? 0: min_el;
  		    max_el = len3 > mmax[max_el]? 0: max_el;
  		  }
  		  else if (len2 >= len1 && len2 >= len3)
  		  {
  		    mmax[0] = len2;
  		    evecs.data[0] = vec2 / sqrtf (len2);
  		    min_el = len3 <= mmax[min_el]? 0: min_el;
  		    max_el = len3 > mmax[max_el]? 0: max_el; 		
  		  }
  		  else
  		  {
  		    mmax[0] = len3;
  		    evecs.data[0] = vec3 / sqrtf (len3);
  		    min_el = len3 <= mmax[min_el]? 0: min_el;
  		    max_el = len3 > mmax[max_el]? 0: max_el;	  
  		  }
  		  
  		  unsigned mid_el = 3 - min_el - max_el;
  		  evecs.data[min_el] = normalize (cross (evecs.data[(min_el+1)%3], evecs.data[(min_el+2)%3] ));
  	      evecs.data[mid_el] = normalize (cross (evecs.data[(mid_el+1)%3], evecs.data[(mid_el+2)%3] ));
  		}
  	  // Rescale back to the original size.
  	  evals *= scale;
    }



/*****************************************************************************************

Function Name		:	PointClassificaton::outlierRemoval	
Purpose	of Function	:	Outlier Removal
Author/date		:	Beena
Modified by/date	:
Calls			:		
Input Params		:
Output Params		:	std::vector<int> *indicesToRemove
Return			:
Remarks			:

*****************************************************************************************/
inline __host__ __device__  float3 compute3DCentroid(
	float3 *gpuPoints,
	idxType *gpuNeighborIndices,
	idxType &totCount, 
	idxType &loc, 
	idxType &x
	)
	{
	float3 centroid = make_float3 (0.0f, 0.0f, 0.0f); 
  	// Compute Centroid:
 	centroid = gpuPoints[x];

	for(idxType y = 0; y < totCount; y++)
		{
		idxType index = gpuNeighborIndices[y + loc];
		centroid += gpuPoints[index];
		}

     if(totCount > 0)
     	 centroid = centroid/(float)(totCount + 1);
     	 
	return centroid;
	}

/*****************************************************************************************

Function Name		:	PointClassificaton::outlierRemoval	
Purpose	of Function	:	Outlier Removal
Author/date		:	Beena
Modified by/date	:
Calls			:		
Input Params		:
Output Params		:	std::vector<int> *indicesToRemove
Return			:
Remarks			:

*****************************************************************************************/
inline __host__ __device__ CovarianceMatrix computeCovarianceMatrix(
	float3 *gpuPoints,
	idxType* gpuNeighborIndices,
	idxType &totCount, 
	idxType &loc, 
	float3& centroid,
	idxType &x
	)
	{
  	// Compute Centroid:
	CovarianceMatrix cov ;
	float3 pt = make_float3 (0.0f, 0.0f, 0.0f);
	
	cov.data[0] = make_float3 (0.0f, 0.0f, 0.0f);
    cov.data[1] = make_float3 (0.0f, 0.0f, 0.0f);
    cov.data[2] = make_float3 (0.0f, 0.0f, 0.0f);
 
 	pt.x = (gpuPoints[x].x - centroid.x);
	pt.y = (gpuPoints[x].y - centroid.y);
	pt.z = (gpuPoints[x].z - centroid.z);
	

	for(idxType y = 0; y < totCount; y++)
		{
		
		idxType index = gpuNeighborIndices[y + loc];
		
		pt.x = (gpuPoints[index].x - centroid.x);
		pt.y = (gpuPoints[index].y - centroid.y);
		pt.z = gpuPoints[index].z - centroid.z;
		
		cov.data[1].y += (pt.y * pt.y);
		cov.data[1].z += (pt.y * pt.z);
		cov.data[2].z += (pt.z * pt.z);
		
		cov.data[0].x += (pt.x * pt.x) ;
      	cov.data[0].y += (pt.y * pt.x) ;
      	cov.data[0].z += (pt.z * pt.x) ;

		
		}
		cov.data[1].x = cov.data[0].y ;
      	cov.data[2].x = cov.data[0].z ;
      	cov.data[2].y = cov.data[1].z;
      	
     /* 	if(totCount > 0)
      	{
		cov.data[0] = cov.data[0]/(float)totCount;
      	cov.data[1] = cov.data[1]/(float)totCount;
      	cov.data[2] = cov.data[2]/(float)totCount ;
      	}
      	*/
      return cov;
	}
    

    /*****************************************************************************************

Function Name		:	PointClassificaton::outlierRemoval	
Purpose	of Function	:	Outlier Removal
Author/date		:	Beena
Modified by/date	:
Calls			:		
Input Params		:
Output Params		:	std::vector<int> *indicesToRemove
Return			:
Remarks			:

*****************************************************************************************/
/*inline void displayCudaError(
	cudaError_t error)
	{
	printf("CUDA error: %s\n", cudaGetErrorString(error));
	if(error != cudaSuccess)
  		{
    		// print the CUDA error message and exit
    		printf("CUDA error: %s\n", cudaGetErrorString(error));
   		exit(-1);
  		}
	}*/

/*****************************************************************************************

Function Name		:	PointClassificaton::outlierRemoval	
Purpose	of Function	:	Outlier Removal
Author/date		:	Beena
Modified by/date	:
Calls			:		
Input Params		:
Output Params		:	std::vector<int> *indicesToRemove
Return			:
Remarks			:

*****************************************************************************************/
 /*inline __host__ __device__ void compute3DCentroid(
	float3 *gpuPoints,
	idxType* gpuNeighborIndices,
	idxType *totCount, 
	idxType *loc,
	float3 *centroid)
	{
  	// Compute Centroid:
  	(*centroid).x = (*centroid).y = (*centroid).z = 0;

	for(idxType y = 0; y < (*totCount); y++)
		{
		idxType index = gpuNeighborIndices[y + (*loc)];
		(*centroid) += gpuPoints[index];
		}

	(*centroid) = (*centroid)/(*totCount);
	return ;
	}

/*****************************************************************************************

Function Name		:	PointClassificaton::outlierRemoval	
Purpose	of Function	:	Outlier Removal
Author/date		:	Beena
Modified by/date	:
Calls			:		
Input Params		:
Output Params		:	std::vector<int> *indicesToRemove
Return			:
Remarks			:

*****************************************************************************************/
 /*inline __host__ __device__ void computeCovarianceMatrix(
	float3 *gpuPoints,
	idxType* gpuNeighborIndices,
	idxType *totCount, 
	idxType *loc, 
	float3& centroid,
	CovarianceMatrix& cov)
	{
  	// Compute Centroid:

	cov.data[0] = make_float3 (0.0f, 0.0f, 0.0f);
    cov.data[1] = make_float3 (0.0f, 0.0f, 0.0f);
    cov.data[2] = make_float3 (0.0f, 0.0f, 0.0f);
	float3 pt;

	for(idxType y = 0; y < (*totCount); y++)
		{
		idxType index = gpuNeighborIndices[y + (*loc)];
		pt.x = gpuPoints[index].x - centroid.x;
		pt.y = gpuPoints[index].y - centroid.y;
		pt.z = gpuPoints[index].z - centroid.z;

		cov.data[1].y += pt.y * pt.y;
		cov.data[1].z += pt.y * pt.z;

		cov.data[2].y += pt.z * pt.z;

      	cov.data[0].x += pt.x * pt.x ;
      	cov.data[0].y += pt.y * pt.x ;
      	cov.data[0].z += pt.z * pt.x ;

		}

		cov.data[1].x = cov.data[0].y ;
      	cov.data[2].x = cov.data[0].z ;
      	cov.data[2].y = cov.data[1].z;
	}*/
