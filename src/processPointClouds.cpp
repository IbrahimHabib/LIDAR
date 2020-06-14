// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr PlaneCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr ObstCloud (new pcl::PointCloud<PointT> ());

    for (int index : inliers->indices)
    {
        PlaneCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
     extract.filter (*ObstCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(ObstCloud, PlaneCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	typename pcl::PointCloud<PointT>::Ptr  inliers(new pcl::PointCloud<PointT>());  // The plane points
	typename pcl::PointCloud<PointT>::Ptr outliers(new pcl::PointCloud<PointT>());  // The not-plane points
    std::unordered_set<int> inliersResult;
    inliersResult = RansacPlane(cloud, maxIterations, distanceThreshold);
    for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			inliers->points.push_back(point);
		else
			outliers->points.push_back(point);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
     return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> (outliers, inliers);

}

// implementation of RANSAC ona plan
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
 std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations
	while (maxIterations --) {
		// Randomly sample subset and fit plane
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) {
			inliers.insert(rand() % cloud->points.size()); 
		}

		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float a, b, c, d, v3_i, v3_j, v3_k;
		// Let v1 be a vector from point 1 to point 2 in the plane
		// Let v2 be a vector from point 1 to point 3 in the plane
		// Let v3 equal the cross product v1 x v2
		v3_i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		v3_j = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		v3_k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);

		// This iteration's plane is modeled by the equation Ax + By + Cz + D = 0
		a = v3_i;
		b = v3_j;
		c = v3_k;
		d = -(v3_i*x1 + v3_j*y1 + v3_k*z1);

		for (int index = 0; index < cloud->points.size(); index++) {
			// Skip if the considered point is already an inlier.
			if (inliers.count(index) > 0) continue;
			
			pcl::PointXYZ point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			float distance = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c*c);

			if (distance <= distanceTol) {
				inliers.insert(index);
			}
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}

	return inliersResult;

}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
  	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  	tree->setInputCloud (cloud);

  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  	ec.setClusterTolerance (clusterTolerance); // 2cm
  	ec.setMinClusterSize (minSize);
  	ec.setMaxClusterSize (maxSize);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud);
  	ec.extract (cluster_indices);

	for (auto getindices : cluster_indices)
	{
		typename pcl::PointCloud<PointT>::Ptr cloudcluster (new pcl::PointCloud<PointT>);	
		for (auto index : getindices.indices)
			cloudcluster->points.push_back (cloud->points[index]);

			cloudcluster->width = cloudcluster->points.size();
			cloudcluster->height = 1;
			cloudcluster->is_dense = true;

			clusters.push_back(cloudcluster);

		
	
	
	}
	

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;


}