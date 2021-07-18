// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


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

/*******************************************************************************/

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
        // Voxel grid reduction:
        typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
        
        typename pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(filterRes, filterRes, filterRes);
        vg.filter(*cloudFiltered);

        // Region of interest:
        typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
        
        typename pcl::CropBox<PointT> RegOfInterest(true);
        RegOfInterest.setMin(minPoint);
        RegOfInterest.setMax(maxPoint);
        RegOfInterest.setInputCloud(cloudFiltered);
        RegOfInterest.filter(*cloudRegion);

        // Remove roof points of subject-car
        std::vector<int> indices;

        typename pcl::CropBox<PointT> roof;
        roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
        roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
        roof.setInputCloud(cloudRegion);
        roof.filter(indices);

        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        for (int point : indices)
            inliers->indices.push_back(point);
        
        typename pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloudRegion);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloudRegion);
        

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

/*******************************************************************************/

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // Creating two point cloud pointers for road and obstacles
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());
    
    // Creating the Extract objectcloud_filtered
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliners from the input cloud into planeCloud
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planeCloud);

    // Extract non plane points (obstable points) into obstCloud
    extract.setNegative (true);
    extract.filter(*obstCloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

/*********** My RansacPlane function to replace pcl's inbuilt function ***********/

// Struct datatype for a 3d point with float variables
struct Point3d
{
	float x,y,z;
};

// This function return coefficients of a plane in 3D space
inline std::vector<float> planeCoEffs(Point3d p1, Point3d p2, Point3d p3)
{
		/* Equation of a plane: Ax + By + Cz + D = 0, define A,B,C,D */
		
		// Define two vectors on the plane; v1 = p2-p1, v2 = p3-p1
		std::vector<float> v1 = {p2.x-p1.x, p2.y-p1.y, p2.z-p1.z};
		std::vector<float> v2 = {p3.x-p1.x, p3.y-p1.y, p3.z-p1.z};

		// Normal vector to the plane n1 = (v1 x v2) {cross product}
		std::vector<float> n1 =   {(p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y),
								   (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z),
								   (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x)};
		// Plane equation coefficients
		float A = n1.at(0);	
		float B = n1.at(1);
		float C = n1.at(2);
		float D = -(A*p1.x + B*p1.y + C*p1.z);
		std::vector<float> coEffs = {A,B,C,D};
		return coEffs;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	// Algorithm:

	// For max iterations 
	// Randomly sample 3 points and fit plane
	// Measure distance between every point and fitted plane
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers
	
	while( maxIterations-- )	// For max iterations
	{
		// Randomly sample three points to make a line
		std::unordered_set<int> inliers;	// Initially stores three points for the line, later stores all the inliers
		                                	// An unordered set makes sure the points are unique
		while(inliers.size()<3)
			inliers.insert(rand()%(cloud->points.size()));	// Inserting index of a point in cloud into inliers
        
        Point3d p1, p2, p3;				// Three points

		auto itr = inliers.begin();		// Returns a pointer to first eliment in inliers
		p1.x = cloud->points[*itr].x;		// Dereferencing the pointer to obtain the value of pointed element
		p1.y = cloud->points[*itr].y;
		p1.z = cloud->points[*itr].z;
		itr++;
		p2.x = cloud->points[*itr].x;		// Dereferencing the pointer to obtain the value of pointed element
		p2.y = cloud->points[*itr].y;
		p2.z = cloud->points[*itr].z;
		itr++;
		p3.x = cloud->points[*itr].x;		// Dereferencing the pointer to obtain the value of pointed element
		p3.y = cloud->points[*itr].y;
		p3.z = cloud->points[*itr].z;

		// Co-efficients of equation of the plane
        std::vector<float> coEffs = planeCoEffs(p1,p2,p3);
		float A = coEffs.at(0);
		float B = coEffs.at(1);
		float C = coEffs.at(2);
		float D = coEffs.at(3);
		
		for(int index = 0; index < cloud->points.size(); index++)
		{
			// If one of the three pointes is selected, the loop continues to next iteration
            if (inliers.count(index)>0) 	
				continue;
			
			PointT point = cloud->points[index]; 	
			float x = point.x;
			float y = point.y;
			float z = point.z;

			// Calculating the distance of (x,y,z) from the plane
			float dist = fabs(A*x + B*y + C*z + D)/sqrt(A*A + B*B + C*C);

            // If distance is less than tollerence, it is added to inliers
			if(dist<distanceTol)
				inliers.insert(index);		
			
		}
        // If the new model has more inliers, it replaces previous inliers in inliersResult
		// inliersResult stores inliers of best modal
		if(inliers.size()> inliersResult.size())
			inliersResult = inliers;	

	}
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RansacPlane took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;
}

/*******************************************************************************/

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
   /* Implementing pcl::sac_Ransac function*/ 
   
   /* 
  // TODO:: Fill in this function to find inliers for the cloud.
    // Creat Coefficients, inliers and segmentation objects 
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients}; // Coefficient object, it defines the segmented plane. useful to render the resultant plane
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices}; // Inliers object, it stores the indices of all the points that belong to the plane.
    pcl::SACSegmentation<PointT> seg; // Segmentation object, this segments the  points of cloud into "plane points" and "non plane points".

    // Setup the methods of seg object.
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE); // Setting to segment a plane model from the cloud.
    seg.setMethodType (pcl::SAC_RANSAC);    // RANSAC (Random sample consensus) is an iterative method. 
    seg.setMaxIterations (maxIterations);    
    seg.setDistanceThreshold (distanceThreshold);    // Farthest a point can be from the model to be considered as an inlier.  

    // Segment the largest planar component from the given cloud.
    seg.setInputCloud (cloud);  // Feeding the input cloud to seg. 
    seg.segment (*inliers, *coefficients);  // Executing the segmentation method.
    */
  
   /* Implimenting custom RansacPlane function */
    std::unordered_set<int> inliersResult = RansacPlane(cloud, maxIterations, distanceThreshold);
	
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices}; // Inliers object, it stores the indices of all the points that belong to the plane.
    
    for( auto ptr = inliersResult.begin(); ptr != inliersResult.end(); ++ptr)
	{
		inliers->indices.push_back(*ptr);
	}

    if(inliers->indices.size () == 0) // Checking if no inliers are found.
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;

    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

/*********** Implementing clustering using my own euclideanCluster function ***********/

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
        // Euclidean clustering using PCL function
        /*
        typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(clusterTolerance);
        ec.setMinClusterSize(minSize);
        ec.setMaxClusterSize(maxSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(clusterIndices);
        */

        /* Euclidean clustering using my own function */
        // This vector stores all the points in the input cloud as float vectors
        std::vector<std::vector<float>> vectorPoints;
        KdTree* tree = new KdTree;

        // Push all the points in the cloud into the vectorPoints
        for (int i = 0; i < cloud->points.size(); i++)
        {
            std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            vectorPoints.push_back(point);
            tree->insert(point, i);    // Custom kdTree takes in float vectors as points
        }

        // Vector that stores clusters as vectors of indices
        std::vector<std::vector<int>> clusterIndices;

        clusterIndices = euclideanCluster(vectorPoints, tree, clusterTolerance);

    for( auto each_cluster : clusterIndices)
    {
        // Generate point clouds for every cluster in "cluserIndices"
        // Push the point clouds into "clusters" vector

        // Reject the clusters that are not within the minSize and maxSize
        if(each_cluster.size() < minSize || each_cluster.size() > maxSize) continue;
        
        // Stores the current cluster
        typename pcl::PointCloud<PointT>::Ptr clusterCloud (new pcl::PointCloud<PointT>);
        
        // Fetch the points of all the indices in the "each_cluster" and add them to "clusterCloud"
        for(auto idx : each_cluster)
          clusterCloud->points.push_back(cloud->points[idx]);
        
        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;

        // Push the current cluster into the vector of clusters
        clusters.push_back(clusterCloud);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

/*******************************************************************************/

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