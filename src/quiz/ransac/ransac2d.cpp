/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	while( maxIterations-- )	// For max iterations
	{
		// Randomly sample two points to make a line
		std::unordered_set<int> inliers;	// Initially stores two points for the line, later stores all the inliers
		                                	// An unordered set makes sure the points are unique
		while(inliers.size()<2)
			inliers.insert(rand()%(cloud->points.size()));	// Inserting index of a point in cloud into inliers

		float x1,y1,x2,y2;				// Coordinates of the two points
		
		auto itr = inliers.begin();		// Returns a pointer to first eliment in inliers
		x1 = cloud->points[*itr].x;		// Dereferencing the pointer to obtain the value of pointed element
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;		
		y2 = cloud->points[*itr].y;

		// Equation of a line: Ax + By + C = 0, where,
		float A = (y1-y2);
		float B = (x2-x1);
		float C = (x1*y2-x2*y1);

		for(int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index)>0) 	// If one of the two pointes is selected, the loop continues to next iteration
				continue;
			
			pcl::PointXYZ point = cloud->points[index]; 	
			float x3 = point.x;
			float y3 = point.y;

			// Calculating the distance of (x3,y3) from the line
			float d = fabs(A*x3 + B*y3 + C)/sqrt(A*A + B*B);

			if(d<distanceTol)
				inliers.insert(index);		// If distance is less than tollerence, it is added to inliers
			
		}

		if(inliers.size()> inliersResult.size())
			inliersResult = inliers;	// If the new model has more inliers, it replaces previous inliers in inliersResult
										// inliersResult stores inliers of best modal

	}
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;
}

struct Point3d
{
	float x,y,z;
};

std::vector<float> planeCoEffs(Point3d p1, Point3d p2, Point3d p3)
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

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	// TODO: Fill in this function

	// For max iterations 
	// Randomly sample subset and fit plane
	// Measure distance between every point and fitted plane
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers
	
	for(int i = 0; i < maxIterations; i++ )	// For max iterations
	{
		// Randomly sample three points to make a line
		std::unordered_set<int> inliers;	// Initially stores three points for the line, later stores all the inliers
		                                	// An unordered set makes sure the points are unique
		while(inliers.size()<3)
			inliers.insert(rand()%(cloud->points.size()));	// Inserting index of a point in cloud into inliers

		Point3d p1, p2, p3;					// Three points

		auto itr = inliers.begin();			// Returns a pointer to first eliment in inliers
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

		std::vector<float> coEffs = planeCoEffs(p1, p2, p3);
		float A = coEffs.at(0);
		float B = coEffs.at(1);
		float C = coEffs.at(2);
		float D = coEffs.at(3);
		
		for(int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index)>0) 	// If one of the three pointes is selected, the loop continues to next iteration
				continue;
			
			pcl::PointXYZ point = cloud->points[index]; 	
			float x = point.x;
			float y = point.y;
			float z = point.z;

			// Calculating the distance of (x,y,z) from the line
			float d = fabs(A*x + B*y + C*z + D)/sqrt(A*A + B*B + C*C);

			if(d<distanceTol)
				inliers.insert(index);		// If distance is less than tollerence, it is added to inliers
			
		}

		if(inliers.size()> inliersResult.size())
			inliersResult = inliers;	// If the new model has more inliers, it replaces previous inliers in inliersResult
										// inliersResult stores inliers of best modal

	}
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RansacPlane took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;
}



int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 8, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
