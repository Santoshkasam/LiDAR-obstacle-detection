// Fine tuned the implementation of Aaron Brown, Udacity.
// Assignment performed by Santosh Kasam.

#ifndef CLUSTER_H
#define CLUSTER_H

// This is a recursive function used to navigate along the tree with a reference
// point from the vector "points" and find nearby points to form clusters
inline void euclideanClusterHelper(int idx, const std::vector<std::vector<float>> points,std::vector<int> &cluster, std::vector<bool> &processed, KdTree* tree, float distanceTol)
{
	processed[idx] = true;
	cluster.push_back(idx);

	std::vector<int> nearbyPoints = tree->search(points[idx], distanceTol);

	for(int id : nearbyPoints)
	{
		if(!processed[id]) 
			euclideanClusterHelper(id, points, cluster,  processed, tree, distanceTol);
	} 


}

// euclideanCluster: for every point in "points" vector, the function searches 
// for nearby points withing the "distanceTol" threshold and forms cluster
// Note: a point once included in a cluster, is not processed again.
inline std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	int i = 0;
	for (int i = 0; i<points.size(); i++)
	{
		// Check if the point i is already processed
		if(processed[i])
		{
			continue;
		}

		// Creat cluster, fill with nearby points, add to list of clusters
		std::vector<int> cluster;
		euclideanClusterHelper(i, points, cluster,  processed, tree, distanceTol);		
		clusters.push_back(cluster);
	}
	
	return clusters;
}

#endif