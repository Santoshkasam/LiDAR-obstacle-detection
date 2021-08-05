# Lidar Obstacle Detection
**The goal of the project is to consistantly detect obstacles in a real LiDAR Point Cloud stream. This is achieved by filtering, segemeting and clustering the point cloud.**

![ObstacleDetectionFPS](https://user-images.githubusercontent.com/48198017/128245366-c20b806b-392d-42f2-9f7c-ce4bff5607dd.gif)

## Lidar
Lidar is an active sensor that emits laser beams and receives them upon reflection. The distance of the obstructing surface is computed using the Time of Flight and speed of the corresponding beam. Each such beam, upon 360 degrees rotation of the Lidar scanner, provides the distances of the obstacles present in the contemporary environment. In addition, the intensities of the reflected beams are recorded. This process is called environment perception, which is the first step in the motion planning of an autonomous robot. 

The point cloud used in this project is obtained using a Velodyne VLP-64 Lidar, where 64 stands for the number of laser emitters in the emitter array. One scan of this sensor generates 256,000 points. 

### Point Cloud
The projection of lidar reflections onto a vector space, after a 360degree scan, results in a point cloud that looks as follows:
<insert point cloud>



## Project Pipeline
1. Load the PCL Point Cloud.
2. Filter the Point Cloud to reduce the density.
3. Segment the points into obstacle cloud and ground plane cloud.
4. Cluster the points that form the objects.
5. Apply bounding boxes on the clustered objects.

Following are the detailed explanations of these steps

## Filtering
The dense point cloud is filtered to decrease the computation time. following are the methods implemented:
1. The point cloud is downsampled by applying voxel grid filter with a grid size of 0.18m, that leaves single point per cell. This is performed using PCL VoxelGrid object. 
2. The points beyond the required range are cropped to eliminate redundant data. This is performed using PCL CropBox class object.
3. The ego car roof points are also removed as the ego car is not considered as an obstacle. This is also performed using PCL  CropBox object.
 
## Segmentation
The point cloud is segmented in order to omit ground plane for object detection. It is performed using the following Ransac algorithm:
1. Randomly sample three points from the cloud and Fit a plane using these three points.
2. Calculate the distance of each point in the cloud from the plane. 
3. If the distance is smaller than tolerance, add the index of the point to the inliers set.
4. If the current set contains more inliers than the previous one, the current one is stored.  
5. After *max_iterations* the set with maximum number of inliers is stored as the ground plane cloud.
6. The outliers are stored as the object cloud.

## Clustering 
 
Clustering is performed to identify groups of points that represent unique objects. This objective is achieved in two stages:
**Stage I**: Populate Kd-Tree
Kd-Tree is a binary tree data structure in which each node is a K-dimnesional point. The points in the data cloud are arranged in the structure of a Kd-tree

 
## Bounding Boxes
 


