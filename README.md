# Lidar Obstacle Detection
The goal of the project is to consistantly detect obstacles in a real lidar point cloud stream. This is achieved by filtering, segemeting and clustering the point cloud. Lidar is an active sensor the provides real-time spatial perception of the environment.

![ObstacleDetectionFPS](https://user-images.githubusercontent.com/48198017/128245366-c20b806b-392d-42f2-9f7c-ce4bff5607dd.gif)

## Lidar
Lidar is an active sensor that emits laser beams and receives them upon reflection. The distance of the obstructing surface is computed using the Time of Flight and speed of the corresponding beam. Each such beam, upon 360 degrees rotation of the Lidar scanner, provides the distances of the obstacles present in the contemporary environment. In addition, the intensities of the reflected beams are recorded. This process is called environment perception, which is the first step in the motion planning of an autonomous robot. 

The point cloud used in this project is obtained using a Velodyne VLP-64 Lidar, where 64 stands for the number of laser emitters in the emitter array. One scan of this sensor generates 256,000 points. 

### Point Cloud
The projection of lidar reflections onto a vector space, after a 360degree scan, results in a point cloud that looks as follows:<br/>
<br/>
![raw point cloud](https://user-images.githubusercontent.com/48198017/128407760-0fa6502d-bdde-41dc-87b2-82eaa8db38f1.png)







## Project Pipeline
1. Load the PCL Point Cloud.
2. Filter the Point Cloud to reduce the density.
3. Segment the points into obstacle cloud and ground plane cloud.
4. Cluster the points that form the obstacles.
5. Apply bounding boxes on the clustered obstacles.

Following are the detailed explanations of these steps

## Filtering
The dense point cloud is filtered to decrease the computation time. following are the methods implemented:
1. The point cloud is downsampled by applying a voxel grid filter with a grid size of 0.18m. It leaves a single point per cell. This process is performed using the PCL VoxelGrid object. 
2. The points beyond the required range are cropped to eliminate redundant data. This is performed using the PCL CropBox class object.
3. The ego car roof points are also removed as the ego car is not considered an obstacle. This is also performed using the PCL  CropBox object.<br/> 
<br/>
![downsampled small](https://user-images.githubusercontent.com/48198017/128407450-1f9be9bb-9ee7-40aa-a32d-5d728781336b.png)


 
## Segmentation
The point cloud is segmented to omit the ground plane for obstacle detection. The segmentation is performed using the following Ransac algorithm:
1. Randomly sample three points from the cloud and Fit a plane using these three points.
2. Calculate the distance of each point in the cloud from the plane. 
3. If the distance is smaller than the tolerance, add the Index of the point to the inliers set.
4. If the current set contains more inliers than the previous one, the current one is stored.  
5. After *max_iterations* the set with the maximum number of inliers is stored as the ground plane cloud.
6. The outliers are stored as the object cloud.<br/>
<br/>
![segmentation_small](https://user-images.githubusercontent.com/48198017/128407484-5f7ba820-b42f-40c0-9c5d-adc407a7ff6d.png)



## Clustering 
 
Clustering is performed to identify groups of points that represent unique obstacles. This objective is achieved by the Euclidien clustering algorithm using a Kd-Tree data structure. The algorithm is as follows:
1. Populate a Kd-Tree with the point cloud data.
 2. Select a point from the point cloud. 
 3. Find the neighbours of the point within the distance tolerance, that are un-clustered.
 4. Find the neighbours of the points identified in the previous step.
 5. Continue the process until all the nearest neighbours are identified.
 6. Store these points as a cluster
 7. Select another un-clustered point and repeat the process from step 3, until no points are left in the cloud.
 8. At the end, store only those clusters whose size is between the range of min_size and max_size and discard the rest.<br/> 
 <br/>
 ![clustering smaller](https://user-images.githubusercontent.com/48198017/128407508-b5abde38-b7c6-4954-a70a-9a1b634c3118.png)

 
## Bounding Boxes
 A bounding box is applied to every cluster. It is a rectangular cuboid generated using the min and max points of a cluster.
 
 


