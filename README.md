# Lidar Obstacle Detection
The goal of the project is to consistantly detect obstacles in a real LiDAR Point Cloud stream by filtering, segemeting and clustering the points of objects.

![ObstacleDetectionFPS](https://user-images.githubusercontent.com/48198017/128245366-c20b806b-392d-42f2-9f7c-ce4bff5607dd.gif)

## Lidar
Lidar is an active sensor that emits laser beams and receives them upon reflection. The distance of the obstructing surface is computed using the Time of Flight and speed of the corresponding beam. Each such beam, upon 360 degrees rotation of the Lidar scanner, provides the distances of the obstacles present in the contemporary environment. In addition, the intensities of the reflected beams are recorded.  The point cloud used in this project is obtained from a Velodyne VLP-64 Lidar. 64 stands for the number of laser emitters in the emitter array. One scan of this sensor generates 256,000 points. This process is called environment perception, which is the first step in the motion planning of an autonomous robot. 

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
1. The point cloud is downsampled by applying voxel grid filter with a grid size of 0.18m, that leaves single point per cell. 
2. The points beyond the required range are cropped to eliminate redundant data. 
3. The ego car roof points are also removed as the ego car is not considered as an obstacle. 
 
## Segmentation
The point cloud is segmented in order to omit ground plane for object detection. The identification and segmentation of the point cloud is performed as follows:
 1. 
 2.
 3.
 
 
## Clustering 
 
## Bounding Boxes
 


