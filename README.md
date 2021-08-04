# Lidar Obstacle Detection
The goal of the project is to consistantly detect obstacles in a real LiDAR Point Cloud stream by filtering, segemeting and clustering the points of objects. 

![ObstacleDetectionFPS](https://user-images.githubusercontent.com/48198017/128245366-c20b806b-392d-42f2-9f7c-ce4bff5607dd.gif)

## Lidar
Lidar is an acitive sensor that emmits laser beams and resieves them upon reflection. The distance of the obstrcuting surface is calculated using the Time of Flight of the corrosponding beam. Each such beam, upon 360 degrees rotation, provides the distances of the obstacles present in the environment in the moment of scanning. In addition, the intensity of the reflected beam is also recorded. This is called environment perception, which is the first step of an autonomous robot in planning the motion.

### Point Cloud
The projections of lidar reflections onto a vector space results in a point cloud that looks as follows:
<insert point cloud>
 A .pcd format point cloud is used. it contains a list of (x,y,x,I) cartesian co-ordinates and intensity values. it is a snapshot of the environmet after a single scan. 
 The point cloud used in this project is obtained from a Velodyne VLP-64 Lidar. 64 stands for the number of laser emitters in the emitter array. a single scan of this sensor generates 256,000 point. 

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

