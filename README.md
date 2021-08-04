# Lidar Obstacle Detection
The goal of the project is to consistantly detect obstacles in a real LiDAR Point Cloud stream by filtering, segemeting, clustering the points of objects. 

![ObstacleDetectionFPS](https://user-images.githubusercontent.com/48198017/128245366-c20b806b-392d-42f2-9f7c-ce4bff5607dd.gif)

## Project Pipeline
1. Load the PCL Point Cloud
2. Filter the Point Cloud 
3. Segment the points into obstacle cloud and ground plane cloud
4. Cluster the points that form the objects
5. Apply bounding boxes on the clustered objects

Following are the detailed explanations of these steps

## Filtering
The dense point cloud is filtered to decrease the computation time. following are the methods implemented:
1. The point cloud is downsampled by applying voxel grid filter of grid size = 0.18m, that leaves single point per cell. 
2. The points beyond the required range are cropped to eliminate redundant data. 
3. The ego car roof points are also removed

