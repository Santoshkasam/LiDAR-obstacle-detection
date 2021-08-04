# Lidar Obstacle Detection
The goal of the project is to consistantly detect obstacles in a real LiDAR Point Cloud stream by filtering, segemeting, clustering the points of objects. 

![ObstacleDetectionFPS](https://user-images.githubusercontent.com/48198017/128245366-c20b806b-392d-42f2-9f7c-ce4bff5607dd.gif)

## Project Pipeline
1. Load the PCL Point Cloud
2. Filter the Point Cloud 
3. Segment the points into obstacle cloud and ground plane cloud
