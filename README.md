# Lidar Object Detection
The goal of the project is to consistantly detect obstacles in a real lidar point cloud stream. This is achieved by filtering, segemeting and clustering the point cloud. Lidar is an active sensor that provides real-time spatial perception of the environment.

![ObstacleDetectionFPS](https://user-images.githubusercontent.com/48198017/128245366-c20b806b-392d-42f2-9f7c-ce4bff5607dd.gif)

## Lidar
Lidar is an active sensor that emits laser beams and receives them upon reflection. The distance of the obstructing surface is computed using the Time of Flight and speed of the corresponding beam. Each such beam, upon 360 degrees rotation of the Lidar scanner, provides the distances of the obstacles present in the contemporary environment. In addition, the intensities of the reflected beams are recorded. This process is called environment perception, which is the first step in the motion planning of an autonomous robot. 

The point cloud used in this project is obtained using a Velodyne VLP-64 Lidar, where 64 stands for the number of laser emitters in the emitter array. One scan of this sensor generates 256,000 points. 

## Workspace

The workspace provided in the SFND classroom comes preinstallated with everything that you need to finish the exercises and projects. Versions used by Udacity for this ND are as follows:

* Ubuntu 16.04
* PCL - v1.7.2
* C++ v11
* gcc v5.5

**Note** The [[CMakeLists.txt](https://github.com/udacity/SFND_Lidar_Obstacle_Detection/blob/master/CMakeLists.txt)] file provided in this repo can be used locally if you have the same package versions as mentioned above. If you want to run this project locally (outside the Udacity workspace), please follow the steps under the **Local Installation** section.


## Local Installation

### Ubuntu 

1. Clone this github repo:

   ```sh
   cd ~
   git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
   ```

2.  Edit [CMakeLists.txt](https://github.com/udacity/SFND_Lidar_Obstacle_Detection/blob/master/CMakeLists.txt) as follows:

   ```cmake
   cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
   
   add_definitions(-std=c++14)
   
   set(CXX_FLAGS "-Wall")
   set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
   
   project(playback)
   
   find_package(PCL 1.11 REQUIRED)
   
   include_directories(${PCL_INCLUDE_DIRS})
   link_directories(${PCL_LIBRARY_DIRS})
   add_definitions(${PCL_DEFINITIONS})
   list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
   
   
   add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
   target_link_libraries (environment ${PCL_LIBRARIES})
   ```

3. Execute the following commands in a terminal

   ```shell
   sudo apt install libpcl-dev
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```

   This should install the latest version of PCL. You should be able to do all the classroom exercises and project with this setup.

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

6. Clone this github repo

   ```shell
   cd ~
   git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
   ```

7. Edit the CMakeLists.txt file as shown in Step 2 of Ubuntu installation instructions above.

8. Execute the following commands in a terminal

   ```shell
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```

### WINDOWS

#### Install via cvpkg

1. Follow the steps [here](https://pointclouds.org/downloads/) to install PCL.

2. Clone this github repo

   ```shell
   cd ~
   git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
   ```

3. Edit the CMakeLists.txt file as shown in Step 2 of Ubuntu installation instructions above.

4. Execute the following commands in Powershell or Terminal

   ```shell
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
### Point Cloud
The projection of lidar reflections onto a vector space results in a point cloud that looks as follows:<br/>

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
2. The points beyond the required range are cropped to eliminate redundant data. This is performed using the PCL CropBox object.
3. The roof points of the ego car are also removed as they are not considered an obstacle. This is also performed using the PCL  CropBox object.<br/> 

![downsampled small](https://user-images.githubusercontent.com/48198017/128407450-1f9be9bb-9ee7-40aa-a32d-5d728781336b.png)


 
## Segmentation
The point cloud is segmented to omit the ground plane for obstacle detection. The segmentation is performed using the following Ransac algorithm:
1. Randomly sample three points from the cloud and Fit a plane using these three points.
2. Calculate the distance of each point in the cloud from the plane. 
3. If the distance is smaller than the tolerance, add the Index of the point to the inliers set.
4. If the current set contains more inliers than the previous one, the current one is stored.  
5. After *max_iterations* the set with the maximum number of inliers is stored as the ground plane cloud.
6. The outliers are stored as the object cloud.<br/>

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
 
 ![clustering smaller](https://user-images.githubusercontent.com/48198017/128407508-b5abde38-b7c6-4954-a70a-9a1b634c3118.png)

 
## Bounding Boxes
 A bounding box is applied to every cluster. It is a rectangular cuboid generated using the min and max points of a cluster.
 
## Final Notes
Thus the obstacles in the point cloud stream are detected. The code is written in C++ using PCL and STL libraries. The template functions make the code functional to multiple point data types like (X, Y, Z), (X, Y, Z, I) and so forth.
 
 


