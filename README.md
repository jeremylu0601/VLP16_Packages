# VLP16 SLAM

### Install the following packages

    sudo apt-get install pcl-tools libomp-dev libpcl-dev
    
[pcl_ros](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)

[Velodyne driver under ROS](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16) 

* Step 1.2 can be ignored
* i.e. ignore the following command
    
      sudo ifconfig eth0 192.168.3.100
    
      sudo route add 192.168.XX.YY eth0
      
 
 [LOAM](https://github.com/laboshinl/loam_velodyne) 
 
 * Change some lines in **loam_velodyne/src/lib/MultiScanRegistration.cpp**
    1. change the lines 182-184 to:
        
            point.x = laserCloudIn[i].x;

            point.y = laserCloudIn[i].y;
        
            point.z = laserCloudIn[i].z;
       
    2. change the lines 199 to:
    
            float angle = std::atan(point.z / std::sqrt(point.y * point.y + point.x * point.x));
 
    3. change the lines 206 to:
    
            float ori = -std::atan2(point.y, point.x);
            
          
### Usage

Open a terminal

    source /opt/ros/melodic/setup.bash
    roslaunch velodyne_pointcloud VLP16_points.launch
    
Open the 2nd ternminal

    source /opt/ros/melodic/setup.bash
    source <path to LOAM workspace>/devel/setup.bash
    roslaunch loam_velodyne loam_velodyne.launch 
  
Open the 3rd ternminal

    source /opt/ros/melodic/setup.bash
    rosbag record -o out /laser_cloud_surround
    # map is published on /laser_cloud_surround

After enough map data is saved,

    source /opt/ros/melodic/setup.bash
    rosrun pcl_ros bag_to_pcd <the .bag file we just saved> /laser_cloud_surround pcd
    # pcd : the folder where the .pcd file are saved 
            feel free to change

Visualize the .pcd file

    cd pcd
    pcl_viewer xxxxxx.pcd 
    # the last .pcd file is the most recent map from LOAM
    # press 5 to change the color
    
# VLP16 Localization

### Install the following packages

 [ndt_omp](https://github.com/koide3/ndt_omp)
 
 
 [hdl_localization](https://github.com/koide3/hdl_localization)
 
* Before catkin_make, please
    
      source <path to ndt_omp workspace>/devel/setup.dash




