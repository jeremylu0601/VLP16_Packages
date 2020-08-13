# VLP16_SLAM

### Install the following packages

    sudo apt-get install pcl-tools libomp-dev libpcl-dev
    
[pcl_ros](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)

[Velodyne driver under ROS](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16) 

* Step 1.2 can be ignored
* i.e. ignore the following command
    
      sudo ifconfig eth0 192.168.3.100
    
      sudo route add 192.168.XX.YY eth0
      
 
 [LOAM](https://github.com/laboshinl/loam_velodyne) 
 
 * Change some lines in loam_velodyne/src/lib/MultiScanRegistration.cpp
    1. change the lines 182-184 to:
        
            point.x = laserCloudIn[i].x;

            point.y = laserCloudIn[i].y;
        
            point.z = laserCloudIn[i].z;
       
    2. change the lines 199 to:
    
            float angle = std::atan(point.z / std::sqrt(point.y * point.y + point.x * point.x));
 
    3. change the lines 206 to:
    
            float ori = -std::atan2(point.y, point.x);

 [ndt_omp](https://github.com/koide3/ndt_omp)
 
 
 [hdl_localization](https://github.com/koide3/hdl_localization)
 
* Before catkin_make, please
    
      source <path_to_ndt_omp_workspace>/devel/setup.dash




