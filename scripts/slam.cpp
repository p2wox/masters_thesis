#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
//#include <pcl/conversions.h>
#include <pcl_ros/transforms.h> 
#include <pcl/common/eigen.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Eigen/Geometry>
#include <ros/callback_queue.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include <iostream>

int VERBOSE = 0;

   
class mapper{
  public:
    mapper(){
      T = Eigen::Transform<tfScalar, 3, Eigen::Affine>::Identity();
      //M = new pcl::PointCloud<pcl::PointXYZ>;
    }
    //Return the current map
    pcl::PointCloud<pcl::PointXYZ> get_map(){
      return M;
    }

    sensor_msgs::PointCloud2 get_map_msg(){
      pcl::PCLPointCloud2 pcl2_data;

      pcl::toPCLPointCloud2(M, pcl2_data);

      sensor_msgs::PointCloud2 map_msg;
      pcl_conversions::fromPCL(pcl2_data, map_msg);
      return map_msg;
    }
    

    //Update map using data received from depth sensor and current location of the robot
    void update_map(const sensor_msgs::PointCloud2::ConstPtr& data){

      //Convert data from PointCloud2 to pcl
      pcl::PCLPointCloud2 pcl2_data;
      pcl_conversions::toPCL(*data,pcl2_data);
      pcl::PointCloud<pcl::PointXYZ> pcl_data;
      pcl::fromPCLPointCloud2(pcl2_data, pcl_data);
      
      //if Map does not exist yet, create map from the current data received fromt the depth sensor
      if(M.empty()) {
        M = pcl_data;
      } else{
        //transform point cloud to correct location using current location of the robot
        pcl::PointCloud<pcl::PointXYZ> pcl_data_transformed;

        pcl::transformPointCloud(pcl_data, pcl_data_transformed, T);

        //Match point cloud to the map

      
        //Combine pointclouds
        M += pcl_data_transformed;
      }
    }      

    //Return the current location of the robot    
    Eigen::Transform<tfScalar, 3, Eigen::Affine> get_location(){
      return T;
    }

    nav_msgs::Odometry get_location_msg(){
      nav_msgs::Odometry location_msg;
  
      location_msg.pose.pose.position.x = T.translation()[0];      
      location_msg.pose.pose.position.y = T.translation()[1];      
      location_msg.pose.pose.position.z = T.translation()[2];      

      Eigen::Quaterniond q;
      q = T.linear();
  
      location_msg.pose.pose.orientation.x = q.x();
      location_msg.pose.pose.orientation.y = q.y();
      location_msg.pose.pose.orientation.z = q.z();
      location_msg.pose.pose.orientation.w = q.w();

      return location_msg;
    }
    
    

    //Update the current location self.T using nav_msgs/Odometry.msg message
    void update_location(const nav_msgs::Odometry::ConstPtr& data){
      //add position part of the odometry data to transformation matrix
      T.translation()[0] = data->pose.pose.position.x;
      T.translation()[1] = data->pose.pose.position.y;
      T.translation()[2] = data->pose.pose.position.z;

      std::cout << data->pose.pose.position.x << '\n';
      //add orientation part of the odometry data to transformation matrix

      Eigen::Quaterniond r;
      r.x() = data->pose.pose.orientation.x;
      r.y() = data->pose.pose.orientation.y;
      r.z() = data->pose.pose.orientation.z;
      r.w() = data->pose.pose.orientation.w;

      T.linear() = r.normalized().toRotationMatrix();
      
      /*
      for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
          T.linear()[i][j] = R[i][j];
        }
      }
      */
    }

  private:
    //Initialize the map to an empty point cloud
    pcl::PointCloud<pcl::PointXYZ> M;

    //Initialize location of the robot to zero 
    Eigen::Transform<tfScalar, 3, Eigen::Affine> T;
};


int main(int argc, char **argv){
  ros::init(argc, argv, "slam_listerer");
  ros::NodeHandle n;

  mapper Map;

  ros::Subscriber pc_sub = n.subscribe("camera/depth/points", 1000, &mapper::update_map, &Map);

  ros::Subscriber IMU_sub = n.subscribe("IMU", 1000, &mapper::update_location, &Map);

  ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("map", 1000);

  ros::Publisher location_pub = n.advertise<nav_msgs::Odometry>("location", 1000);  

  ros::Rate r(10);


  while(ros::ok()){
    sensor_msgs::PointCloud2 map_msg;
    nav_msgs::Odometry location_msg;

    map_msg = Map.get_map_msg();
    location_msg = Map.get_location_msg();  

    map_pub.publish(map_msg);
    location_pub.publish(location_msg);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
