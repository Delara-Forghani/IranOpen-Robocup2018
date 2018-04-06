#include <cstdlib>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <victim_detection/Object.h>
#include <victim_detection/Detections.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#define PI 3.14159265

int imageRows = 0;
int imageColumns = 0;
std::vector<float> range_of_laser;
double dist_ray_towards_victim = -1;
double robot_yaw_angle = 0;
double robot_position_angel=0;
double robotx = 0;
double roboty = 0;
ros::Publisher dead_victPub, alive_victPub;
//std::vector<unsigned char> thermalImageArray;
std::string robot_namespace;
int width_of_image = 0;
int number_of_rays = 0;
double angle_range_camera = 0;
int angle_range_laser = 0;    
  struct victLocation {
    double x,y;
  };
std::vector<victLocation> victims;
double find_Distance(int left,int right){
    // *** should change when laser changes
    double min = 18.0;
    ROS_INFO("leftrange %f, rightrange %f ",range_of_laser[left],range_of_laser[right]);
    ROS_INFO("leftbeam %d, rightbeam %d ",left,right);
    for (int i = left; i <= right; i++){
      ROS_INFO("beam %d, range %f ",i,range_of_laser[i]);
      if((range_of_laser[i] <= min)&&(range_of_laser[i] >= 1))
        min = range_of_laser[i];
      }
    ROS_INFO("distance calculated!  \n");
    return min;
  }
victLocation find_victim_location(double vict_angle){
    if (dist_ray_towards_victim == -1){
      ROS_INFO("Distance value is not allowed! it is -1 \n");
	}
    double victx = robotx + dist_ray_towards_victim * cos(robot_yaw_angle -( vict_angle*PI / 180 ));
    double victy = roboty + dist_ray_towards_victim * sin(robot_yaw_angle -( vict_angle*PI / 180 ));
    ROS_INFO("dist %f ryas %f  va %f \n",dist_ray_towards_victim,robot_yaw_angle,vict_angle);
    victLocation vl;
    vl.x = victx;
    vl.y = victy;
    ROS_INFO("victim location is :  \n");
    return vl;
  }


void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
       range_of_laser = scan->ranges;
  }


void detectionCallback(const victim_detection::Detections::ConstPtr& detect){
 std::vector<victim_detection::Object> obi = detect->objects;
    if(obi.size() > 0)
    {
      if (obi[0].label == "Human"){
       int right_side = obi[0].right_top_x - width_of_image / 2;
       double right_angle = atan2((double)right_side,(double)277.19135641132)*((double)180/PI);
       int right_beam = (double)((double)((double)number_of_rays / (double)angle_range_laser) * right_angle) + (double)(number_of_rays / 2) ;
       int left_side = obi[0].left_bot_x - width_of_image / 2 ;
       double left_angle=atan2((double) left_side,(double)277.19135641132)*((double)180/PI);
       int left_beam = (double)((double)((double)number_of_rays / (double)angle_range_laser) * left_angle) + (double)(number_of_rays / 2);
       dist_ray_towards_victim = find_Distance(left_beam,right_beam);
       victims.push_back(find_victim_location((right_angle + left_angle)/2));
           
    }
  }
}

void robotPositionCallback(const nav_msgs::Odometry::ConstPtr& odom){
 robotx=odom->pose.pose.position.x;
 roboty=odom->pose.pose.position.y;
 tf::Pose pose;
 tf::poseMsgToTF(odom->pose.pose, pose);
 robot_yaw_angle = tf::getYaw(pose.getRotation());

}

int main(int argc, char **argv)
{
 std::string laser_topic,detector_topic,odom_topic;
 ros::init(argc, argv, "victim_position");
 ros::NodeHandle nh;
 ros::Subscriber scanSubscriber;
 ros::Subscriber victimSubscriber;
 ros::Subscriber robotSubscriber;
 ros::Rate pub_rate(20);

 nh.getParam("victim_position/robot_namespace", robot_namespace);
 nh.getParam("victim_position/width_of_image", width_of_image);
 nh.getParam("victim_position/angle_range_laser", angle_range_laser);
 nh.getParam("victim_position/angle_range_camera", angle_range_camera);
 nh.getParam("victim_position/number_of_rays",number_of_rays);

  laser_topic = '/' + robot_namespace + "/scan";
 // thermal_str = '/' + robot_namespace + "/themral_camera/image";
 // detector_str = '/' + robot_namespace + "/detector_node/detections";    //????
  detector_topic = "/detector_node/detections";
  odom_topic = '/' + robot_namespace + "/odom";


 scanSubscriber=nh.subscribe<sensor_msgs::LaserScan>(laser_topic,10,laserScanCallback);
 victimSubscriber = nh.subscribe<victim_detection::Detections>(detector_topic,10,detectionCallback);
 robotSubscriber=nh.subscribe<nav_msgs::Odometry>(odom_topic,10,robotPositionCallback);
 while (ros::ok())
  {

    ros::spinOnce();
    pub_rate.sleep();
  }
  exit(0);
  return 0;
}
