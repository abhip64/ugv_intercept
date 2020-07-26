#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <mavros_msgs/SetMode.h>
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <Eigen/Dense>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <geometry_msgs/PoseStamped.h>

#include "quad_perch/math_operations.h"


geometry_msgs::Pose vehicle_pose;
geometry_msgs::PoseWithCovarianceStamped vehicle_stamped_pose, image_pose; 
ros::Time curr_time;

bool find_index;
int base_link_index; 

int image_id = 0;

Eigen::Vector3d image_pos, image_pos_earth_frame, mavPos_;
Eigen::Vector4d mavAtt_;


void image_pos_transform()
{
  image_pos << -image_pos(1) + 0.1, -image_pos(0), -image_pos(2);

  image_pos = quat2RotMatrix(mavAtt_)*image_pos;
  //Eigen::Vector4d 
  //image_pos = mavAtt_*image_pos*mavAtt_;

  image_pos_earth_frame = image_pos + mavPos_;

  //std::cout<<image_pos_earth_frame<<"\n"<<"$$$$$$$$$$"<<"\n";
}

void image_pos_sub(const apriltag_ros::AprilTagDetectionArray& msg){
  
  if(!(msg.detections.empty()))
  {
  //   std::cout<<"C************"<<"\n";

  image_id = msg.detections[0].id[0];

  // std::cout<<image_id<<"\n";
  //image_pose = msg.detections[0].pose;
  image_pose = msg.detections[0].pose;

  image_pos << msg.detections[0].pose.pose.pose.position.x, msg.detections[0].pose.pose.pose.position.y, msg.detections[0].pose.pose.pose.position.z;

  curr_time = ros::Time::now();

  image_pos_transform();
}
}


void mavposeCallback(const geometry_msgs::PoseStamped& msg){

  mavPos_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "apriltag_data_transform");

  ros::NodeHandle node;

  curr_time = ros::Time::now();

  ros::Publisher vehicle_img_pos_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ugv_camera/pose", 1000);

  ros::Subscriber mavposeSub_      = node.subscribe("/mavros/local_position/pose", 50, mavposeCallback,ros::TransportHints().tcpNoDelay());

  ros::Subscriber imageposSub_   = node.subscribe("/tag_detections", 30, image_pos_sub,ros::TransportHints().tcpNoDelay());

  ros::Rate loop_rate(30);

  int frame_sequence_ = 0;


  while (ros::ok())
  {

    if(image_id == 7)
    {
    vehicle_stamped_pose.header.stamp = curr_time;

    vehicle_stamped_pose.header.seq = ++ frame_sequence_;

    vehicle_stamped_pose.header.frame_id = "odom";

    vehicle_stamped_pose.pose.pose.position.x = image_pos_earth_frame(0);
    vehicle_stamped_pose.pose.pose.position.y = image_pos_earth_frame(1);
    vehicle_stamped_pose.pose.pose.position.z = image_pos_earth_frame(2);

    vehicle_stamped_pose.pose.covariance = image_pose.pose.covariance;

   // vehicle_img_pos_pub.publish(vehicle_stamped_pose);
    }
    ros::spinOnce();

    loop_rate.sleep();
    
  }


  return 0;
}