#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle):
  nodeHandle_(nodeHandle)
{
  if (!(nodeHandle_.getParam("param_topic_name",topic_name_))& nodeHandle_.getParam("param_topic_queue_size",topic_size_queue_)&
      (nodeHandle_.getParam("param_linear_p_gain",linear_p_gain))&(nodeHandle_.getParam("param_angular_p_gain",angular_p_gain)))
  {
    ROS_ERROR("Parameter reading error");
    ros::requestShutdown();
  }





   subscriber_ = nodeHandle_.subscribe(topic_name_, topic_size_queue_,
                                      &HuskyHighlevelController::scanCallback, this);

   publisher_vel_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
   publisher_marker_ = nodeHandle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );


  ROS_INFO("Successfully launched node.");
}

HuskyHighlevelController::~HuskyHighlevelController()
{

}


void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan& msg)
{
  std::int32_t min_index=0;
  double small = msg.ranges[min_index];
  for (std::size_t i=0; i< msg.ranges.size(); i++)
  {
    if(small > msg.ranges[i])
    {
      small = msg.ranges[i];
      min_index = i;
    }
  }


  double range_resolution=(msg.angle_max-msg.angle_min)/msg.ranges.size();
  double meas_ang = range_resolution * min_index + msg.angle_min; //rad

  ROS_INFO_STREAM(" min_dint: " + std::to_string(small));
  ROS_INFO_STREAM(" meas_ang:"+std::to_string(meas_ang));

  geometry_msgs::Point pillar;
  pillar.x=small*cos(meas_ang);
  pillar.y=small*sin(meas_ang);

  ROS_INFO_STREAM(" x: " + std::to_string(pillar.x));
  ROS_INFO_STREAM(" y: "+std::to_string(pillar.y));

  double linear_error = pillar.x;
  double angular_error = meas_ang;

  vel_cmd.linear.x = linear_error * linear_p_gain - 0.2f ; //stop ahead the pillar
  vel_cmd.angular.z = -1*(angular_error * angular_p_gain); // manual coordinate transformation
  publisher_vel_.publish(vel_cmd);

  ROS_INFO_STREAM(" linear_error: " + std::to_string(linear_error));
  ROS_INFO_STREAM(" angular_error:"+std::to_string(angular_error));


  /*
   *
   *  difficult version
   */
//  geometry_msgs::Point pillarfFromOdom;
//  try{
//  geometry_msgs::TransformStamped transformStamped = tfBuffer_.lookupTransform("odom","base_link",ros::Time(0));
//  tf2::doTransform(pillar,pillarfFromOdom, transformStamped);
//
//  }
//
//
//  catch (tf2::TransformException &ex)
//  {
//    ROS_WARN("%s",ex.what());
//    ros::Duration(1.0).sleep();
//  }




  /* Rviz marker */
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link"; // easy version
//  marker.header.frame_id = "odom"; // difficult version
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pillar.x; // easy version
  marker.pose.position.y = (-1)*pillar.y; // easy version
//  marker.pose.position.x = pillarfFromOdom.x; // difficult version
//  marker.pose.position.y = pillarfFromOdom.y; // difficult version
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;


  publisher_marker_.publish( marker );


}


} /* namespace */
