#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/buffer.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>


namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_vel_;
  ros::Publisher publisher_marker_;
  std::string topic_name_;
  std::int32_t topic_size_queue_;
  geometry_msgs::Twist vel_cmd;
  double linear_p_gain;
  double angular_p_gain;

//
//  tf2_ros::Buffer tfBuffer;
//  tf2_ros::TransformListener tfListener(tfBuffer);


  void scanCallback(const sensor_msgs::LaserScan& msg);



};

} /* namespace */
