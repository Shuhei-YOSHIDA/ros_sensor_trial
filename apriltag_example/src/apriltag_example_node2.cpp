/**
 * @file apriltag_example_node2.cpp
 * @brief Detect the tag(id=0),  set a path on it, and navigate the camera to follow the path
 */

#include "tf2/exceptions.h"
#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>

using namespace std;

nav_msgs::Path generatePath()
{
  nav_msgs::Path path_msg;

  return path_msg;
}

visualization_msgs::MarkerArray pathToMarkers(const nav_msgs::Path& path_msg)
{
  visualization_msgs::MarkerArray mrks_msg;

  return mrks_msg;
}

visualization_msgs::MarkerArray makeNaviMarkers(string comment, const geometry_msgs::TransformStamped& trg)
{
  visualization_msgs::MarkerArray navi_msg;
  if (trg.child_frame_id == "") // Make Error-info marker
  {
    // Put error-info marker in front of camera

  }
  else // Make Navi marker
  {
    // Pose of tag origin is converted to camera-coord

  }

  return navi_msg;
}

void tagCB(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  // What for?

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apriltag_example_node2");
  ros::NodeHandle nh;

  ROS_INFO("Use fiducial marker and broadcast its TF, please");

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
  ros::Publisher path_mrks_pub = nh.advertise<visualization_msgs::MarkerArray>("path_mrk", 1, true);
  ros::Publisher navi_mrks_pub = nh.advertise<visualization_msgs::MarkerArray>("navi_mrk", 1, true);
  ros::Subscriber tags_sub = nh.subscribe("tag_detections", 1, tagCB);

  const string target_tag_frame_id = "tag_0";
  const string camera_frame_id = argc > 1 ? argv[1] : "camera_color_optical_frame";

  // Generate path
  auto path_msg = generatePath();
  auto mrks_msg = pathToMarkers(path_msg);

  // Publish path
  path_pub.publish(path_msg);
  path_mrks_pub.publish(mrks_msg);

  ROS_INFO("Recoginize the tag by your camera, please");
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  ros::Rate loop(30);
  // Start to navigate(Check whether camera reaches to each waypoint)
  for (auto target : path_msg.poses)
  {
    ROS_INFO("Current target is ..."); /// @todo
    while (ros::ok())
    {
      ros::spinOnce();
      // Navigate by TF
      geometry_msgs::TransformStamped tf_msg;
      try
      {
        tf_msg = tf_buffer.lookupTransform(camera_frame_id, target_tag_frame_id, ros::Time(0));
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        // Show error message(Marker)
        string comment = "could not find any tag";
        tf_msg.header.frame_id = camera_frame_id;
        tf_msg.child_frame_id = "";
        visualization_msgs::MarkerArray navi_mrk = makeNaviMarkers(comment, tf_msg);
        navi_mrks_pub.publish(navi_mrk);
        loop.sleep();
        continue;
      }

      // Check whether camera reaches or not
      bool is_reached = false;
      if (is_reached)
      {
        string comment = "Reached!";
        visualization_msgs::MarkerArray navi_mrk = makeNaviMarkers(comment, tf_msg);
        navi_mrks_pub.publish(navi_mrk);
        ROS_INFO("Reached");
        ros::Duration(1.0).sleep();
        break;
      }
      else
      {
        // Navigate camera
        visualization_msgs::MarkerArray navi_mrk;// = makeNaviMarkers();
        navi_mrks_pub.publish(navi_mrk);
      }

      loop.sleep();
    }
  }
  ROS_INFO("Finished");
  ros::waitForShutdown();

  return 0;
}
