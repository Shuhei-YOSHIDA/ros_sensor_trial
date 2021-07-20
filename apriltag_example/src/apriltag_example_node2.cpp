/**
 * @file apriltag_example_node2.cpp
 * @brief Detect the tag(id=0),  set a path on it, and navigate the camera to follow the path
 */

#include "tf2/convert.h"
#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>

using namespace std;

nav_msgs::Path generatePath(string tag_id = "tag_0")
{
  const double radius = 0.5;
  const unsigned int layer_num = 3;
  const unsigned int separate_num = 6; // (360/60)
  const double layer_dis = 0.5;
  const double least_height = 0.5;

  nav_msgs::Path path_msg;
  path_msg.header.frame_id = tag_id;
  auto stamp = ros::Time::now();
  path_msg.header.stamp = stamp;

  for (int i = 0; i < layer_num; i++)
  {
    for (int j = 0; j < separate_num + 1; j++)
    {
      double yaw_rad = 2.*M_PI/separate_num * j;
      geometry_msgs::PoseStamped p;
      p.header.frame_id = tag_id;
      p.header.stamp = stamp;
      p.pose.position.x = radius*cos(yaw_rad);
      p.pose.position.y = radius*sin(yaw_rad);
      p.pose.position.z = least_height + layer_dis * i;

      tf2::Quaternion q;
      q.setRPY(0, 0, M_PI + yaw_rad);
      tf2::convert(q, p.pose.orientation);

      path_msg.poses.push_back(p);
    }
  }

  return path_msg;
}

visualization_msgs::MarkerArray pathToMarkers(const nav_msgs::Path& path_msg)
{
  visualization_msgs::MarkerArray mrks_msg;
  int count = 0;
  for (auto pose : path_msg.poses)
  {
    visualization_msgs::Marker mrk;
    mrk.header.frame_id = pose.header.frame_id;
    mrk.header.stamp = ros::Time::now();
    mrk.pose = pose.pose;
    mrk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    mrk.text = "id_" + to_string(count);
    mrk.id = count;
    count++;
    mrk.color.a = 1.0;
    mrk.color.r = 0.678431;
    mrk.color.g = 0.847059;
    mrk.color.b = 0.901961; // light blue
    mrk.scale.z = 0.030;

    mrks_msg.markers.push_back(mrk);
  }

  return mrks_msg;
}

visualization_msgs::MarkerArray makeNaviMarkers(string comment,
    const geometry_msgs::TransformStamped& trg_on_camera_frame)
{
  auto trg = trg_on_camera_frame;
  visualization_msgs::MarkerArray navi_msg;
  auto stamp = ros::Time::now();
  if (trg.child_frame_id == "") // Make Error-info marker
  {
    // Put error-info marker in front of camera
    visualization_msgs::Marker text_mrk;
    text_mrk.header.frame_id = trg.header.frame_id;
    text_mrk.header.stamp = stamp;
    text_mrk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_mrk.text = "No tag detected";
    text_mrk.color.a = 1.0;
    text_mrk.color.r = 1.0; // Red
    text_mrk.scale.z = 0.030;
    text_mrk.pose.position.z = 0.2;
    text_mrk.pose.orientation.w = 1.0;
    text_mrk.id = 0;

    navi_msg.markers.push_back(text_mrk);

    visualization_msgs::Marker error_mrk;
    error_mrk.header.frame_id = trg.header.frame_id;
    error_mrk.header.stamp = stamp;
    error_mrk.type = visualization_msgs::Marker::SPHERE;
    error_mrk.color.a = 0.6;
    error_mrk.color.r = 1.0;
    error_mrk.color.g = 1.0; // Yellow
    error_mrk.scale.x = error_mrk.scale.y = error_mrk.scale.z = 0.025;
    error_mrk.pose.position.z = text_mrk.pose.position.z + 0.1;
    error_mrk.id = 1;

    navi_msg.markers.push_back(error_mrk);
  }
  else // Make Navi marker
  {
    // Pose of tag origin is converted to camera-coord
    geometry_msgs::PoseStamped tag_origin, tag_pose_on_cam;
    tag_origin.header.frame_id = trg.child_frame_id;
    tag_origin.pose.orientation.w = 1.0;
    tf2::doTransform(tag_origin, tag_pose_on_cam, trg);

    visualization_msgs::Marker text_mrk;
    text_mrk.header.frame_id = trg.header.frame_id;
    text_mrk.header.stamp = stamp;
    text_mrk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_mrk.text = "Follow the arrow";
    text_mrk.color.a = 1.0;
    text_mrk.color.g = 1.0;
    text_mrk.color.b = 1.0; // Aqua
    text_mrk.scale.z = 0.030;
    text_mrk.pose.position.z = 0.2;
    text_mrk.pose.orientation.w = 1.0;
    text_mrk.id = 0;

    navi_msg.markers.push_back(text_mrk);

    visualization_msgs::Marker arrow_mrk;
    arrow_mrk.header.frame_id = trg.header.frame_id;
    arrow_mrk.header.stamp = stamp;
    arrow_mrk.type = visualization_msgs::Marker::ARROW;
    arrow_mrk.color.a = 1.0;
    arrow_mrk.color.r = 1.0;
    arrow_mrk.color.g = 1.0; // Yellow
    arrow_mrk.pose.orientation.w = 1.0;
    arrow_mrk.points.resize(2);
    arrow_mrk.points[0].x = arrow_mrk.points[0].y = arrow_mrk.points[0].z = 0.0;
    arrow_mrk.points[1].x = trg.transform.translation.x;
    arrow_mrk.points[1].y = trg.transform.translation.y;
    arrow_mrk.points[1].z = trg.transform.translation.z;
    arrow_mrk.scale.x = 0.010;
    arrow_mrk.scale.y = 0.020;
    arrow_mrk.id = 1;

    navi_msg.markers.push_back(arrow_mrk);
  }

  return navi_msg;
}

void tagCB(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  // What for?

}

// tf_camera_tag : transform tag_id pose into camera coord system
bool isCameraReached(const geometry_msgs::PoseStamped& target_pose_on_tag_id,
                     const geometry_msgs::TransformStamped& tf_camera_tag )
{
  geometry_msgs::PoseStamped target_pose_on_camera;
  tf2::doTransform(target_pose_on_tag_id, target_pose_on_camera, tf_camera_tag);
  Eigen::Vector3d p;
  tf::pointMsgToEigen(target_pose_on_camera.pose.position, p);
  Eigen::Quaterniond q;
  tf::quaternionMsgToEigen(target_pose_on_camera.pose.orientation, q);

  if (p.norm() < 0.1 && Eigen::AngleAxisd(q).angle() < 3.0 * M_PI/180.)
  {
    return true;
  }

  return false;
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
  auto path_msg = generatePath(target_tag_frame_id);
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
      if (isCameraReached(target, tf_msg))
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
        string comment = "go to target";
        visualization_msgs::MarkerArray navi_mrk = makeNaviMarkers(comment, tf_msg);
        navi_mrks_pub.publish(navi_mrk);
      }

      loop.sleep();
    }
  }
  ROS_INFO("Finished");
  ros::waitForShutdown();

  return 0;
}
