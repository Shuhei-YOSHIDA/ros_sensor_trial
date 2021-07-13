/**
 * @file apriltag_example_node.cpp
 */

#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

using namespace std;

ros::Publisher marker_pub;

// Publish marker that is on the tag(id=0)
void example1(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  // Check whether the tag (id=0) exists in msg or not
  bool is_found = false;
  apriltag_ros::AprilTagDetection detected_tag;
  for (auto detection : msg->detections)
  {
    auto found_itr = find(detection.id.begin(), detection.id.end(), 0);
    // if standalone tag exists...
    if (found_itr != detection.id.end() && detection.id.size() == 1)
    {
      detected_tag = detection;
      is_found = true;
    }
  }
  if (!is_found) return;

  visualization_msgs::MarkerArray mrks;

  const int max_mrks = 10;
  for (int i = 0; i < max_mrks; i++)
  {
    visualization_msgs::Marker mrk;
    mrk.header.stamp = msg->header.stamp;
    mrk.id = i;
    mrk.color.a = 1;
    mrk.color.r = 1. - float(i)/max_mrks;
    mrk.color.b = 0. + float(i)/max_mrks;
    mrk.type = visualization_msgs::Marker::ARROW;
    mrk.header.frame_id = detected_tag.pose.header.frame_id;
    mrk.pose.orientation = detected_tag.pose.pose.pose.orientation;
    mrk.pose.position = detected_tag.pose.pose.pose.position;
    Eigen::Quaterniond q;
    Eigen::Vector3d p(detected_tag.size[0]*2. * i, 0, 0);
    tf::quaternionMsgToEigen(mrk.pose.orientation, q);
    p = q.inverse()*p;
    mrk.pose.position.x += p[0];
    mrk.pose.position.y += p[1];
    mrk.pose.position.z += p[2];
    mrk.scale.x = detected_tag.size[0];
    mrk.scale.y = detected_tag.size[0]/10.;
    mrk.scale.z = detected_tag.size[0]/4.;

    mrks.markers.push_back(mrk);
  }

  marker_pub.publish(mrks);
}

void tagCB(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  example1(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apriltag_example_node");
  ros::NodeHandle nh;

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("tag_mrk", 1);

  ros::Subscriber tag_sub = nh.subscribe("tag_detections", 1, tagCB);

  ros::spin();
  return 0;
}
