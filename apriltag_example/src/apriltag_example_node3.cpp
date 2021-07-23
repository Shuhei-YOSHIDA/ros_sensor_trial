/**
 * @file apriltag_example_node3.cpp
 * @brief Calculate moving average and varience os poses of tags(id=1,2) from a pose of the tag(id=0)
 */

#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <deque>

using namespace std;

class MovingAve
{
public:
  MovingAve(double time_duration, string frame_id, string child_frame_id)
    : _dur(time_duration), _frame_id(frame_id), _child_frame_id(child_frame_id)
  {
  }

  void addData(const geometry_msgs::TransformStamped& msg);
  bool calcMovingAve(geometry_msgs::TransformStamped& ave);

private:
  deque<geometry_msgs::TransformStamped> _deq;
  ros::Duration _dur;
  string _frame_id;
  string _child_frame_id;
};

void MovingAve::addData(const geometry_msgs::TransformStamped& msg)
{
  if (msg.header.frame_id != "")  _deq.push_back(msg);

  auto now_stamp = ros::Time::now();
  while (true && !_deq.empty())
  {
    if (_deq.front().header.stamp < now_stamp - _dur) _deq.pop_front();
    else break;
  }
}

bool MovingAve::calcMovingAve(geometry_msgs::TransformStamped& ave)
{
  if (_deq.empty()) return false;

  ave.header.frame_id = _frame_id;
  ave.child_frame_id = _child_frame_id;

  ave.transform.rotation.w = 1.0; ///@todo FIXIT

  int n = _deq.size();
  for (auto it = _deq.begin(); it != _deq.end(); it++)
  {
    ave.transform.translation.x += (*it).transform.translation.x;
    ave.transform.translation.y += (*it).transform.translation.y;
    ave.transform.translation.z += (*it).transform.translation.z;
  }

  ave.transform.translation.x /= n;
  ave.transform.translation.y /= n;
  ave.transform.translation.z /= n;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apriltag_example_node3");
  ros::NodeHandle nh;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  string id0 = "tag_0";
  string id1 = "tag_1";
  string id2 = "tag_2";

  Eigen::Vector3d ref_01(0, +1.51, 0);
  Eigen::Vector3d ref_02(0, -1.51, 0);

  double time_duration_sec(5.);
  MovingAve ave01(time_duration_sec, id0, id1);
  MovingAve ave02(time_duration_sec, id0, id2);

  auto poses_pub = nh.advertise<geometry_msgs::PoseArray>("moving_ave", 1);
  auto mrks_pub = nh.advertise<visualization_msgs::MarkerArray>("moving_ave_mrks", 1);

  geometry_msgs::PoseArray posea_msg;
  visualization_msgs::MarkerArray mrks_msg;
  posea_msg.header.frame_id = id0;
  posea_msg.poses.resize(2);
  posea_msg.poses[0].orientation.w = 1.0;
  posea_msg.poses[1].orientation.w = 1.0;
  posea_msg.poses[0].position.x = ref_01[0]; 
  posea_msg.poses[0].position.y = ref_01[1]; 
  posea_msg.poses[0].position.z = ref_01[2]; 
  posea_msg.poses[1].position.x = ref_02[0]; 
  posea_msg.poses[1].position.y = ref_02[1]; 
  posea_msg.poses[1].position.z = ref_02[2]; 
  mrks_msg.markers.resize(2);
  for (int i = 0; i < 2; i++)
  {
    mrks_msg.markers[i].header.frame_id = id0;
    mrks_msg.markers[i].color.a = 1.0;
    mrks_msg.markers[i].color.r = 1.0;
    mrks_msg.markers[i].id = i;
    mrks_msg.markers[i].pose = posea_msg.poses[i];
    mrks_msg.markers[i].scale.z = 0.50;
    mrks_msg.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    mrks_msg.markers[i].text = "not detected";
  }


  ros::Rate loop(50);

  while(ros::ok())
  {
    ros::spinOnce();
    geometry_msgs::TransformStamped tag_01, tag_02;
    //auto stamp = ros::Time::now() - ros::Duration(0.100);
    auto stamp = ros::Time::now();// - ros::Duration(0.100);
    try
    {
      tf_buffer.canTransform(id0, id1, stamp, ros::Duration(1./30));
      tag_01 = tf_buffer.lookupTransform(id0, id1, stamp);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_DEBUG("Could not obtain tf tag0-tag1");
    }

    try
    {
      tf_buffer.canTransform(id0, id2, stamp, ros::Duration(1./30));
      tag_02 = tf_buffer.lookupTransform(id0, id2, stamp);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_DEBUG("Could not obtain tf tag0-tag2");
    }

    ave01.addData(tag_01);
    ave02.addData(tag_02);

    geometry_msgs::TransformStamped tf_ave_01;
    if (ave01.calcMovingAve(tf_ave_01))
    {
      double x, y, z;
      posea_msg.poses[0].orientation = tf_ave_01.transform.rotation;
      x = posea_msg.poses[0].position.x = tf_ave_01.transform.translation.x;
      y = posea_msg.poses[0].position.y = tf_ave_01.transform.translation.y;
      z = posea_msg.poses[0].position.z = tf_ave_01.transform.translation.z;
      Eigen::Vector3d r(x, y, z);

      mrks_msg.markers[0].pose = posea_msg.poses[0];
      mrks_msg.markers[0].pose.position.z += 0.5;
      mrks_msg.markers[0].text =
        to_string(x) + "m\n" + to_string(y) + "m\n" + to_string(z) + "m\n"
        + "norm: " + to_string((r-ref_01).norm()) + "m";
    }
    else
    {
      posea_msg.poses[0].orientation.w = 1.0;
      posea_msg.poses[0].position.x = ref_01[0];
      posea_msg.poses[0].position.y = ref_01[1];
      posea_msg.poses[0].position.z = ref_01[2];

      mrks_msg.markers[0].pose = posea_msg.poses[0];
      mrks_msg.markers[0].pose.position.z += 0.5;
      mrks_msg.markers[0].text = "tag 0 to 1 not detected";
    }

    geometry_msgs::TransformStamped tf_ave_02;
    if (ave02.calcMovingAve(tf_ave_02))
    {
      double x, y, z;
      posea_msg.poses[1].orientation = tf_ave_02.transform.rotation;
      x = posea_msg.poses[1].position.x = tf_ave_02.transform.translation.x;
      y = posea_msg.poses[1].position.y = tf_ave_02.transform.translation.y;
      z = posea_msg.poses[1].position.z = tf_ave_02.transform.translation.z;
      Eigen::Vector3d r(x, y, z);

      mrks_msg.markers[1].pose = posea_msg.poses[1];
      mrks_msg.markers[1].pose.position.z += 0.5;
      mrks_msg.markers[1].text =
        to_string(x) + "m\n" + to_string(y) + "m\n" + to_string(z) + "m\n"
        + "norm: " + to_string((r-ref_02).norm()) + "m";
    }
    else
    {
      posea_msg.poses[1].orientation.w = 1.0;
      posea_msg.poses[1].position.x = ref_02[0];
      posea_msg.poses[1].position.y = ref_02[1];
      posea_msg.poses[1].position.z = ref_02[2];

      mrks_msg.markers[1].pose = posea_msg.poses[1];
      mrks_msg.markers[1].pose.position.z += 0.5;
      mrks_msg.markers[1].text = "tag 0 to 2 not detected";
    }

    posea_msg.header.stamp = stamp;
    for (auto&& m : mrks_msg.markers) m.header.stamp = stamp;
    poses_pub.publish(posea_msg);
    mrks_pub.publish(mrks_msg);

    loop.sleep();
  }

  return 0;
}
