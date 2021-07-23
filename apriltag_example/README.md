apriltag_example
====

Example of apriltag_ros package

# Prepare your camera
As a example, Realsense d435 is used.
```
$ roslaunch realsense2_camera rs_d435_camera_with_model.launch
```

# continueous_detection_1.launch
```
$ roslaunch apriltag_example continuous_detection_1.launch \
  camera_frame:="camera_color_optical_frame" \
  image_topic:="/camera/color/image_raw" \
  camera_info_topic:="/camera/color/camera_info"
```

Fix arguments for your camera.

# Examples1
## apriltag_example_node1
After apriltag is executed, run this node.
```
$ rosrun apriltag_example apriltag_example_node1
```
Markers are published on the tag whose id is zero.

## apriltag_example_node2
After apriltag is executed, run this node.
```
$ rosrun apriltag_example apriltag_example_node2
```

A path is set on fram_id "tag_0"(the tag of id=0).
And you move the camera to each waypoints which is pointed by yellow arrow.

## apriltag_exmaple_node3
After apriltag or rosbag replay is executed, run this node.
```
$ rosrun apriltag_example apriltag_example_node3
```

You should put tags whose ids are 0, 1, and 2.
A tag of id=1 should be put on y=1.51m from id=0's tag.
And a tag of id=2 should be put on y=-1.51m from id=0's tag.

The node publishes markers to represents moving average of poses of tags(id=1,2) from the tag of id=0.
