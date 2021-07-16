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
$ roslaunch apriltag_example continueous_detection_1.launch \
  camera_frame:="camera_color_optical_frame" \
  image_topic:="/camera/color/image_raw" \
  camera_info_topic:="/camera/color/camera_info"
```

Fix arguments for your camera.

# Examples1
## apriltag_example_node1
After apriltag is executed, run this node.
```
$ rosrun apriltag_example apriltag_example_node
```
Markers are published on the tag whose id is zero.

## apriltag_example_node2
