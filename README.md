# dxslam_ros

This ROS package is a wrapper of [DXSLAM](https://github.com/ivipsourcecode/dxslam) and [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2). For either system, this project can build a ROS node which takes RGB-D images from ROS topics, feed them into the SLAM system, and publish the estimated camera pose.

The node also takes care of ROS tf. It publishes the transformation from the world frame to the robot/sensor frame. Both can be specified by the user. For example,
1. when running on a robot with a fixed frame from `base_link` to `camera`, one can specify `pub_tf_child_frame:=base_link` to let the node publish tf from `map` to `base_link`;
2. when the robot has an odometry node publishing tf from `odom` to `base_link`, one can specify `pub_tf_child_frame:=odom`;
3. when running with a handheld camera, one may specify `pub_tf_child_frame:=camera` or `pub_tf_child_frame:=camera_optical_frame`.

Note that the estimated pose from the SLAM system is the transform between the current camera optical frame and a fixed reference frame. The ROS node will check other information from the tf tree, including tf between `pub_tf_parent_frame` and the reference frame, and that between `pub_tf_child_frame` to the camera optical frame (as specified in the header of the RGB image). It then calculates the tf between `pub_tf_parent_frame` and `pub_tf_child_frame` based on above information and the esaimted pose.

It can also reads camera intrinsics from the camera_info topic (highly recommended so you don't have to change your config file when changing camera!). To enable it, put `Camera.fromTopic: 1` into your config file ([example](https://github.com/cedrusx/dxslam_ros/blob/dxslam/config/realsense_d435.yaml)).

# Supported SLAM systems

If you use any of the systems in a research work, please properly cite the corresponding paper.

### DXSLAM [[code](https://github.com/ivipsourcecode/dxslam)][[pdf](https://arxiv.org/pdf/2008.05416.pdf)]

> Dongjiang Li, Xuesong Shi, Qiwei Long, Shenghui Liu, Wei Yang, Fangshi Wang, Qi Wei, Fei Qiao, "DXSLAM: A Robust and Efficient Visual SLAM System with Deep Features," in 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Oct 2020, pp. 4958–4965.

```
@inproceedings{li2020dxslam,
  title={{DXSLAM}: A Robust and Efficient Visual {SLAM} System with Deep Features},
  author={Li, Dongjiang and Shi, Xuesong and Long, Qiwei and Liu, Shenghui and Yang, Wei and Wang, Fangshi and Wei, Qi and Qiao, Fei},
  booktitle={2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4958--4965},
  year={2020},
  month={Oct},
}
```

### ORB-SLAM2 [[code](https://github.com/raulmur/ORB_SLAM2)][[pdf](https://arxiv.org/pdf/1610.06475.pdf)]

> Raul Mur-Artal and Juan D Tard ́os, "ORB-SLAM2: An open-source SLAM system for monocular, stereo, and RGB-D cameras," IEEE Transactions on Robotics, vol. 33, no. 5, pp. 1255–1262, 2017.

```
@article{mur2017orb,
  title={{ORB-SLAM2}: An open-source {SLAM} system for monocular, stereo, and {RGB-D} cameras},
  author={Mur-Artal, Raul and Tard{\'o}s, Juan D},
  journal={IEEE Transactions on Robotics},
  volume={33},
  number={5},
  pages={1255--1262},
  year={2017},
  publisher={IEEE}
}
```

# Setup

First, clone the codes into your catkin workspace. You can skip `dxslam` or `ORB_SLAM2` if only needing the other.
```
cd YOUR_CATKIN_WS/src/
git clone https://github.com/cedrusx/dxslam_ros
git clone https://github.com/cedrusx/deep_features
git clone https://github.com/ivipsourcecode/dxslam
git clone https://github.com/cedrusx/ORB_SLAM2  # this is a fork of raulmur/ORB_SLAM2 with a few bug fixings
```

If you already have the `dxslam` or `ORB_SLAM2` repo in your system and do not want to clone again, you can simply put them into the `dxslam_ros` folder, or alongside your catkin workspace. Then `dxslam_ros` would find it during compiling. See the `SLAM_SEARCH_PATH` in [`CMakeLists.txt`](https://github.com/cedrusx/dxslam_ros/blob/dxslam/CMakeLists.txt).

Then, build `dxslam` and/or `ORB_SLAM2` as normal. Please check the README of these repos to set up the prerequisites.
```
cd dxslam
./build.sh
cd ..

cd ORB_SLAM2
./build.sh
cd ..
```

If you use `dxslam`, you'll also need to set up ROS+Python3 environment for `deep_features`. Check its [README](https://github.com/cedrusx/deep_features).

Finally, build `dxslam_ros` with your favorate catkin tool:
```
cd YOUR_CATKIN_WS
. /opt/ros/VERSION/setup.bash
catkin build    # OR catkin_make
```

# Run

Only for DXSLAM - Run `feature_extraction/feature_extraction_node.py` from [deep_features](https://github.com/cedrusx/deep_features) to extract and publish features for images on a given topic:
```
cd YOUR_CATKIN_WS
. devel/setup.bash
rosrun feature_extraction feature_extraction_node.py [PARAMS]
```

And launch dxslam_ros in another terminal, for example:
```
cd YOUR_CATKIN_WS
. devel/setup.bash
roslaunch dxslam_ros rgbd.launch slam:=dxslam camera:=d400 pub_tf_child_frame:=base_link
# OR
roslaunch dxslam_ros rgbd.launch slam:=orbslam2 camera:=d400 pub_tf_child_frame:=base_link
```

Then play your ROS bag or launch `realsense_ros` to provide a live stream.
The arguments above are configurated for the [OpenLORIS-Scene](https://lifelong-robotic-vision.github.io/dataset/scene) datasets. You may need to change them if using other data.

When succeeded, you should be able to see live tracking results in the ORB_SLAM2 GUI, and properly maintained ROS tf and pose topics with RViz or `rqt_tf_tree`.
