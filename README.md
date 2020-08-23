# dxslam_ros

This ROS package is a wrapper of DXSLAM described in [this paper](https://arxiv.org/pdf/2008.05416) (to be published in IROS 2020):

> Dongjiang Li, Xuesong Shi, Qiwei Long, Shenghui Liu, Wei Yang, Fangshi Wang, Qi Wei, Fei Qiao, "DXSLAM: A Robust and Efficient Visual SLAM System with Deep Features," arXiv preprint arXiv:2008.05416, 2020.

```
@article{li2020dxslam,
  title={{DXSLAM}: A Robust and Efficient Visual {SLAM} System with Deep Features},
  author={Dongjiang Li and Xuesong Shi and Qiwei Long and Shenghui Liu and Wei Yang and Fangshi Wang and Qi Wei and Fei Qiao},
  journal={arXiv preprint arXiv:2008.05416},
  year={2020}
}
```

Note that the feature extraction module is a separate ROS package not described here. We suggest you to set it up firstly:
[deep_features](https://github.com/cedrusx/deep_features)

# Setup

First, clone this repo into your catkin workspace:
```
cd YOUR_CATKIN_WS/src/
git clone --recursive https://github.com/cedrusx/dxslam_ros
```

Note that the `--recursive` flag will tell git to clone the submodules as well, which is, the [dxslam](https://github.com/cedrusx/dxslam) repo.
If you have cloned without this flag, you can use `git submodule init; git submodule update` to check out the submodules.

If you already have the `dxslam` repo in your system and do not want to clone again, you can simply put it under `dxslam_ros`, or alongside your catkin repo.
Then `dxslam_ros` would find it during compiling. See [`CMakeLists.txt`](https://github.com/cedrusx/dxslam_ros/blob/dxslam/CMakeLists.txt).

Then, build `dxslam` as normal:
```
cd dxslam
./build.sh
```

Finally, build `dxslam_ros` with your favorate catkin tool:
```
cd YOUR_CATKIN_WS
. /opt/ros/VERSION/setup.bash
catkin build    # OR catkin_make
```

# Run

Run `feature_extraction_node.py` from `[deep_features](https://github.com/cedrusx/deep_features)/feature_extraction` to extract and publish features for images on a given topic:
```
cd YOUR_CATKIN_WS
. devel/setup.bash
rosrun feature_extraction feature_extraction_node.py [PARAMS]
```

And launch dxslam_ros in another terminal:
```
cd YOUR_CATKIN_WS
. devel/setup.bash
roslaunch dxslam_ros rgbd.launch camera:=d400 pub_tf_child_frame:=base_link
```

Then play your ROS bag or launch `realsense_ros` to provide a live stream.
The arguments above are configurated for the [OpenLORIS-Scene](https://lifelong-robotic-vision.github.io/dataset/scene) datasets. You may need to change them if using other data.

When succeeded, you should be able to see live tracking results in the DXSLAM GUI (same as in ORB_SLAM2), and properly maintained ROS tf and pose topics.
