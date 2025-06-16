misys@AIX-411-UBUNTU-22:~/f1tenth_ws/src/lqr_pid$ ros2 topic echo /tf_static 
transforms:
- header:
    stamp:
      sec: 1750073332
      nanosec: 46160514
    frame_id: ego_racecar/base_link
  child_frame_id: ego_racecar/back_left_wheel
  transform:
    translation:
      x: 0.0
      y: 0.12065
      z: 0.0508
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1750073332
      nanosec: 46160514
    frame_id: ego_racecar/base_link
  child_frame_id: ego_racecar/back_right_wheel
  transform:
    translation:
      x: 0.0
      y: -0.12065
      z: 0.0508
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1750073332
      nanosec: 46160514
    frame_id: ego_racecar/base_link
  child_frame_id: ego_racecar/front_left_hinge
  transform:
    translation:
      x: 0.3302
      y: 0.12065
      z: 0.0508
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1750073332
      nanosec: 46160514
    frame_id: ego_racecar/base_link
  child_frame_id: ego_racecar/front_right_hinge
  transform:
    translation:
      x: 0.3302
      y: -0.12065
      z: 0.0508
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1750073332
      nanosec: 46160514
    frame_id: ego_racecar/base_link
  child_frame_id: ego_racecar/laser_model
  transform:
    translation:
      x: 0.275
      y: 0.0
      z: 0.165
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---
transforms:
- header:
    stamp:
      sec: 1750073693
      nanosec: 733544183
    frame_id: map
  child_frame_id: odom
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
