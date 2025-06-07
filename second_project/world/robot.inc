define kinect ranger
(
  sensor
  (
    pose [ 0.3 0.0 0.0 0]
    size [0.1 0.1 0.1 ]
    range  [0 100]
    fov 120.0
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.1 0.1 0.03 ]
)

define kinect1 ranger
(
  sensor
  (
    pose [ -0.3 0.0 0.0 180]
    size [0.1 0.1 0.1 ]
    range  [0 100]
    fov 120.0
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.1 0.1 0.03 ]
)

define robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  
  localization "odom"


  odom_error [0.2 0.2 0.0 0.3 ]

  size [ 0.54 0.4 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "grey"
  
  kinect()
  kinect1()
)
