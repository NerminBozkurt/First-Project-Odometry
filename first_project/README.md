Perception, Localization and Mapping for Mobile Robots Course Project

21/05/25:
CmakeList package.xml ve sector_times.msg ----> paket adi first_project
calistrimak icin : 
roslaunch first_project launch.launch

--------------------------------------//-------------------------------
This package has three nodes:
odometer.cpp
gps_odometer.cpp
sector_times.launch

odometer node reads speed values and does dead reckoning odometry. Euler, runge kutta, or exact integration method is selectable via parameters. Steering angle readings are filtered before using to stabilize straight paths and sharp curves, with the tuning parameters also adjustable in the launch file. This node reads from "/speedsteer", publishes odometry message into "/odom", and broadcasts the TF.

gps_odometer node reads front gps values and computes the distance to generate the pose of the car. Reads from "/swiftnav/front/gps_pose", publishes odometry message into "/gps_odom", and broadcasts the TF.

sector_times node reads from "/swiftnav/front/gps_pose" and "/speedsteer". The sector points are given via parameters as Latitude,Longitude pairs in launch. It also takes distance parameters in launch for the detection of sector boundary crossings. This node synchronizes the speed and gps readings using message filters, detects the sectors, and computes the total sector time with the mean speed of the sector. It publishes a custom message "sector_times", containing the current sector, the time elapsed and the mean speed so far in the current sector, into the "/sector_times" topic at a 1 Hz frequency with a timer.  

launch.launch file sets the parameters, launches rviz with the configuration loaded from odometry_view.rviz file.
