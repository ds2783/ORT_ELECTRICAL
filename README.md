## Olympus Rover Trials 2024/2025 submission repository

Git-hosted ROS2 code for the ORT rover, includes packages for the base station controls as well. 

Hardware includes:
- IMU: ICM20948
- Time of Flight sensors: VL53L4CX
- GPS: Adafruit Ultimate GPS w/o USB-C
- Optical Flow sensor: PMW3901 
- Battery Fuel Gauge: INA260
- PWM Board: ?

The system uses a custom GUI developed in OpenGL using python, with a custom video server streaming both UDP video feed, and sending images on request via TCP.

Position estimation is handled using sensor fusion of two ToF sensors, an IMU, an OFS and a GPS.
- The OFS sensor is used to measure distance travelled relative to the orientation of the rover.
- The IMU is used to find the orientation of the rover, upon which we can rotate the distance travelled around the Yaw of the Rover.
- The Time of Flight sensors are used to measure distances, and calibrate the OFS sensor for different materials and lighting conditions. 
(This is done by using the preprogrammed calibration routines.)
- The GPS was used as a backup, to check if our position estimation is accurate or subject to drift.

The ROS2 package is run on Raspbian via a Docker container. With all the networking done over a standard WiFi router, using 2.4Ghz frequency, as it is less prone to attenuation.
