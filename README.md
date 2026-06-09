# carbot_ws

The ROS2 brain for **carbot** — my self-driving RC car that learns to drive from camera
images. This is the glue that runs the whole thing live on a Jetson Orin Nano.

The learned part goes from camera **pixels to waypoints**: the model looks at the image and
predicts a short path of where the car should go next (running on the camera feed at ~4 FPS).
A **pure-pursuit controller** then follows those waypoints, turning them into steering + speed.
The serial node sends that down to the ESP32 and reads odometry back. Pose comes from a
hand-rolled EKF (`ackermann_ekf`) that dead-reckons the car — an Ackermann motion model off
the drive commands, corrected with wheel-speed odometry + IMU yaw (no GPS or map, just where
the car is relative to where it started), running at 20 Hz. That pose is what pure pursuit
follows against. There's also keyboard teleop for manual control.

**Packages:** `carbot_ackermann` (serial bridge, IMU, EKF) · `carbot_inference` (waypoint
sender + pure pursuit) · `teleop`

**Stack:** ROS2 · C++ · pure pursuit · custom EKF (dead reckoning) · Jetson Orin Nano

## Part of the carbot project

- [carbot_drivetrain](https://github.com/tomludbrook10/carbot_drivetrain) — ESP32 drivetrain firmware
- [carbot_inference](https://github.com/tomludbrook10/carbot_inference) — real-time camera→waypoints model (TensorRT)
- [carbot_action_model](https://github.com/tomludbrook10/carbot_action_model) — trains the image→action model
- [carbot_teleoperation](https://github.com/tomludbrook10/carbot_teleoperation) — remote driving + data recording
