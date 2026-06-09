# carbot_ws

The ROS2 brain for **carbot** — my self-driving RC car that learns to drive from camera
images. It's end-to-end **pixels straight to robot actions**: raw camera pixels go in, motor
and steering commands come out, with no hand-coded driving rules in between. This is the glue
that runs the whole thing live on a Jetson Orin Nano.

Things I learnt building this: mapping pixels directly to actions is really, *really* hard.
(I have since developed a deep respect for anyone who does this for a living.)

The autonomous loop: the camera feeds the trained model, which spits out waypoints; a
pure-pursuit controller turns those into steering + speed; the serial node sends that down
to the ESP32 and reads odometry back. Pose comes from a hand-rolled EKF (`ackermann_ekf`)
that dead-reckons the car — running an Ackermann motion model off the drive commands and
correcting with wheel-speed odometry + IMU yaw (no GPS or map, just tracking where the car
is relative to where it started). That pose is what pure pursuit follows. There's also
keyboard teleop for manual control.

**Packages:** `carbot_ackermann` (serial bridge, IMU, EKF) · `carbot_inference` (waypoint
sender + pure pursuit) · `teleop`

**Stack:** ROS2 · C++ · pure pursuit · custom EKF (dead reckoning) · Jetson Orin Nano

## Part of the carbot project

- [carbot_drivetrain](https://github.com/tomludbrook10/carbot_drivetrain) — ESP32 drivetrain firmware
- [carbot_inference](https://github.com/tomludbrook10/carbot_inference) — real-time camera→waypoints model (TensorRT)
- [carbot_action_model](https://github.com/tomludbrook10/carbot_action_model) — trains the image→action model
- [carbot_teleoperation](https://github.com/tomludbrook10/carbot_teleoperation) — remote driving + data recording
