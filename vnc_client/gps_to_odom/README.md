## Description

This ROS module receives local coordinate positions from the VNC and generates odometry data the vehicle will use in following the path. The folowing variables are generated based on the provided xy waypoint positions: `heading_angle`, `steering`, `orientation (quaternion z and w)`, `linear velocity (x and y)`, and `angular velocity (z)`.

Once this data has been generated, the `generate_bag` ROS module is called in order to store the data in a ROS-readable bag file on the vehicle.

Information regarding how the variables are generated can be found in the implementation section of our Final Project Report.

## Testing

Tests for this module can be found in the `/test/test_gps_to_odom/` directory of this repository.
