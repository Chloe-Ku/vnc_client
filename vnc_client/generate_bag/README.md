## Description

This ROS module receives the odometry data from the `gps_to_odom` module located on the vehicle and writes it to a ROS-readable bag file the vehicle will use to navigate along the route in the `bags` folder.

The `bags` folder also contains files used for testing, as well as a script used to convert bag files to csv so that they can be read and tested more easily.

## Testing

Tests for this module can be found in the `/test/test_generate_bag/` directory of this repository.
