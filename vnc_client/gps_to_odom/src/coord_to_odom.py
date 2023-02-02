# tf.transformations alternative is not yet available in tf2
import time
import rospy
import json
from tf.transformations import *
import math
import numpy as np
from scipy.interpolate import splprep
from scipy.interpolate import splev
from scipy.misc import derivative
import matplotlib.pyplot as plt
import sys
import os.path
from os import path
# sys.path.insert(1, os.path.join(sys.path[0], '../../generate_bag/scripts'))
from write_bag import write_bag_file

# Time
start_time = time.time()
# Wheelbase of the vehicle (meters)
wheelbase = 1.7526
# The maximum allowable radial acceleration of the vehicle. Currently arbitrary
max_radial_accleration = .5

def within_threshold(x1, x2, y1, y2):
    """
    Calculates the percent difference between the points. A fraction of a
    percent can be used for greater accuracy. However, this requires smaller
    intervals and much longer runtime.
    Returns true if the points are within 1%
    """
    pd_x = abs(x2 - x1) / abs(x2)
    pd_y = abs(y2 - y1) / abs(y2)
    return True if pd_x < 0.05 and pd_y < 0.05 else False

def file_exists(coordinates):
    """
    Checks to see if a file with the given starting and ending point already
    exists. Files are named as the first 5 characters of their starting coordinates
    followed by the first 5 characters fo their ending coordinates. If a file with
    the same name exists, the conversion module does not need to run.
    Name example: 1.234_1.234_to_2.345_2.345.bag
    Returns "exists" if the path exists, else the name of the new bag file
    """
    # Get starting and ending points
    start_point = str(coordinates[0]["position"]["x"])[0:5] + "_" + str(coordinates[0]["position"]["y"])[0:5]
    end_point = str(coordinates[len(coordinates) - 1]["position"]["x"])[0:5] + "_" + str(coordinates[len(coordinates) - 1]["position"]["y"])[0:5]
    name = start_point + "_to_" + end_point + ".bag"
    # Construct the relative path
    current_path = os.path.abspath(os.path.dirname(__file__))
    if path.exists(os.path.join(current_path, "../../generate_bag/bags/", name)):
        return "exists"
    else:
        return name

def convert_to_odom(coordinates):
    """
    Coordinate data in the form:
    {
        "position": {
        "x": 0,
        "y": 0,
        "z": 0
    },
        "speed": 0
    }

    generates: heading_angle from arctan of the position derivatives
               orientation from the quaternion of the heading_angle
               linear and angular velocities from the orientation
               steering from ackerman steering based on heading
    """
    # Check to see if an existing file exists on the vehicle
    name = file_exists(coordinates)
    if name == "exists":
        return

    if len(coordinates) < 4:
        raise ValueError("Requires at least 4 coordinates")

    # The odometry data generated
    odom = []
    # Placeholder DriveInfo data
    drive_data = []

    print("Smoothing the polyline")
    # Split the coordinates into separate data sets
    x = np.empty(len(coordinates))
    y = np.empty(len(coordinates))
    # Parse the coordinates
    for i in range(len(coordinates)):
        x[i] = coordinates[i]['position']['x']
        y[i] = coordinates[i]['position']['y']
    
    # plt.plot(x, y, 'o')

    # Contour distance based on:
    # Smooth spline representation of an arbitrary contour
    # Author: yurez
    # Date: June 10th, 2016
    # Availability: https://stackoverflow.com/questions/14344099/smooth-spline-representation-of-an-arbitrary-contour-flength-x-y
    
    # get the cumulative distance along the contour
    dist = np.sqrt((x[:-1] - x[1:])**2 + (y[:-1] - y[1:])**2)
    dist_along = np.concatenate(([0], dist.cumsum()))

    # build a spline representation of the contour
    spline, u = splprep(np.array([x, y]), u=dist_along, s=0)

    # resample it at smaller distance intervals
    # A smaller interval increases accuracy with the tradeoff of significantly slower execution
    interp_d = np.linspace(dist_along[0], dist_along[-1], 1000 * len(coordinates))
    interp_x, interp_y = splev(interp_d, spline)

    # plt.plot(interp_x, interp_y)
    # plt.show()

    # Using interp_d (the interpolated points) results in greater precision
    # Calculate the derivates
    interp_dx, interp_dy  = splev(interp_d, spline, der=1)
    # Normalize the resulting tangent vectors (derivatives)
    N = np.linalg.norm(np.array([interp_dx, interp_dy]), axis=0)
    interp_x_diffs = interp_dx/N
    interp_y_diffs = interp_dy/N
    # Derivatives of only the given position values
    dx = np.empty(len(coordinates))
    dy = np.empty(len(coordinates))

    print("Differentiating positions")
    # Derivatives based on position, not spline
    for i in range(len(dx)):
        if i != 0 and i < len(dx) - 1:
            dx[i] = x[i + 1] - x[i - 1]
            dy[i] = y[i + 1] - y[i - 1]
        # Take the slope to the next point since we can't accurately extrapolate
        elif i == 0:
            dx[i] = x[i + 1] - x[i]
            dy[i] = y[i + 1] - y[i]
        # Take the slope from the previous since we can't accurately extrapolate
        else:
            dx[i] = x[i] - x[i - 1]
            dy[i] = y[i] - y[i - 1]

    print("Calculating the heading angles")
    # Heading angles to return
    heading_angle = np.empty(len(coordinates))
    # Since the spline does not exactly include all the points in x and y
    # we need to ensure there is an acceptable match
    i = 0
    while i < len(x):
        j = i
        matched = 0
        while j < len(interp_x):
            if within_threshold(x[i], interp_x[j], y[i], interp_y[j]):
                matched = 1
                # Calculate the heading angle as the angle of the derivative
                heading_angle[i] = math.atan2(interp_y_diffs[j], interp_x_diffs[j])
                break;
            j += 1
        # If no acceptable match is found
        if matched == 0:
            break;
        i += 1
    # Report that no match could be found; the spline is too sparse
    if i != len(coordinates):
        raise RuntimeError("Error smoothing the polyline")

    print("Calculating steering and orientation")
    # Steering angles to return
    steering = np.empty(len(coordinates))
    # z and w orientation
    orientation_z = np.empty(len(coordinates))
    orientation_w = np.empty(len(coordinates))

    #The instantaneous turning radius (I think in meters) at any given point along the path
    turning_radius = np.empty(len(coordinates))

    for i in range(len(coordinates)):
        # Perform the quaternion transformation
        q = quaternion_from_euler(0, 0, heading_angle[i])
        orientation_z[i] = q[2]
        orientation_w[i] = q[3]
        # We can look ahead 1 vertex
        if i < len(heading_angle) - 1:
            # Reciprocal slopes
            slope1 = -1 / math.tan(heading_angle[i])
            slope2 = -1 / math.tan(heading_angle[i + 1])
            # Given y = mx + b find intercepts
            b1 = y[i] - slope1 * x[i]
            b2 = y[i + 1] - slope2 * x[i + 1]
            # Solve for x, then y
            center_x = (b2 - b1) / (slope1 - slope2)
            center_y = slope1 * center_x + b1
            # Find the turn radius (distance between center and (x1, y1)
            R = math.sqrt((x[i] - center_x)**2 + (y[i] - center_y)**2)

            #Store our turning radius to use for velocity smoothing
            turning_radius[i] = R

            #This steering angle will always be positive because we're taking the atan of two positive numbers. 
            #We need to determine the sign of the steering angle based on the current heading angle of the vehicle
            steeringVal = math.atan(wheelbase/R)

            #Define a unit vector in the direction of our current heading angle
            H = [np.cos(heading_angle[i]), np.sin(heading_angle[i]), 0]
            #Define the change in position between the current position and the center. Then normalize
            #the resulting vector so that we have two unit vectors
            deltaP = [center_x - x[i], center_y - y[i], 0]
            deltaP = deltaP / np.linalg.norm(deltaP)

            #Take the cross product of our heading vector and our positional update vector. This will give us the sign of
            #the rotation that we need. If our cross product is positive, we need to turn left. If our cross product is negative
            #we need to turn right. 
            cross = np.cross(H, deltaP)

            #Normalize our cross product to just extract the sign
            sign = cross[2] / np.abs(cross[2])

            #Finally, update the sign of our steering angle based on the cross product

            steeringVal = steeringVal * sign
            

            if steeringVal < -1.0:
                steeringVal = -1.0
            elif steeringVal > 1:
                steeringVal = 1

            steering[i] = steeringVal
        # We cannot safely extrapolate
        # Maintain the same heading and subsequent steering
        else:
            steering[i] = steering[i - 1]

    print("Calculating linear and angular velocities")
    # Linear is just the x and y derivatives.

    angular_z = np.empty(len(coordinates))
    # Take the changes in the derivative of the heading angle as the angular z
    for i in range(len(coordinates)):
        previous_idx = i - 1
        next_idx = i + 1
        if i == 0:
            # Can only look ahead by one angle
            previous_idx = 0
            next_idx = 1
        elif i == len(coordinates) - 1:
            # Can only look behind by one angle
            previous_idx = i - 1
            next_idx = i
        # Differentiate
        angular_z[i] = heading_angle[next_idx] - heading_angle[previous_idx]
        # Invert if we crossed Pi radians counterclockwise
        # In the case that this large heading angle is "legitimate" the error
        # will be caught in the steering angle tests
        if angular_z[i] > math.pi:
            angular_z[i] = (heading_angle[next_idx] - 2 * math.pi) - heading_angle[previous_idx]
        # Invert if we crossed Pi radians clockwise
        elif angular_z[i] < -math.pi:
            angular_z[i] = (heading_angle[next_idx] + 2 * math.pi) - heading_angle[previous_idx]

        # All values are calculated so we can write them now
        odom.append({ "odom.pose.pose.position.x" : x[i],
             "odom.pose.pose.position.y" : y[i],
             "odom.pose.pose.position.z" : 0.0,
             "odom.pose.pose.orientation.z" : orientation_z[i],
             "odom.pose.pose.orientation.w" : orientation_w[i],

             #Linear x and linear y make up the two components of our velocity vector
             "odom.twist.twist.linear.x" : dx[i],
             "odom.twist.twist.linear.y" : dy[i],
             "odom.twist.twist.angular.z" : angular_z[i],
             "steering" : steering[i],
             "heading_angle" : heading_angle[i],
             # Half second time update for each waypoint
             "time" : start_time + (i / 2)
             # If the frame ids become necessary they can be written this way
             # this should require adding the child_frame_id to the custom Odom.msg
             # "frame_id" : "odom",
             # "child_frame_id" : "chassis_baselink"
        })
        # Dummy data for DriveInfo
        drive_data.append({
            "power_on" : "0", # Represents boolean false
            "throttle_right" : 1.0,
            "throttle_left" : 1.0,
            "brake_front_right" : 0.0,
            "brake_front_left" : 0.0,
            "brake_rear_right" : 0.0,
            "brake_rear_left" : 0.0,
            "steering_left" : 0.0,
            "steering_right" : 0.0,
            "wheel_speed_front_left" : 0.0,
            "wheel_speed_front_right" : 0.0,
            "wheel_speed_rear_left" : 0.0,
            "wheel_speed_rear_right" : 0.0,
            # Half second time update for each waypoint
            "time" : start_time + (i / 2)
        })
    write_bag_file(name, drive_data, odom)

