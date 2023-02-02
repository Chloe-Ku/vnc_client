import rosbag
import rospy
import tf
import math
import os.path
from os import path
from custom_msgs.msg import *
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
# Create absolute path to this file
current_path = os.path.abspath(os.path.dirname(__file__))

def write_bag_file(name, drive_data, odom_data):
    '''drive_data: the DriveInfo.msg data in the form of a JSON object
       format: 
		drive_data = {
				"power_on" = "False",
                                "throttle_right" : 0.0,
                                "throttle_left" : 0.0,
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
                                "time" : rostime
			    }

       odom_data: the Odom.msg data in the form of a JSON object
       format:
		odom_data = {
                                "odom.pose.pose.position.x" : 0.0
                                "odom.pose.pose.position.y" : 0.0
                                "odom.pose.pose.position.z" : 0.0
                                "odom.twist.twist.linear.x" : 0.0
                                "odom.twist.twist.linear.y" : 0.0
                                "odom.twist.twist.angular.z" : 0.0
                                "steering" : 0.0
                                "heading_angle" : 0.0
                                "time" : rostime
			    }

	Note: decided to use JSON as importing the custom .msg files can
	add unneeded difficulty
    '''
    # Create the new output .bag file
    writeBag = rosbag.Bag(os.path.join(current_path, '../bags/', name), 'w')
    # Append to 'testOut.bag' if there is an existing .bag file
    # A new bag file should most likely be created for each path
    # But this is how appending can be accomplished
    # if path.exists(os.path.join(current_path, '../bags/test.bag')):
    #    print('Appending to file')
    #    readBag = rosbag.Bag(os.path.join(current_path, '../bags/test.bag'))

    #    try:
    #        # Write each message from the existing file
    #        for topic, msg, t in readBag.read_messages(topics=['DriveInfo']):
    #            writeBag.write('DriveInfo', msg)
    #        for topic, msg, t in readBag.read_messages(topics=['Odom']):
    #            writeBag.write('Odom', msg)
    #    finally:
    #        print('finished appending')
    #        readBag.close()

    Path_msg = Path()
    Path_msg.header.frame_id = "/map"
    # Path_msg.header.stamp = rospy.Time.now()

    try:
        for drive_entry in drive_data:
                # Set the DriveInfo message data
                # Dummy data is currently being passed in for these values
                DriveCom_msg = DriveInfo()
                DriveCom_msg.power_on = bool(drive_entry["power_on"])
                DriveCom_msg.throttle_right = drive_entry["throttle_right"]
                DriveCom_msg.throttle_left = drive_entry["throttle_left"]
                DriveCom_msg.brake_front_right = drive_entry["brake_front_right"]
                DriveCom_msg.brake_front_left = drive_entry["brake_front_right"]
                DriveCom_msg.brake_rear_right = drive_entry["brake_rear_right"]
                DriveCom_msg.brake_rear_left = drive_entry["brake_rear_left"]
                DriveCom_msg.steering_left = drive_entry["steering_left"]
                DriveCom_msg.steering_right = drive_entry["steering_right"]
                DriveCom_msg.wheel_speed_front_left = drive_entry["wheel_speed_front_left"]
                DriveCom_msg.wheel_speed_front_right = drive_entry["wheel_speed_front_right"]
                DriveCom_msg.wheel_speed_rear_left = drive_entry["wheel_speed_rear_left"]
                DriveCom_msg.wheel_speed_rear_right = drive_entry["wheel_speed_rear_right"]
                DriveCom_msg.header.stamp = rospy.Time.from_sec(drive_entry["time"])
                # Write the data
                writeBag.write('DriveInfo', DriveCom_msg)

        for odom_entry in odom_data:
                # Set the Odometry data
                Odom_msg = VehicleOdomInfo()
                # Data generated by /gps_to_odom/src/coord_to_odom.py
                Odom_msg.odom.pose.pose.position.x = odom_entry["odom.pose.pose.position.x"]
                Odom_msg.odom.pose.pose.position.y = odom_entry["odom.pose.pose.position.y"]
                Odom_msg.odom.pose.pose.position.z = odom_entry["odom.pose.pose.position.z"]
                Odom_msg.odom.pose.pose.orientation.z = odom_entry["odom.pose.pose.orientation.z"]
                Odom_msg.odom.pose.pose.orientation.w = odom_entry["odom.pose.pose.orientation.w"]
                Odom_msg.odom.twist.twist.linear.x = odom_entry["odom.twist.twist.linear.x"]
                Odom_msg.odom.twist.twist.linear.y = odom_entry["odom.twist.twist.linear.y"]
                Odom_msg.odom.twist.twist.angular.z = odom_entry["odom.twist.twist.angular.z"]
                Odom_msg.steering = odom_entry["steering"]
                Odom_msg.heading_angle = odom_entry["heading_angle"]
                Odom_msg.header.stamp = rospy.Time.from_sec(odom_entry["time"])
                # The frame ids can be written this way if needed
                # this would require adding the child_frame_id to the custom Odom.msg
                # Odom_msg.header.frame_id = odom_entry["frame_id"]
                # Odom_msg.child_frame_id = odom_entry["child_frame_id"]
                # Write the data
                writeBag.write('Odom', Odom_msg)

                pose = PoseStamped()

                pose.pose.orientation.w = 1.0
                pose.pose.position.x = odom_entry["odom.pose.pose.position.x"]
                pose.pose.position.y = odom_entry["odom.pose.pose.position.y"]
                pose.pose.position.z = odom_entry["odom.pose.pose.position.z"]
                pose.pose.orientation.x = 0
                pose.pose.orientation.y = 0
                pose.pose.orientation.z = odom_entry["odom.pose.pose.orientation.z"]
                pose.pose.orientation.w = odom_entry["odom.pose.pose.orientation.w"]

                Path_msg.poses.append(pose)

    finally:
        writeBag.close()

    print("Hello")

    #Open up a second bag file to write our MarkersArray to 
    markerBag = rosbag.Bag('/home/chloe/record_for_repay.bag', 'w')

    markerBag.write('/ecoprt/path/auto', Path_msg)
    markerBag.close()  


