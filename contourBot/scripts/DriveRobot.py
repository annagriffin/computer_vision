#!/usr/bin/env python3
"""
Class that takes a set of waypoints and have robot flow the waypoints
"""

import rospy
import math

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Twist, Pose, PoseStamped, Point
from visualization_msgs.msg import Marker

from tf import TransformListener
from tf import TransformBroadcaster
from helper_functions import TFHelper


class DriveRobot(object):
    def __init__(self, waypoints):
        rospy.init_node('contourBot')

        # parameters related to waypoints
        self.waypoints = waypoints
        self.reach_first_point = False
        self.angle_offset = 0.1
        self.distance_offset = 0.01

        # current position of robot
        self.odom_pose = None
        self.cur_pose_x = 0
        self.cur_pose_y = 0
        self.cur_pose_yaw = 0

        # next waypoint robot tries to go to
        self.waypoint_pose_x = 0
        self.waypoint_pose_y = 0
        self.waypoint_pose_theta = 0
        self.finish_trace = False

        # robot motion parameters
        self.angle_diff = math.inf
        self.distance_to_waypoint = math.inf
        self.velocity = Twist()
        self.k_linear_vel = 0.2
        self.k_angular_vel = 0.2
        self.update_interval = rospy.Duration(0.1)
        self.last_update_time = rospy.Time(0)

        # enable listening for and broadcasting coordinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()
        self.transform_helper = TFHelper()

        # coordinate frames
        self.base_frame = "base_link"
        self.odom_frame = "odom"

        # set up pubs and subs
        # rospy.Subscriber("/odom", Pose, self.update_robot_pose)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.waypoints_marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # visualization parameters
        self.waypoints_marker = Marker()
        self.waypoint_marker_line_color = ColorRGBA()
        self.waypoint_marker_line_color.r = 0
        self.waypoint_marker_line_color.g = 1.0
        self.waypoint_marker_line_color.b = 0
        self.waypoint_marker_line_color.a = 1.0

    def initialize_waypoints_as_markers(self):
        self.waypoints_marker.id = 0
        self.waypoints_marker.header.frame_id = self.odom_frame
        self.waypoints_marker.type = Marker.LINE_STRIP
        self.waypoints_marker.ns = 'Waypoints'
        self.waypoints_marker.action = 0
        self.waypoints_marker.scale.x = 0.05
        self.waypoints_marker.scale.y = 0.05
        self.waypoints_marker.scale.z = 0.05
        for waypoint in self.waypoints:
            new_point = Point()
            new_point.x = waypoint[0]
            new_point.y = -waypoint[1]
            new_point.z = 0   # neato is spawned at z=1.5 in gazebo
            self.waypoints_marker.points.append(new_point)
            self.waypoints_marker.colors.append(self.waypoint_marker_line_color)

    def update_robot_pose(self):
        current_time = rospy.Time(0)
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, current_time, rospy.Duration(0.5))
        if not (self.tf_listener.canTransform(self.base_frame, self.odom_frame, current_time)):
            # need to know how to transform between base and odometry frames
            # this will eventually be published by either Gazebo or neato_node
            return

        # find out where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=current_time,frame_id=self.base_frame),
                        pose=Pose())
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
        # store the the odometry pose in a more convenient format (x,y,theta)
        current_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        self.cur_pose_x = current_odom_xy_theta[0]
        self.cur_pose_y = current_odom_xy_theta[1]
        self.cur_pose_yaw = current_odom_xy_theta[2]

    def reach_waypoint_direction(self):
        self.angle_diff = self.waypoint_pose_theta - self.cur_pose_yaw
        return abs(self.angle_diff) <= self.angle_offset

    def reach_waypoint(self):
        self.distance_to_waypoint = math.sqrt((self.waypoint_pose_x - self.cur_pose_x) ** 2 + (
                self.waypoint_pose_y - self.cur_pose_y) ** 2)
        return abs(self.distance_to_waypoint) <= self.distance_offset

    def drive_robot(self):
        while not self.reach_waypoint_direction():
            self.velocity.angular.z = self.k_angular_vel * self.angle_diff
            print("update robot angular velocity: ", self.velocity.angular.z)
            self.vel_pub.publish(self.velocity)
            self.update_robot_pose()
        self.velocity.angular.z = 0
        self.vel_pub.publish(self.velocity)

        while not self.reach_waypoint():
            self.velocity.linear.x = self.k_linear_vel * self.distance_to_waypoint
            print("update robot linear velocity: ", self.velocity.linear.x)
            self.vel_pub.publish(self.velocity)
            self.update_robot_pose()
        self.velocity.angular.x = 0
        self.vel_pub.publish(self.velocity)

    def follow_waypoint(self):
        for waypoint in self.waypoints:
            print("current waypoint: ", waypoint)
            self.waypoint_pose_x = waypoint[0]
            self.waypoint_pose_y = -waypoint[1]  # in picture frame down is positive y
            self.waypoint_pose_theta = math.atan2(self.waypoint_pose_y, self.waypoint_pose_y)
            time_offset = rospy.Time.now() - self.last_update_time
            if time_offset > self.update_interval:
                self.drive_robot()
        self.finish_trace = True

    def run(self):
        self.initialize_waypoints_as_markers()

        r = rospy.Rate(5)
        while not(rospy.is_shutdown()):
            self.waypoints_marker_pub.publish(self.waypoints_marker)
            print("Contour waypoint visualization published")
            if not self.finish_trace:
                self.follow_waypoint()
                pass
            r.sleep()


if __name__ == '__main__':
    contour_waypoints = [(3, 3),
                 (4, 3),
                 (5, 3),
                 (6, 4),
                 (7, 5),
                 (6, 6),
                 (5, 7),
                 (4, 7),
                 (3, 7),
                 (2, 6),
                 (1, 5),
                 (2, 4),
                 (3, 3)]
    bot = DriveRobot(contour_waypoints)
    bot.run()


