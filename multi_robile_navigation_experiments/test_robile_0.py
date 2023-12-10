#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
#from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

"""
Basic navigation demo to go to poses.
"""

class LeaderRobot(Node):
    ''' Class for leader robot '''
    def __init__(self, node_name='leader_robot', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        self.namespace = namespace
        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        self.robot_amcl_pose_with_covariance = None

        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self.amclPoseCallback,
                                                              amcl_pose_qos)
        self.follower_pose_pub = self.create_publisher(PoseStamped,
                                                      '/robile_1/goal_update', #TODO change this to get the follower name
                                                      10)
        

    def amclPoseCallback(self, msg):
        self.get_logger().debug('Received amcl pose')
        self.robot_amcl_pose_with_covariance = msg
        return

    def publishFollowerUpdateGoal(self, timestamp):
        
        if self.robot_amcl_pose_with_covariance:
            goal_pose1 = PoseStamped()
            goal_pose1.header.frame_id = 'robile_1'+'/map'
            goal_pose1.header.stamp = timestamp
            goal_pose1.pose = self.robot_amcl_pose_with_covariance.pose.pose

            self.follower_pose_pub.publish(goal_pose1)
            print ("Publishing pose : ", goal_pose1.pose.position.x, goal_pose1.pose.position.y)

            return

    def _waitForInitialPose(self):
        while not self.robot_amcl_pose_with_covariance:
            self.get_logger().info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
        return

    def spin_once(self,timeout_sec=1.0):
        rclpy.spin_once(self, timeout_sec=timeout_sec)
        return

    

def main():
    rclpy.init()
    robot_namespace = 'robile_0'

    leader = LeaderRobot(namespace=robot_namespace)
    leader._waitForInitialPose()
    navigator = BasicNavigator(namespace=robot_namespace)
    #navigator = BasicNavigator()

    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 3.45
    # initial_pose.pose.position.y = 2.15
    # initial_pose.pose.orientation.z = 1.0
    # initial_pose.pose.orientation.w = 0.0
    # navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # set our demo's goal poses to follow
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = robot_namespace+'/map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 0.0  
    goal_pose1.pose.position.y = 3.08
    goal_pose1.pose.orientation.w = 0.707
    goal_pose1.pose.orientation.z = 0.707
    goal_poses.append(goal_pose1)

    # additional goals can be appended
    # goal_pose2 = PoseStamped()
    # goal_pose2.header.frame_id = robot_namespace+'/map'
    # goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose2.pose.position.x = 1.5
    # goal_pose2.pose.position.y = -3.75
    # goal_pose2.pose.orientation.w = 0.707
    # goal_pose2.pose.orientation.z = 0.707
    # goal_poses.append(goal_pose2)
    # goal_pose3 = PoseStamped()
    # goal_pose3.header.frame_id = robot_namespace+'/map'
    # goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose3.pose.position.x = -3.6
    # goal_pose3.pose.position.y = -4.75
    # goal_pose3.pose.orientation.w = 0.707
    # goal_pose3.pose.orientation.z = 0.707
    # goal_poses.append(goal_pose3)

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose1)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################
        leader.spin_once()
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            now = navigator.get_clock().now()

            print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)) + 
                    'Time : ' + str( now - nav_start))
            leader.publishFollowerUpdateGoal(navigator.get_clock().now().to_msg())



            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=1200.0):
                navigator.cancelTask()

            # Some follow waypoints request change to demo preemption
            # if now - nav_start > Duration(seconds=35.0):
            #     goal_pose4 = PoseStamped()
            #     goal_pose4.header.frame_id = 'map'
            #     goal_pose4.header.stamp = now.to_msg()
            #     goal_pose4.pose.position.x = -5.0
            #     goal_pose4.pose.position.y = -4.75
            #     goal_pose4.pose.orientation.w = 0.707
            #     goal_pose4.pose.orientation.z = 0.707
            #     goal_poses = [goal_pose4]
            #     nav_start = now
            #     navigator.followWaypoints(goal_poses)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    #navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()

