#! /usr/bin/env python3
import rclpy
from robile_behaviour import Robot, RobotContext, FollowerState

def main():
    print ("init")
    rclpy.init()
    robot_namespace = 'robile_1'

    robot = Robot(node_name='robile_1_namespace', namespace=robot_namespace, robot_state='Follower')
    robot.robot_context = RobotContext(FollowerState(), robot.navigator, robot)
    #robot.execute_navigation()
    print ("done")


if __name__ == '__main__':
    main()

