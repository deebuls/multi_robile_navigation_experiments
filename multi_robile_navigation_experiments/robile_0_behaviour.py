#! /usr/bin/env python3
import rclpy
from robile_behaviour import Robot

def main():
    print ("init")
    rclpy.init()
    robot_namespace = 'robile_0'

    robot = Robot(node_name=robot_namespace, namespace=robot_namespace)
    #robot.execute_navigation()
    print ("done")


if __name__ == '__main__':
    main()

