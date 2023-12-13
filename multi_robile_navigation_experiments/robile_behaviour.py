from __future__ import annotations
from abc import ABC, abstractmethod
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from robot_navigator import BasicNavigator, TaskResult

from std_msgs.msg import Int32, String, Bool

import time #remove

class RobotContext:
    """
    The RobotContext defines the interfaces to the clients.
    It also maintains the current state of the Robot
    """
    _state = None
    _node = None
    _navigator = None
    """
    reference of the current state
    """

    def __init__(self, state:State, navigator:BasicNavigator, node:Node) -> None:
        self.transition_to(state, navigator, node)


    def transition_to(self, state:State, navigator:BasicNavigator, node:Node) -> None:
        """
        The Context allows changing the State object at runtime
        """

        print (f"RobotContext: Transition to {type(state).__name__}")
        self._state = state
        self._state.context = self 
        navigator.cancelTask()
        self.execute_behaviour(navigator, node)

    def execute_behaviour(self, navigator, node) -> None:
        self._state.execute_behaviour(navigator, node)

class State(ABC):
    """
    The base state class declares methods that both the leader and follower
    robot should implement 
    """
    LEADER=1
    FOLLOWER=0

    @property
    def context(self) -> Context:
        return self._context

    @context.setter
    def context(self, context:Context) -> None:
        self._context = context

    @abstractmethod
    def execute_behaviour(self, navigator, node) -> None:
        pass

    def sendEDDImessage(self, leader_or_follower):

        msg = Int32()
        msg.data = leader_or_follower # for leader state send 1
        self.node.eddi_robot_state_pub.publish(msg)

        msg = Bool()
        msg.data = self.node.camera_working
        self.node.eddi_camera.publish(msg)
        
        msg.data = self.node.lidar_working
        self.node.eddi_lidar.publish(msg)

        msg.data = self.node.navigation_working
        self.node.eddi_navigation_robile_0.publish(msg)
        self.node.eddi_navigation_robile_1.publish(msg)

class FollowerState(State):
    def _initSubscibersPublishers(self, node):
        #publisher to publish goal update to follower based on follower namespace
        self.follower_pose_pub = node.create_publisher(PoseStamped,
                                                      node.namespace+'/goal_update', #TODO change this to get the follower name
                                                      10)
        self.leader_goal_upate = None

        self.leader_goal_sub = node.create_subscription(PoseStamped,
                                                        '/follower'+'/goal_update', #adding / in beginning doesnot add namespace
                                                        self.leaderPoseGoalCallback,
                                                        10)
        
    def leaderPoseGoalCallback(self, msg):
        print ("FollowerState: received message from leader forwarding  to ", self.node.namespace)
        #changing the frame_id to self namespace and publishing to self goal update
        msg.header.frame_id = self.node.namespace+'/map'
        self.follower_pose_pub.publish(msg)
        print ("FollowerState : forwarding pose : ", self.node.namespace, msg.pose.position.x, msg.pose.position.y)

    def amclPoseCallback(self, msg):
        self.get_logger().debug('Received amcl pose')
        self.robot_amcl_pose_with_covariance = msg
        return

    def stopSubscription(self):
        self.node.destroy_subscription(self.leader_goal_sub)
        return



    def execute_behaviour(self, navigator, node):
        self.node = node
        self._initSubscibersPublishers(node)

        self.sendEDDImessage(leader_or_follower=State.FOLLOWER)
        # set our demo's goal poses to follow
        goal_poses = []
        goal_pose1 = PoseStamped()
        goal_pose1.header.frame_id = node.namespace+'/map'
        goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose1.pose.position.x = 0.0  
        goal_pose1.pose.position.y = 3.08
        goal_pose1.pose.orientation.w = 0.707
        goal_pose1.pose.orientation.z = 0.707
        goal_poses.append(goal_pose1)
        
        nav_start = navigator.get_clock().now()
        #navigator.followWaypoints(goal_poses)
        navigator.goToPose(goal_pose1, behavior_tree='/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/follow_point.xml')
        #navigator.goToPose(goal_pose1)

        i = 0
        while not navigator.isTaskComplete():
            node.spin_once()
            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                now = navigator.get_clock().now()

                print('FollowerState: Executing current pose: ' +
                        'Time : ' + str( now - nav_start))

                self.sendEDDImessage(leader_or_follower=State.FOLLOWER)
                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=1200.0):
                    print ("FollowerState : Timeout")
                    self.stopSubscription()
                    navigator.cancelTask()
                    #self.transition_to(LeaderState(), navigator, node)
                if node.eddi_adaptation_msg == 'follower_takeover':
                    print ("FollowerState : Takeover message ##")
                    self.stopSubscription()
                    #navigator.cancelTask()
                    print ("FollowerState : Next will transition to LeaderState")
                    self._context.transition_to(LeaderState(), navigator, node)
                    #self.transition_to(LeaderState(), navigator, node)


                # If some message from the EDDI then change state to leader TODO
                #if self.change_to_leader_transition:
                #    navigator.cancelTask()
                #    print ("FollowerState : Next will transition to LeaderState")
                #    self.transition_to(LeaderState(), navigator, node)
                    


        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('FollowerState: Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('FollowerState: Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('FollowerState: Goal failed!')
        else:
            print('FollowerState: Goal has an invalid return status!')

class WaitingForLeaderState(State):

    def execute_behaviour(self, navigator, node):
        self.node = node

        self.sendEDDImessage(State.FOLLOWER)
        nav_start = navigator.get_clock().now()
        now = navigator.get_clock().now()
        # Some navigation timeout to demo cancellation
        while not now - nav_start > Duration(seconds=1200.0):
            self.sendEDDImessage(State.FOLLOWER)
            print ("WaitingForLeaderState: Sleeping now")
            time.sleep(1)

        print ("WaitingForLeaderState : Next will transition to Follower")
        self._context.transition_to(FollowerState(), navigator, node)
        
class LeaderState(State):
    def _initSubscibersPublishers(self, node):
        #publisher to publish goal update to follower based on follower namespace
        self.follower_pose_pub = node.create_publisher(PoseStamped,
                                                      '/follower'+'/goal_update',  #adding / in beginning doesnot add namespace
                                                      10)

        self.leader_goal_upate = None


    def publishFollowerUpdateGoal(self, node, timestamp):
        if node.robot_amcl_pose_with_covariance:
            goal_pose1 = PoseStamped()
            goal_pose1.header.frame_id = 'robile_1'+'/map'
            goal_pose1.header.stamp = timestamp
            goal_pose1.pose = node.robot_amcl_pose_with_covariance.pose.pose

            self.follower_pose_pub.publish(goal_pose1)
            print ("Publishing pose : ", goal_pose1.pose.position.x, goal_pose1.pose.position.y)

        return


    def execute_behaviour(self, navigator, node):
        self.node = node
        self._initSubscibersPublishers(node)
        # Wait for navigation to fully activate, since autostarting nav2
        navigator.waitUntilNav2Active()
        self.publishFollowerUpdateGoal(node, navigator.get_clock().now().to_msg())

        # set our demo's goal poses to follow
        goal_poses = []
        goal_pose1 = PoseStamped()
        goal_pose1.header.frame_id = node.namespace+'/map'
        goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose1.pose.position.x = 0.0  
        goal_pose1.pose.position.y = 3.08
        goal_pose1.pose.orientation.w = 0.707
        goal_pose1.pose.orientation.z = 0.707
        goal_poses.append(goal_pose1)

        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(goal_poses)

        i = 0
        while not navigator.isTaskComplete():
            node.spin_once()
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 10 == 0:
                now = navigator.get_clock().now()

                print('LeaderState: Executing current waypoint: ' +
                        str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)) + 
                        'Time : ' + str( now - nav_start))
                self.publishFollowerUpdateGoal(node, navigator.get_clock().now().to_msg())

                self.sendEDDImessage(State.LEADER)
                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=120.0):
                    if node.eddi_adaptation_msg == 'follower_takeover':
                        pass
                    else:
                        node.navigation_working = False
                        #TODO move eddi messages to state class and call in all while loops
                        self.sendEDDImessage(State.LEADER)
                        self._context.transition_to(WaitingForLeaderState(), navigator, node)
                        navigator.cancelTask()

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('LeaderState: Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('LeaderState: Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('LeaderState: Goal failed!')
        else:
            print('LeaderState: Goal has an invalid return status!')
            
        pass



class Robot(Node):

    ''' Class for both robot '''
    def __init__(self, node_name='', namespace='', robot_state='Leader'):
        super().__init__(node_name=node_name, namespace=namespace)
        self.namespace = namespace
        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        self.robot_amcl_pose_with_covariance = None
        self.eddi_adaptation_msg = None

        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self.amclPoseCallback,
                                                              amcl_pose_qos)
        self.eddi_robot_state_pub = self.create_publisher(Int32,
                '/'+self.namespace+'/robile_position', 10)
        self.eddi_camera = self.create_publisher(Bool,
                '/'+self.namespace+'/consert/camera', 1)
        self.eddi_lidar = self.create_publisher(Bool,
                '/'+self.namespace+'/consert/lidar', 1)
        self.eddi_navigation_robile_0 = self.create_publisher(Bool,
                '/robile_0/consert/navigation', 1)
        self.eddi_navigation_robile_1 = self.create_publisher(Bool,
                '/robile_1/consert/navigation', 1)
        #TODO when the problem of adapatation message is fixed
        #remove above two and make it single
        #self.eddi_navigation = self.create_publisher(Bool,
        #        '/'+self.namespace'/consert/navigation', 1)

        self.camera_working = True
        self.lidar_working = True
        self.navigation_working = True

        self.eddi_robot_adaptation_sub = self.create_subscription(String, '/'+self.namespace+'/adaptation', self.eddi_adaptation_callback, 10)
        self.spin_once()
        self.navigator = BasicNavigator(namespace=namespace)
        if robot_state == 'Leader':
            self.robot_context = RobotContext(LeaderState(), self.navigator, self)
        else:
            self.robot_context = RobotContext(FollowerState(), self.navigator, self)

        self.waitForInitialPose()
        #TODO subscribe to all the  topics of eddi based on node name 


    def eddi_adaptation_callback(self, msg): 
        self.get_logger().info("Recevied adapation msg : %s" % msg.data)
        self.eddi_adaptation_msg = msg.data

    def amclPoseCallback(self, msg):
        self.get_logger().debug('Received amcl pose')
        self.robot_amcl_pose_with_covariance = msg
        return

    def waitForInitialPose(self):
        while not self.robot_amcl_pose_with_covariance:
            self.get_logger().info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
        return

    def spin_once(self,timeout_sec:float = 1.0) -> None:
        rclpy.spin_once(self, timeout_sec=timeout_sec)
        return

    def execute_navigation(self):
        self.robot_context.execute_behaviour(self.navigator, self)

