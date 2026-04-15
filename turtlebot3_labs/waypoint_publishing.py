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
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from .utils.new_coordinates import WaypointGenerator as wp
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import tf2_ros
from rclpy.node import Node
pkg_path = get_package_share_directory('turtlebot3_labs')
file_path = os.path.join(pkg_path, 'utils', 'room_waypoints.yaml')
import math
"""
Basic navigation demo to go to pose.
"""

class waypoint_sender(Node):
    def __init__(self):
     super().__init__('waypoint_sender_node')
     self.navigator = BasicNavigator()
     #print(self.navigator.get_parameters())
     self.initial_pose = PoseStamped()
     self.target_room = "default"
     self.width = 0.3
     self. current_room = "defaut"
     self.entry_point = []
     self.current_room_coords = []
     self.num_wp = 0
     self.transform = []
     self.wp_index = 0
     self.path  = []
     self.distance = 0.0
     self.buffer  = tf2_ros.Buffer()
     self.tf = tf2_ros.TransformListener(self.buffer,self)
     self.waypoints = [] #array of waypoints 
     with open(file_path, 'r') as f: #creating the variable storing the map corners and room points 
      raw_corners = yaml.load(f, Loader=yaml.FullLoader)
      self.corner_list = raw_corners['rooms']
  

    def compute_relative_tf(self):
       self.get_logger().info("attempting to retrieve transform")
       start_time = self.get_clock().now()
       while (self.get_clock().now() - start_time).nanoseconds < 1e9:
               rclpy.spin_once(self, timeout_sec=0.1)
       is_available = self.buffer.wait_for_transform_async('odom','base_link',rclpy.time.Time())
       self.transform = self.buffer.lookup_transform('odom','base_link',rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=5.0))

    def do_path_length(self):
        #calculating the path length before move,
        #  since feedback only publishes once we actually send the waypoint
       d = 0.0
       for i in range(len(self.path.poses)-1):
          a = self.path.poses[i].pose.position
          b = self.path.poses[i+1].pose.position
          d += math.sqrt(pow((b.x -a.x),2) +pow((b.y-a.y),2))
       return  d
    

    def request_room(self):
     gen = wp()
     while True:
       try:
        user_room = str(input("please specify a room for input, or type EXIT to quit: ").strip())
       except ValueError: 
          print("incorrect room name, try again")
       else:
          break
       if user_room == 'EXIT':
        return False 
     self.target_room = user_room

     self.current_room_coords = self.corner_list[user_room]['corners']
      #returns array of corners for dict entry at given input
     print("current room corners : ",self.current_room_coords)
     self.entry_point = self.corner_list[user_room]['entry_point']

     width = self.width
     coords = self.current_room_coords
     self.compute_relative_tf()
     self.waypoints = gen.generate_waypoints(coords,width,self.entry_point) 
     self.num_wp = len(self.waypoints)

     print(self.waypoints)

     return True




    def array_to_pose(self,array): 
       #need to switch python array wp into ros2 me
       output = PoseStamped()
       output.header.stamp = self.navigator.get_clock().now().to_msg()
       output.header.frame_id = 'map'
       output.pose.position.x = float(array[0])
       output.pose.position.y = float(array[1])
       output.pose.orientation.w = 1.0
       return output
    def send_start_pose(self):
        initial_pose = self.initial_pose
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.003756121266633272
        initial_pose.pose.position.y =  -0.0051910472102463245
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = 0.0007728559666214475
        initial_pose.pose.orientation.w = 0.9999997013467828
        self.initial_pose = initial_pose
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

    def send_safe_pose(self):
        pose = self.initial_pose
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = 3.3
        pose.pose.position.y = -0.8
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        
        self.navigator.goToPose(pose)
        
    def get_current_pose(self): #had to add this since it's not natively part of the simple navigator we're using 
     try:
   
         t = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
         
         current_pose = PoseStamped()
         current_pose.header.frame_id = 'map'
         current_pose.header.stamp = self.get_clock().now().to_msg()
         current_pose.pose.position.x = t.transform.translation.x
         current_pose.pose.position.y = t.transform.translation.y
         current_pose.pose.position.z = t.transform.translation.z
         current_pose.pose.orientation = t.transform.rotation
         return current_pose
     except Exception as e:
         self.get_logger().error(f"Could not get robot pose: {e}")
         return None    

  

    def send_waypoint(self,waypoint):
        goal_pose = self.array_to_pose(waypoint)
        print(waypoint)
        current_pose = self.get_current_pose() 
        try: 
         path = self.navigator.getPath(current_pose, goal_pose)
         self.path = path
         if path:
         
          actual_length = self.do_path_length()

          x = goal_pose.pose.position.x - current_pose.pose.position.x
          y = goal_pose.pose.position.y - current_pose.pose.position.y
          straight_length = math.sqrt(pow(x,2)+ pow(y,2))
          
          difference = abs(actual_length-straight_length) #magnitude of the difference between the two, we dont care about sign 
          #we disregard the sign as the actual length will always be longer than or equal to straight length. 
          print("difference : ",difference)  
         # if difference > 0.5:
          #   temp_waypooins = self.
        except: 
              print("no pathable point")
              pass
        go_to_pose_task = self.navigator.goToPose(goal_pose)
        
        i = 0
        while not self.navigator.isTaskComplete():

         i =i + 1
         feedback = self.navigator.getFeedback()
         if feedback and i % 5 == 0:
          print(
              'Estimated time of arrival: '
              + '{:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                  / 1e9
              )
              + ' seconds.'
          )

          print(
              'Distance remaining: '
              + '{:.2f}'.format(feedback.distance_remaining)
              + ' meters.'
          )

          if Duration.from_msg(feedback.navigation_time) > Duration(seconds=15.0):
              self.navigator.cancelTask()
              break
          if feedback.distance_remaining <=0.35:
             break
          result = self.navigator.getResult()
          if result == TaskResult.SUCCEEDED:
              print('Goal succeeded!')
          elif result == TaskResult.CANCELED:
              print('Goal was canceled!')
          elif result == TaskResult.FAILED:
            
              print('Goal failed!')
          else:
              print('Goal has an invalid return status!')

        i = 0
    def navigate_room(self):
     while True:
       if not self.request_room():
          return False
       for i in range(self.num_wp):
          self.send_waypoint(self.waypoints[i])
       
def main() -> None:
    rclpy.init()
    
    publisher = waypoint_sender()
    publisher.send_start_pose()
    publisher.send_safe_pose()
    publisher.navigate_room()

    publisher.navigator.lifecycleShutdown()

    
    exit(0)


if __name__ == '__main__':
    main()