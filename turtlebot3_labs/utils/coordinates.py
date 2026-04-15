import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Pose
from tf2_geometry_msgs import do_transform_pose,do_transform_pose_stamped
import tf2_geometry_msgs.tf2_geometry_msgs 

class WaypointGenerator(Node):
    def __init__(self):
        super().__init__('waypoint_generator')

        # room_coordinates is a 2x4 array: 1 is top left, 2 is bottom left, 3 is bottom right, 4 is top right
        # [x1, x2, x3, x4]
        # [y1, y2, y3, y4]
        
        # width= robot width

                        
        # for i, wp in enumerate(self.waypoints):
        #     self.get_logger().info(f"Waypoint {i}: x={wp[0]:.2f}, y={wp[1]:.2f}")
            
    def generate_waypoints(self,room_coordinates, width,entry,tf):
        print("room coordinates")
        print(room_coordinates)
        # Extract room bounds
        xleft = room_coordinates[0][0]
     #   print("bleft : ",xleft)
        xright = room_coordinates[0][1]
     #   print("xright : ",xright)
        ybottom = room_coordinates[1][1]
     #   print(" : ",xright)
        ytop = room_coordinates[1][0]
    
        x = xleft + width/ 2
        y = ybottom + width/ 2
    
        index = 0
        waypoints = []
        waypoints.append([entry[0],entry[1]])
        while x < xright - width/ 2:
            robot_frame = PoseStamped()
            robot_frame.header.frame_id = "odom"  
            robot_frame.header.stamp = self.get_clock().now().to_msg()
            robot_frame.pose.position.x = x
            robot_frame.pose.position.y = y
         #   print(robot_frame)
       #    print(type(tf))
         #  tf_pose = do_transform_pose_stamped(robot_frame,tf)
            tf_pose = robot_frame
            waypoints.append([tf_pose.pose.position.x,tf_pose.pose.position.y])
            
            if index % 2 == 1:
                x += width/2
            else:
                if y == ybottom + width/ 2:
                    y = ytop - width/ 2
                elif y == ytop - width/ 2:
                    y = ybottom + width/ 2
            index += 1
        
        waypoints.append([xright - width/ 2, y])
        index += 1
        
        if y == ybottom + width/ 2:
            y = ytop - width/ 2
        elif y == ytop - width/ 2:
            y = ybottom + width/ 2
            
        waypoints.append([xright - width/ 2, y])
        
        return waypoints
    
    
    
            