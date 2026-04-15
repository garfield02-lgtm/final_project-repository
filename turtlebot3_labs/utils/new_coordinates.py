import rclpy
from rclpy.node import Node


class WaypointGenerator(Node):
    def __init__(self):
        super().__init__('waypoint_generator')

        # room_coordinates is a 2x4 array: 1 is top left, 2 is bottom left, 3 is bottom right, 4 is top right
        # [x1, x2, x3, x4]
        # [y1, y2, y3, y4]
        
        # w = robot width

                        
        # for i, wp in enumerate(self.waypoints):
        #     self.get_logger().info(f"Waypoint {i}: x={wp[0]:.2f}, y={wp[1]:.2f}")
    def waypoint_step(self, vert, vert_min, vert_max, step_vert, hor, step_hor,w,index):
            vert += step_vert

            if vert >= vert_max and index == 0:
                vert = vert_max
                index = 1
            elif vert >= vert_max and index == 1:
                vert = vert_max
                step_vert = -w
                hor += step_hor
                index = 0
            elif vert <= vert_min and index == 0:
                vert = vert_min
                index = 1
            elif vert <= vert_min and index == 1:
                vert = vert_min
                step_vert = w
                hor += step_hor
                index = 0
            return hor, vert, step_vert, index
    def generate_waypoints(self, room_coordinates, w, entry_point):
        
        # Extract room bounds
        xleft = room_coordinates[0][0]
        xright = room_coordinates[0][1]
        ybottom = room_coordinates[1][1]
        ytop = room_coordinates[1][0]

        x_length = xright - xleft
        y_length = ytop - ybottom        
        
        xmin = xleft+w/2
        xmax = xright-w/2
        ymin = ybottom+w/2
        ymax = ytop-w/2

        if x_length > y_length:
            clean_along_x = 1
        else:
            clean_along_x = 0

        if abs(entry_point[1] - ybottom) > abs(entry_point[1] - ytop):
            y = ymax
        else:
            y = ymin

        if abs(entry_point[0] - xleft) > abs(entry_point[0] - xright):
            x = xmax
        else:
            x = xmin
        index = 0
        waypoints = []

        if clean_along_x == 0 and x == xmin:
            print("bottom_right")
            step_across = w
            if y == ymin:
                step_vert = w
            else:
                step_vert = -w

            while x < xmax:
                waypoints.append([x,y])
                x, y, step_vert, index = self.waypoint_step(vert=y, vert_min=ymin, vert_max=ymax, step_vert=step_vert, hor=x, step_hor=step_across, w=w, index=index)
            
            x = xmax
            while x == xmax:
                waypoints.append([x,y])
                x, y, step_vert, index = self.waypoint_step(vert=y, vert_min=ymin, vert_max=ymax, step_vert=step_vert, hor=x, step_hor=step_across, w=w, index=index)
            
        elif clean_along_x == 0 and x == xmax:
            print("top_right")
            step_across = -w
            if y == ymin:
                step_vert = w
            else:
                step_vert = -w

            while x > xmin:
                waypoints.append([x,y])
                x, y, step_vert, index = self.waypoint_step(vert=y, vert_min=ymin, vert_max=ymax, step_vert=step_vert, hor=x, step_hor=step_across, w=w, index=index)
            
            x = xmin
            while x == xmin:
                waypoints.append([x,y])
                x, y, step_vert, index = self.waypoint_step(vert=y, vert_min=ymin, vert_max=ymax, step_vert=step_vert, hor=x, step_hor=step_across, w=w, index=index)

        elif clean_along_x == 1 and y == ymin:
            step_across = w
            if x == xmin:
                step_vert = w
            else:
                step_vert = -w

            while y < ymax:
                waypoints.append([x,y])
                y, x, step_vert, index = self.waypoint_step(vert=x, vert_min=xmin, vert_max=xmax, step_vert=step_vert, hor=y, step_hor=step_across, w=w, index=index)

            y = ymax
            while y == ymax:
                waypoints.append([x,y])
                y, x, step_vert, index = self.waypoint_step(vert=x, vert_min=xmin, vert_max=xmax, step_vert=step_vert, hor=y, step_hor=step_across, w=w, index=index)

        elif clean_along_x == 1 and y == ymax:
            step_across = -w
            if x == xmin:
                step_vert = w
            else:
                step_vert = -w

            while y > ymin:
                waypoints.append([x,y])
                y, x, step_vert, index = self.waypoint_step(vert=x, vert_min=xmin, vert_max=xmax, step_vert=step_vert, hor=y, step_hor=step_across, w=w, index=index)

            y = ymin
            while y == ymin:
                waypoints.append([x,y])
                y, x, step_vert, index = self.waypoint_step(vert=x, vert_min=xmin, vert_max=xmax, step_vert=step_vert, hor=y, step_hor=step_across, w=w, index=index)
        return waypoints
    

  
           