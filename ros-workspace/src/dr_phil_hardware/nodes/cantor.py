#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker,MarkerArray
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped
import collections
import dr_phil_hardware.RoutePlanner as RoutePlanner


class Cell:
    def __init__(self, row, col):
        self.row_in_grid = row
        self.col_in_grid = col

        self.is_free_space = None
        self.coordinates = []

        self.center = None
        

    def set_cell_availability(self, status):
        self.is_free_space = status
    def add_coordinate(self,coord):
        self.coordinates.append(coord)

    def update_and_return_central_point(self):
        self.center = np.mean(np.array(self.coordinates),axis=0)
        return self.center


class Grid:
    #resolution is meters per pixel
    #total_pixels_area is in pixels
    def __init__(self, ):
        rospy.init_node('cantor')
        self.global_map = None
        self.resolution = None
        self.origin = None
        self.map_size_x = None
        self.map_size_y = None
        self.map_data = None
        self.global_costmap_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback=self.global_costmap_callback)
        self.pub_visualization_marker = rospy.Publisher('/marked_cells', MarkerArray, queue_size=10)
        self.cell_size = 0
        self.cell_width = None
        self.number_of_total_buckets = 0
        self.unexplored_regions = {}

        self.explored_regions = {}  
        #self.available_zones = {}
        self.cell_size_meters = 1


        #used temporarily
        self.spin_count =0




                 
            


    def global_costmap_callback(self, costmap):
        if costmap is not None:
            self.global_map = costmap
            self.resolution = costmap.info.resolution
            self.origin = costmap.info.origin
            self.map_size_x = costmap.info.width
            self.map_size_y = costmap.info.height
            self.map_data = costmap.data
            min_ =  0 
            max_ = self.map_size_x   
            self.cell_size = int(self.cell_size_meters* (1/self.resolution)) #m / (m/p) = m * p/m = p
            # print("cell size" + str(self.cell_size))
            # print("map size"  + str(self.map_size_x))
            self.cell_width = int((max_-min_) / (self.cell_size))
            # print(self.cell_width)
            # print(self.cell_size*self.cell_width)

            self.number_of_total_buckets = int(self.cell_width*self.cell_width)
        
        

    #translates 2D point location into a single integer – the grid cell that the point is in. 
    def _hash_pinpoint_grid_cell(self,x,y):

        grid_cell = (int(x/self.cell_size)) + (int(y/self.cell_size))*int(self.cell_width)
        
        return self.function_map_negative_to_positive_bijection(grid_cell)
        #return grid_cell

    #define f: Z-> N
    def function_map_negative_to_positive_bijection(self,n):
        if n>=0:
            return n*2
        else:
            return (-n * 2) - 1 

    def check_unavailable(self):
        print("Check unavailable")


        for map_x in range(5,self.map_size_x-5):
            for map_y in range(5,self.map_size_y-5):
                idx = (map_x) + (map_y) * self.map_size_x
                world_x, world_y = self.mapToWorld(map_x,map_y)
                cell = self._hash_pinpoint_grid_cell(map_x, map_y)
                # print("map_x " + str(world_x) + " map_y " + str(world_y))
                # print(self.cell_width)
                c = self.unexplored_regions.get(cell)
                if c is None: 
                    row,col = self.pinpoint_location_in_grid(cell)
                    self.unexplored_regions[cell] = Cell(row,col)
                c = self.unexplored_regions[cell]
                if self.map_data[idx] ==0:
                    if (c.is_free_space != False):
                        self.unexplored_regions[cell].set_cell_availability(True)                              
                else:
                    self.unexplored_regions[cell].set_cell_availability(False)
                
                self.unexplored_regions[cell].add_coordinate([world_x,world_y])
                    

        self.mark_all_available_cells()


       
    def pinpoint_location_in_grid(self,cell_num):
        col = int((cell_num %self.cell_width))
        row = int((cell_num / self.cell_width))
        return row,col

    
        


    def mark_all_available_cells(self):
        markers=[]
        print("Mark available cells")

        all_cells = self.return_sorted_dict(self.unexplored_regions)
        # print(all_cells)
        for key,value in all_cells.items():
            #mark as available
            if value.is_free_space:
                print(key)
                #world_x, world_y = self.find_center_point_grid_cell(key)
                coords = value.update_and_return_central_point()
                markers.append(self.mark_cell(coords[0],coords[1],key))
                # routeplanner = RoutePlanner.RoutePlanner([0,0], self.unexplored_regions)
                # print("Tour" + str(routeplanner.tourIndex))
                # print(routeplanner.calculateTourCost(routeplanner.tourIndex))

                # print(routeplanner.generateRoute())
                # print(routeplanner.calculateTourCost(routeplanner.tourIndex))
        self.unexplored_regions = dict(all_cells)
        print("End")
        self.pub_visualization_marker.publish(markers)

    def mark_cell(self, x , y, cell):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp =  rospy.Time.now()
        marker.ns = "cell markers"
        marker.id = cell
        marker.type =1 #Cube
        marker.action = 0 #add/modify
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = self.cell_size * self.resolution
        marker.scale.y =self.cell_size * self.resolution
        marker.scale.z = 0.1
        marker.color.a = 0.6
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0)
        
        return marker
    #Returns the dictionary sorted in type OrderedDict
    def return_sorted_dict(self,dictionary):
        return collections.OrderedDict(sorted(dictionary.items()))

  

                
                    

            


    def worldToMap(self, world_x, world_y):
        map_x = ((world_x - self.global_map.info.origin.position.x) / self.global_map.info.resolution)
        map_y = ((world_y - self.global_map.info.origin.position.y) / self.global_map.info.resolution)
        return map_x,map_y
    
    def mapToWorld(self, map_x, map_y):

        world_x = self.origin.position.x + (map_x + 0.5) * self.resolution
        world_y = self.origin.position.y + (map_y + 0.5) * self.resolution
        return world_x,world_y


    def spin(self):
        if self.global_map is not None and self.spin_count<1:
            self.check_unavailable()
            self.spin_count+=1



    



# def cantor_pairing(x, y):
#     return int((x + y) * (x + y +1)/2 + y)

# def cantor_pairing_nd(*args):
#     if len(args) == 2:
#         return cantor_pairing(args[0], args[1])
#     return cantor_pairing(cantor_pairing_nd(*args[:-1]), args[-1])




if __name__ == '__main__':

    grid= Grid()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        grid.spin()
        rate.sleep()







