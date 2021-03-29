import numpy as np
import math
from typing import Dict,List



#The class tackles the Traveling Salesmen Problem using TwoOpt for finding the estimated, best journey given a list of points and a starting point.
class RoutePlanner: 
    def __init__(self, start_point : List[float], grid_cells : Dict):

        STARTING_INDEX = -1
        #Stores the distances from one point to another
        # Represents the pathing scheduled from RoutePlanner object, with the value in tourIndex representing a mapping to the grid number in the array.
        #  I.e) The indicies represent the flight number in order (with 0 being the departure at the starting point). The value 5 at an index i signifies that the robot  gets in range to grid cell numbered 5 
        #This will be modified to represent the best tour
        self.free_cells= [cell_num for cell_num,val in  grid_cells.items() if val.is_free_space==True]
        print(self.free_cells)
        #Add the start point to represent the starting index. Make sure that the grid cell numbers are all positive numbers 
        # self.tourIndex.insert(0,STARTING_INDEX)
        self.free_cells.insert(0,None) 
        self.tourIndex = [x for x in range(0,len(self.free_cells))]

 
        self.tourLength = len(self.tourIndex)
        print((self.tourLength))
        print(len(self.tourIndex))

        self.distances = np.zeros(shape=(self.tourLength,self.tourLength))


        self.actual_distance = 0


        """
        * Constructs a distance matrix representing the distances from one point to another when the constructor is called. 
        """

        #Initialise a 2D array to store the distances as an adjacency matrix.
        for i in range(0,self.tourLength):
            currCoordinates = None
            if (i ==0) :
                currCoordinates = start_point
            else:
                cell = self.free_cells[i]
                currCoordinates =  grid_cells[cell].update_and_return_central_point()
            
            for j in range(0,i):
                #Calculate the distance between the two points if we are not at the same point.
                if (i != j):
                    toCoordinates = None
                    if (j == 0):
                        toCoordinates = start_point    
                    else:
                        toCoordinates= grid_cells[self.free_cells[j]].update_and_return_central_point()
                    
                    #calculate distance between two points
                    dist = self.euclidean(currCoordinates,toCoordinates)
         
                    self.distances[i][j] = dist
                    self.distances[j][i] = dist
            
            
        
    def euclidean(self,pointA,pointB):
        x_A , y_A = pointA[0], pointA[1]
        x_B , y_B = pointB[0], pointB[1]
        return math.sqrt(math.pow(x_A - x_B, 2) + math.pow(y_A - y_B, 2))

        
    """
    * Runs the TwoOpt Heuristic of obtaining the best path journey. 
    * @return the journey chosen by the algorithm as list of integers with values corresponding to the cell number. Note that the value -1, STARTING_INDEX, corresponds to the starting point.
    """
    def generateRoute(self):
        self.actual_distance=0
        self.twoOpt()
        cells = self.free_cells
        cells.pop(0)
        return (cells)

    def nearest_cell(self,grid,cells,current_position):
        lowest_distance = float('inf')
        next_cell = None
        for cell in cells:
            dist = self.euclidean(current_position, grid.unexplored_regions[cell].update_and_return_central_point())
            if dist < lowest_distance:
                next_cell = cell
                lowest_distance = dist
        
        self.actual_distance+=lowest_distance
        return next_cell


    """
        * Using the distance matrix, calculate the distance to traverse a route configuration
        * @param tour The path journeys (The idea similar to the explanation for tourIndex attribute).
        * @return the total distance to traverse this route configuration.
    """
    def calculateTourCost(self, tour):
        cost = 0.0
        for i in range(0,self.tourLength-1):
            current = tour[i]
            to = tour[i+1]
            cost += self.distances[current][to]
        
        current = tour[self.tourLength-1]
        to = tour[0]
        cost += self.distances[current][to]
        return cost


    """
        * The algorithm for attempting to find the best route configuration for Dr-Phil. 
        * The main idea is that it tries to attempt reversing portions of routes and sees if there are any improvements. This is done iteratively until reversing doesnt give a better result.
        * 
    """
    def twoOpt(self):
        improved = True
        while improved==True:
            improved = False
            originalDistance = self.calculateTourCost(self.tourIndex)
            for m in range(1,self.tourLength-1):
                for n in range(m+1,self.tourLength):
                    newRoute, newOrderedCells = self.attemptReverseRoute(m,n)
                    newDistance = self.calculateTourCost(newRoute)
                    if (newDistance < originalDistance):
                        self.tourIndex = newRoute
                        self.free_cells = newOrderedCells
                        improved = True

    # def greedy():

                    
                
            
        



    """
    * Consider taking the reverse route between a point at index m in our tour and another point at index n. If the cost of 
    * reversing the route is lower than the cost originally, then change the new tour with the better cost.
    * We are only comparing the changes by examining the new distances between the points (or cells) we are attempting the reverse. 
    * The costs of the route with unchanged elements will still yield the same result 
    * (i.e A , B , C , D , F , G). Reversing B and F, the new route will be : A , F , D ,C , B , G. Examine if the new route total cost is lower
    * @param mInd the path number to consider swapping the routes for
    * @param nInd the second path number to consider swapping the routes for
    * @return the new path journeys
    """
    #mInd < nInd
    def attemptReverseRoute(self, mInd, nInd):
        newRoute = np.arange(self.tourLength)
        newOrderedCells = []
        #print(newRoute.shape)
        index = 0;
        for i in range(0,mInd): 
            newRoute[i] = self.tourIndex[i];
            newOrderedCells.append(self.free_cells[self.tourIndex[i]])
            index+=1;
        #Go from right(nInd) to left(mInd)
        for i in range(nInd,mInd-1,-1):
            newRoute[index] = self.tourIndex[i];
            newOrderedCells.append(self.free_cells[self.tourIndex[i]])
            index+=1;
        
        for i in range(nInd+1, self.tourLength):
            newRoute[index] = self.tourIndex[i];
            newOrderedCells.append(self.free_cells[self.tourIndex[i]])
            index+=1
        
        
        return newRoute, newOrderedCells;

    def gettourIndex(self) :
        return self.tourIndex;









    
