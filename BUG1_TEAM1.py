######### MAE 195 Motion Planning #############################
########## Bug 1 Algorithm #############################

# importing necessary libraries for the algorithm
from sys import platform
import numpy as np
import math
import matplotlib.pyplot as plt
from shapely.geometry.polygon import Polygon
from shapely import geometry

animation = True

class Planner: 
    #Define the goal points and 
    def __init__(self, start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y):
    	#initialize the goal and start points along with obstacle 
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.obstacle_x = obstacle_x
        self.obstacle_y = obstacle_y
        self.path_x = [start_x]
        self.path_y = [start_y]
        #This is to determine the boundaries of the polygons
        self.boundary_x = []
        self.boundary_y = []

        # want to determine the boundaries.
        for obs_x, obs_y in zip(obstacle_x, obstacle_y):
            for stp_x, stp_y in zip([1, 0, -1, -1, -1, 0, 1, 1],
                                    [1, 1, 1, 0, -1, -1, -1, 0]):
                candidate_x, candidate_y = obs_x+ stp_x, obs_y + stp_y
                valid_point = True
                for x, y in zip(obstacle_x, obstacle_y):
                    if candidate_x == x and candidate_y == y:
                        valid_point = False
                        break
                if valid_point:
                    self.boundary_x.append(candidate_x), self.boundary_y.append(candidate_y)
    #Defines movement when in free workspace
    def movement(self):
        return self.path_x[-1] + np.sign(self.goal_x - self.path_x[-1]) , self.path_y[-1] + np.sign(self.goal_y - self.path_y[-1])
    
    # Defines movement when in obstavcle boundary
    def movement_obstacles(self,visited_x,visited_y):
        ## These are the step sizes for the movement along the boundaries
        for step_x, step_y in zip([1,0,-1,0],[0,1,0,-1]):
            candidate_x,candidate_y = self.path_x[-1] + step_x,self.path_y[-1]+step_y
            for x,y in zip(self.boundary_x,self.boundary_y):
                candidate_point = True 
                if candidate_x == x and candidate_y == y:
                    for i,j in zip(visited_x,visited_y):
                        if candidate_x == i and candidate_y == j:
                            candidate_point = False 
                            break 
                    if candidate_point: 
                        return candidate_x,candidate_y,False
                if not candidate_point:
                    break 
        return self.path_x[-1],self.path_y[-1], True

def main():
    # set obstacle positions
    o_x, o_y = [], []

    #Defines Start and Goal location
    start_x ,start_y = input("Enter the starting point: eg, 70 10:  ").split()
    s_x,s_y= int(start_x),int(start_y)
    goal_x,goal_y = input("Enter the goal points: eg , 100 125:   ").split()
    g_x,g_y = int(goal_x),int(goal_y)
    print('PLEASE WAIT FOR AROUND 30-40 seconds')
    print('-------LOADING------------------------ ')
 
    #Defining individual obstacles estimation for a 200 * 200 grid
    poly1 = Polygon([(85,15), (100,40), (80,60), (65,35), (85,15)])
    x,y = poly1.exterior.xy
    for i in range(int(min(x)), int(max(x))):
        for j in range(int(min(y)), int(max(y))):
            point = geometry.Point(i,j)
            if poly1.contains(point) == True:
                o_x.append(i)
                o_y.append(j)


    poly2 = Polygon([(35,70), (60,75), (55,105), (30,100), (35,70)])
    x,y = poly2.exterior.xy
    for i in range(int(min(x)), int(max(x))):
        for j in range(int(min(y)), int(max(y))):
            point = geometry.Point(i,j)
            if poly2.contains(point) == True:
                o_x.append(i)
                o_y.append(j)

    poly3 = Polygon([(130,100), (150,120), (130,145), (110,125), (130,100)])
    x,y = poly3.exterior.xy
    for i in range(int(min(x)), int(max(x))):
        for j in range(int(min(y)), int(max(y))):
            point = geometry.Point(i,j)
            if poly3.contains(point) == True:
                o_x.append(i)
                o_y.append(j)

    poly4 = Polygon([(135,157.5), (120,180), (60,140), (120,50), (180,90), (165,112.5), (120,82.5), (90,127.5), (135,157.5)])
    x,y = poly4.exterior.xy
    for i in range(int(min(x)), int(max(x))):
        for j in range(int(min(y)), int(max(y))):
            point = geometry.Point(i,j)
            if poly4.contains(point) == True:
                o_x.append(i)
                o_y.append(j)
    
    ## checking if the input point is valid.
    point_start = geometry.Point(s_x,s_y)
    point_goal = geometry.Point(g_x,g_y)
    polygon_list = [poly1,poly2,poly3,poly4]
    for poly in polygon_list:
        if poly.contains(point_start) == True or poly.contains(point_goal) == True: 
            raise Exception('The input are incorrect. The value was:{}'.format(point_start))
            break


    # initializing the parameters of the bug_algorithm 
    bug= Planner(s_x, s_y, g_x, g_y, o_x, o_y)
    bug_direction = 'normal' # first the bug moves in the free workspace
    candidate_x = -np.inf
    candidate_y = -np.inf
    exit_x = -np.inf
    exit_y = -np.inf
    distance = np.inf
    back_to_p_hit = False # checking if it still the first round
    second_round = False
    # plotting the values in a graph
    if animation:
        plt.plot(bug.obstacle_x, bug.obstacle_y, ".y")
        plt.plot(bug.path_x[-1],bug.path_y[-1],"og")
        plt.annotate("Start",(bug.path_x[-1],bug.path_y[-1]),fontsize= 10,color ='g')
        plt.plot(bug.goal_x,bug.goal_y,"db")
        plt.annotate("Goal",(bug.goal_x,bug.goal_y),fontsize= 10,color ='g')
        #ax.plot(bug.r_x[-1],bug1.r_y[-1]))
        plt.plot(bug.boundary_x, bug.boundary_y, ".k")
        ax = plt.gca()
        ax.set_facecolor("lightcyan")
        plt.xlim([0,200]) # Defining the grid
        plt.ylim([0,200])
        plt.grid(True) 
        plt.title('BUG 1 (contributor: Members of Team 1)')
    # determining if the candiates points are in the boundary
    for x,y in zip(bug.boundary_x,bug.boundary_y):
        if bug.path_x[-1] == x and bug.path_y[-1] == y:
            bug_direction = 'obstacle'
            break
    
    x_visited, y_visited = [],[]
    while True: 
        if bug.path_x[-1] == bug.goal_x and bug.path_y[-1] == bug.goal_y:
            break  ### when goal reached break
        # when the bug moves in the free workspace 
        if bug_direction == 'normal':
            candidate_x,candidate_y = bug.movement()
        # when the bug detects obstacle
        if bug_direction =="obstacle":
            candidate_x,candidate_y,back_to_p_hit = bug.movement_obstacles(x_visited,y_visited)
        if bug_direction == 'normal':
            p_hit = False
            for x,y in zip(bug.boundary_x,bug.boundary_y):
                if candidate_x == x and candidate_y == y:
                    #candidate_x,candidate_y, p = bug.movement_obstacles(x_visited,y_visited)
                    bug.path_x.append(candidate_x),bug.path_y.append(candidate_y)
                    x_visited[:], y_visited[:] = [], []
                    x_visited.append(candidate_x), y_visited.append(candidate_x)
                    bug_direction = 'obstacle'
                    distance = np.inf
                    back_to_p_hit = False 
                    second_round = False
                    p_hit = True 
                    #Annotating the p hit point
                    plt.annotate("P_hit",(candidate_x,candidate_y),fontsize= 10,color='b')
                    break
            if not p_hit:
                bug.path_x.append(candidate_x),bug.path_y.append(candidate_y)
        # alforithm for odometer,  calculating the distance from every boundary point
        elif bug_direction == 'obstacle':
            d = np.linalg.norm(np.array([candidate_x,candidate_y]-np.array([bug.goal_x,bug.goal_y])))
            
            if d < distance and not second_round:
                exit_x,exit_y = candidate_x,candidate_y
                distance = d 
            if back_to_p_hit and not second_round: 
                second_round = True ## now it's the second round , so we have to exit 
                del bug.path_x[-len(x_visited):]
                del bug.path_y[-len(y_visited):]
                x_visited[:] , y_visited[:] = [],[]
            bug.path_x.append(candidate_x),bug.path_y.append(candidate_y)
            x_visited.append(candidate_x),y_visited.append(candidate_y)
            if candidate_x == exit_x and candidate_y == exit_y and second_round: 
                bug_direction = 'normal'
        if animation: 
            plt.plot(bug.path_x,bug.path_y,"-r") # plotting the path
            plt.pause(0.001)
    if animation:
        plt.show()


if __name__ == '__main__':
    main()

