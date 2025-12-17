import random
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
class G:
    def __init__(self):
        '''Initialize starting point, number of nodes, step size, domain size'''
        self.q_init = [random.randrange(0,100),random.randrange(0,100)]
        self.q_goal=[random.randrange(0,100),random.randrange(0,100)]
        self.K=500
        self.delta=1
        self.domain= [[0,100],[0,100]]
        self.G=[self.q_init] # Start the chain at the initial goal
        self.dists=[]
        self.edges=[]
        self.path_edges = []
        self.num_circles=40
        print(self.q_goal)

    def _random_config(self,domain):
        '''Create a random configuration within the established domain'''
        self.rand_conf_x=random.randrange(domain[0][0],domain[0][1])
        self.rand_conf_y=random.randrange(domain[1][0],domain[1][1])
        self.rand_pos=[self.rand_conf_x,self.rand_conf_y]
        return self.rand_pos
    
    def nearest_vertex(self,pos,G):
        '''Loop through the current configurations and find the nearest vertex to the new randomly generated one'''
        #locate the nearest vertex in G to the new rand_pos
        q_near=None
        step = [0,0]
        dirs=[]
        for vertex in G:
            x=vertex[0]-pos[0]
            y=vertex[1]-pos[1]
            # print(x, y)
            dir=math.sqrt(x**2+y**2) #Get vector length, scalar
            dirs.append(dir)
            if min(dirs)==dir:
                q_near=vertex
                if dir == 0:
                    break
                v_hat=[(pos[0]-vertex[0])/dir,(pos[1]-vertex[1])/dir]#2D vector
                step=v_hat*self.delta
            if dir == 0:
                break
            else:
                continue
        return q_near,step

    def new_config(self,near_pos,step):
        '''Takes the step defined in nearest_vertex towards the new random position'''
        new_pos=[near_pos[0]+step[0],near_pos[1]+step[1]]   
        dist=math.dist(new_pos,near_pos)
        self.dists.append(dist)
        self.G.append(new_pos)
        self.edges.append((near_pos,new_pos))
        return new_pos

    def circles(self):
        '''Create obstacles for the RRT tree'''
        circles=[]
        for circle in range(self.num_circles):
            r=random.randrange(1,10)
            x=random.randrange(0,100)
            y=random.randrange(0,100)
            # Make sure the circles do not contain the goal or the start
            if ((self.q_goal[0] - x)**2 + (self.q_goal[1] - y)**2 <= r**2):
                continue
            if ((self.q_init[0] - x)**2 + (self.q_init[1] - y)**2 <= r**2):
                continue
            else:
                circles.append(r)
                circles.append(x)
                circles.append(y)
        return circles
    
    def _check_collisions(self,q_near,step,circle):
        '''Checks if the path from q_near to q_new intersects with an obstacle. Checks if the path passes through the circle radius'''
        q_new = [q_near[0] + step[0], q_near[1] + step[1]]
        collide=False
        for x in range(0,len(circle),3):
            if ((q_new[0]-(circle[x+1]))**2 + (q_new[1]-(circle[x+2]))**2 <= circle[x]**2):
                collide=True
                break
        return collide

    def _check_LOS(self, pos):
        '''Checks if there are any obstacles in the way of the robot and the end target.
        
        The way to do this will probably be to move directly towards the goal and constantly
        do collision checking, only draw the line if it does not break.'''
        # Get distance and direction between current and goal positions
        x_diff = self.q_goal[0] - pos[0]
        y_diff = self.q_goal[1] - pos[1]
        dist = math.sqrt(x_diff**2 + y_diff**2)
        collide = False
        # direction = np.atan2(y_diff, x_diff)

        # Step along that direction until either a. The goal is reached or b. A collision is detected
        current_pos = pos
        while not ((self.q_goal[0] - 0.5 < current_pos[0] < self.q_goal[0] + 0.5) and (self.q_goal[1] - 0.5 < current_pos[1] < self.q_goal[1] + 0.5)):
            x_diff = self.q_goal[0] - current_pos[0]
            y_diff = self.q_goal[1] - current_pos[1]
            dist = math.sqrt(x_diff**2 + y_diff**2)
            v_hat = [(self.q_goal[0] - current_pos[0])/dist, (self.q_goal[1] - current_pos[1])/dist]
            step = v_hat * self.delta
            collide = self._check_collisions(current_pos, step, self.circle)
            if collide:
                return False
            else:
                new_pos = [current_pos[0] + step[0], current_pos[1] + step[1]]
                current_pos = new_pos
        return True

    def optimal_path(self, G):
        """Find the optimal path once the goal has been reached."""
        self.path = self.G.reverse()
        
        #Step through 100 points between the goal and the start pos and check collision at each location.
    def create_tree(self):
        '''Loop through the number of vertices and for each one
        generate a random point and take a step towards that point,
        creating a new vertex at that point and adding that vertex to G'''
        K=0
        patches=[]
        self.circle=self.circles()

        while K<self.K:
            self.rand_pos = self._random_config(self.domain)
            self.q_near,self.step = self.nearest_vertex(self.rand_pos,self.G)
            collide=self._check_collisions(self.q_near,self.step,self.circle)
            LOS = self._check_LOS(self.q_near)
            print(LOS)
            if LOS:
                self.edges.append((self.q_near, self.q_goal))
                break
            if collide:
                continue
            else:
                self.new_config(self.q_near,self.step)
                K+=1
        #Graph the tree
        x_cords=[[point[0] for point in self.G]]
        y_cords=[[point[1] for point in self.G]]
        print(self.G)
        for x in range(0,len(self.circle),3):
            circle1=plt.Circle((self.circle[x+1],self.circle[x+2]),self.circle[x],color='black')
            patches.append(circle1)
        fig,ax=plt.subplots()
        for x in patches:
            ax.add_patch(x)
        plt.scatter(x_cords,y_cords,color='Blue',s=10)
        #Graph the connections between each vertex
        lines=LineCollection(self.edges,color='blue',linewidths=2)
        plt.gca().add_collection(lines)
        #Set the parameters for the graph
        plt.xlim((0,100))
        plt.ylim((0,100))
        plt.title(f'Figure 1: RRT after {len(self.G)} iterations')
        plt.scatter(self.q_init[0],self.q_init[1], color='red', s=20)
        plt.scatter(self.q_goal[0],self.q_goal[1], color='green', s=20)
        plt.show()
        
if __name__=='__main__':
    RRT=G()
    RRT.create_tree()







