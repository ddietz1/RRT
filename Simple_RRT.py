"""RRT program."""
import math
import random

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection


class G:
    """Class to create the RRT."""

    def __init__(self):
        """Initialize necessary variables, list, and dicts."""
        self.q_init = [random.randrange(0, 150), random.randrange(0, 150)]
        self.q_goal = [random.randrange(0, 150), random.randrange(0, 150)]
        self.K = 500
        self.delta = 1
        self.domain = [[0, 150], [0, 150]]
        self.G = [self.q_init]  # Start the chain at the initial goal
        self.tree = {}  # Dictionary to hold the child and parent nodes
        self.tree[tuple(self.q_init)] = None
        self.dists = []
        self.edges = []
        self.path_edges = []
        self.num_circles = 40
        self.goal_found = False

    def _random_config(self, domain):
        """Create a random configuration within the established domain."""
        self.rand_conf_x = random.randrange(domain[0][0], domain[0][1])
        self.rand_conf_y = random.randrange(domain[1][0], domain[1][1])
        self.rand_pos = [self.rand_conf_x, self.rand_conf_y]
        return self.rand_pos

    def nearest_vertex(self, pos, G):
        """
        Locate the nearest node to the newly generated one.

        :param pos: newly generated position in the tree
        :param G: List of all the nodes currently on the tree
        """
        q_near = None
        step = [0, 0]
        dirs = []
        for vertex in G:
            x = vertex[0]-pos[0]
            y = vertex[1]-pos[1]
            direction = math.sqrt(x**2+y**2)
            dirs.append(direction)
            if min(dirs) == direction:
                q_near = vertex
                if direction == 0:
                    break
                v_hat = [
                    (pos[0]-vertex[0])/direction,
                    (pos[1]-vertex[1])/direction
                ]
                step = v_hat*self.delta
            if direction == 0:
                break
            else:
                continue
        return q_near, step

    def new_config(self, near_pos, step):
        """Take a unit step towards the new node."""
        new_pos = [near_pos[0]+step[0], near_pos[1]+step[1]]
        dist = math.dist(new_pos, near_pos)
        self.dists.append(dist)
        self.G.append(new_pos)
        self.edges.append((near_pos, new_pos))
        self.tree[tuple(new_pos)] = tuple(near_pos)
        return new_pos

    def circles(self):
        """Create obstacles for the RRT tree."""
        circles = []
        for circle in range(self.num_circles):
            r = random.randrange(1, 15)
            x = random.randrange(0, 150)
            y = random.randrange(0, 150)
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

    def _check_collisions(self, q_near, step, circle):
        """Check if the path intersects with an obstacle."""
        q_new = [q_near[0] + step[0], q_near[1] + step[1]]
        collide = False
        for x in range(0, len(circle), 3):
            if (
                (q_new[0]-(circle[x+1]))**2 +
                (q_new[1]-(circle[x+2]))**2 <=
                circle[x]**2
            ):
                collide = True
                break
        return collide

    def _check_LOS(self, pos):
        """Check if there are obstacles between robot and the end target."""
        # Get distance and direction between current and goal positions
        x_diff = self.q_goal[0] - pos[0]
        y_diff = self.q_goal[1] - pos[1]
        dist = math.sqrt(x_diff**2 + y_diff**2)
        collide = False
        # direction = np.atan2(y_diff, x_diff)

        # Step until a collision is detected or
        # the goal is reached
        current_pos = pos
        while not (
            (self.q_goal[0] - 0.5 < current_pos[0] < self.q_goal[0] + 0.5) and
            (self.q_goal[1] - 0.5 < current_pos[1] < self.q_goal[1] + 0.5)
        ):
            x_diff = self.q_goal[0] - current_pos[0]
            y_diff = self.q_goal[1] - current_pos[1]
            dist = math.sqrt(x_diff**2 + y_diff**2)
            v_hat = [
                (self.q_goal[0] - current_pos[0])/dist,
                (self.q_goal[1] - current_pos[1])/dist
            ]
            step = v_hat * self.delta
            collide = self._check_collisions(current_pos, step, self.circle)
            if collide:
                return False
            else:
                new_pos = [current_pos[0] + step[0], current_pos[1] + step[1]]
                current_pos = new_pos
        return True

    def optimal_path(self):
        """Find the optimal path once the goal has been reached."""
        path = []
        current = tuple(self.q_goal)

        while current is not None:
            path.append(current)
            current = self.tree[current]
        path.reverse()
        return path

    def RRTStep(self):
        """Step though one iteration of the tree."""
        self.rand_pos = self._random_config(self.domain)
        self.q_near, self.step = self.nearest_vertex(self.rand_pos, self.G)

        if self._check_collisions(self.q_near, self.step, self.circle):
            return False

        if self._check_LOS(self.q_near):
            self.G.append(self.q_goal)
            self.tree[tuple(self.q_goal)] = tuple(self.q_near)
            self.edges.append((self.q_near, self.q_goal))
            return True  # goal reached

        self.new_config(self.q_near, self.step)
        return False

    def create_tree(self):
        """Call helper functions K times to create the tree."""
        fig, ax = plt.subplots()
        plt.xlim((0, 150))
        plt.ylim((0, 150))
        plt.title('Figure 1: RRT Graph')

        # Graph obstacles
        self.circle = self.circles()  # Generate obstacles
        patches = []
        for x in range(0, len(self.circle), 3):
            circle1 = plt.Circle(
                (self.circle[x+1],
                 self.circle[x+2]),
                self.circle[x], color='black'
            )
            patches.append(circle1)

        for x in patches:
            ax.add_patch(x)
        # K = 0

        # Plot the end goal and start position
        plt.scatter(self.q_init[0], self.q_init[1], color='purple', s=30)
        plt.scatter(self.q_goal[0], self.q_goal[1], color='green', s=30)

        # Initialize the graphing methods
        nodes = ax.scatter([], [], s=10, color='blue')
        tree_lines = LineCollection([], colors='blue', linewidths=1)
        ax.add_collection(tree_lines)

        def update(frame):
            if not self.goal_found:
                reached = self.RRTStep()

                cords = [(point[0], point[1]) for point in self.G]

                nodes.set_offsets(cords)
                tree_lines.set_segments(self.edges)
                ax.set_title(f'RRT Growth – Nodes: {len(self.G)}')

                if reached:
                    self.goal_found = True
                    path = self.optimal_path()
                    path_edges = [
                        (path[i], path[i+1]) for i in range(len(path) - 1)
                    ]
                    ax.add_collection(
                        LineCollection(path_edges, colors='red', linewidths=3)
                    )
                    anim.event_source.stop()
            return nodes, tree_lines
        anim = FuncAnimation(
            fig,
            update,
            frames=self.K,
            interval=30,
            blit=True
        )
        anim.save('RRT.gif', writer=PillowWriter(fps=30))
        plt.show()

        # while K < self.K:
        #     self.rand_pos = self._random_config(self.domain)
        #     self.q_near, self.step = self.nearest_vertex(self.rand_pos, self.G)
        #     collide = self._check_collisions(
        #         self.q_near, self.step, self.circle
        #     )
        #     LOS = self._check_LOS(self.q_near)
        #     if LOS:
        #         self.G.append(self.q_goal)
        #         self.tree[tuple(self.q_goal)] = tuple(self.q_near)
        #         self.edges.append((self.q_near, self.q_goal))
        #         break
        #     if collide:
        #         continue
        #     else:
        #         self.new_config(self.q_near, self.step)
        #         K += 1

        # Graph the tree
        # x_cords = [[point[0] for point in self.G]]
        # y_cords = [[point[1] for point in self.G]]

        # # Extract optimal path
        # path = self.optimal_path()
        # path_coordsx = [[point[0] for point in path]]
        # path_coordsy = [[point[1] for point in path]]

        # Add the edges for the optimal path
        # path_edges = []
        # for i in range(len(path) - 1):
        #     path_edges.append((path[i], path[i+1]))

        # # Graph all nodes in the tree
        # plt.scatter(x_cords, y_cords, color='Blue', s=10)

        # # Graph the connections between each node
        # lines = LineCollection(self.edges, color='blue', linewidths=2)
        # plt.gca().add_collection(lines)

        # # Graph only the connections in the optimal path with a new color
        # path_lines = LineCollection(path_edges, color='red', linewidths=2)
        # plt.gca().add_collection(path_lines)

        # # Set the parameters for the graph

        # # Plot the optimal path
        # plt.scatter(path_coordsx, path_coordsy, color='red', s=10)

        # plt.show()


if __name__ == '__main__':
    RRT = G()
    RRT.create_tree()
