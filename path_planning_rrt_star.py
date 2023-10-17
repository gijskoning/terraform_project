# python file that contains functions for pathplanning
"""

Path planning Sample Code with RRT with Reeds-Shepp path

author: AtsushiSakai(@Atsushi_twi)

"""
import copy
import math
import random
import time

import pygame

import matplotlib.pyplot as plt
import numpy as np
from collision_detection import *

from rrt_src import reeds_shepp_path_planning
from video_exporter import VideoGenerator
from rotation_methods import config_to_polygon
from rrt_src.reeds_shepp_path_planning import mod2pi, MODE_TO_INT
from environment import convert_pos, DISPLAY, m2p

plt.interactive(True)


class RRTStarReedsShepp:
    """
    Class for RRT star planning with Reeds Shepp path
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y, yaw):
            self.x = x  # meters
            self.y = y  # meters
            self.yaw = yaw  # radians

            self.path_x = []
            self.path_y = []
            self.path_yaw = []
            self.path_direction = []
            self.path_mode = []
            self.cost = 0.0
            self.parent = None

        def overwrite_with(self, other_node):
            # copy all other_node attributes to this node.
            for a in dir(self):
                if not a.startswith('__'):
                    setattr(self, a, getattr(other_node, a))

        @classmethod
        def create_with_path(cls, x, y, yaw, px, py, pyaw, directions, modes, cost, parent):
            n = cls(x, y, yaw)
            n.path_x = px
            n.path_y = py
            n.path_yaw = pyaw
            n.path_direction = directions
            n.path_mode = modes
            n.cost = cost
            n.parent = parent
            return n

    def __init__(self, start, goal, obstacle_list, rand_area,
                 max_iter=200,
                 connect_circle_dist=20.0  # used to be 50
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        self.start = self.Node(start[0], start[1], start[2])  # meters
        self.end = self.Node(goal[0], goal[1], goal[2])  # meters
        self.min_x = rand_area[0]
        self.min_y = rand_area[1]
        self.max_x = rand_area[2]
        self.max_y = rand_area[3]
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.connect_circle_dist = connect_circle_dist

        self.curvature = 0.35
        self.goal_yaw_th = np.deg2rad(1.0)

        # Used in Reed shepp path planning. Minimal size for a segment. - Gijs
        self.step_size = 0.7
        self.yaw_distance_weight = 1  # Higher weight increases the effect of the yaw distance in the distance metric

    def distance_pos_nodes(self, node1, node2):
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node1.y) ** 2)

    def distance_metric(self, node1: Node, node2: Node):

        def distance_angle(x, y):
            # Expect angles in radians
            x = mod2pi(x, lower_bound=0)
            y = mod2pi(y, lower_bound=0)
            error = abs(math.sin((x - y) / 2))

            return error

        return self.distance_pos_nodes(node1, node2) + distance_angle(node1.yaw, node2.yaw) * self.yaw_distance_weight

    def get_nearest_node_index(self, node_list, rnd_node):
        """
        This is the distance metric
        """
        min_node = None
        min_cost = float('inf')
        for node in node_list:
            cost = self.distance_metric(node, rnd_node)
            if min_cost > cost:
                min_node = node
                min_cost = cost

        return min_node

    @staticmethod
    def check_collision(node, obstacle_list):  # called for every new possible node
        path_list = list(zip(node.path_x, node.path_y, node.path_yaw))  # list of tuples (x,y,yaw) for all steps in path

        if node is None:  # if no node is passed, todo: can we remove this?
            return False

        for config in path_list:  # for each (x,y,yaw) config step in the proposed path to the new node
            car = config_to_polygon(config)
            for obstacle in obstacle_list:  # for every obstacle
                if collide(obstacle, car):  # check if the two passed polygons collide/overlap
                    # if there is collision for one config in path
                    return False  # path is invalid
        return True  # if not a single collision along path, path is safe

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        near_inds = []
        max_dist = r ** 2
        for i, node in enumerate(self.node_list):
            dist = self.distance_metric(node, new_node)
            if dist <= max_dist:
                near_inds.append(i)
        return near_inds

    def planning(self,
                 video_capture: VideoGenerator,
                 refresh_background,
                 video_generation=False,
                 animation=True,
                 search_until_max_iter=True):
        """
        planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        self.goal_node_list = []
        iter_steps_time = []
        iter_start_time = None
        for i in range(self.max_iter):
            if iter_start_time is not None:
                iter_steps_time.append(time.time()-iter_start_time)
            iter_start_time = time.time()
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_node = self.get_nearest_node_index(self.node_list, rnd)
            # todo steering function gives many options and selects shortest but what if that one collides with objects?

            new_node = self.steer(nearest_node, rnd)

            if self.check_collision(new_node, self.obstacle_list):
                near_indexes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_indexes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_indexes)
                    self.try_goal_path(new_node)
            if (animation and i % 5 == 0) or i == self.max_iter - 1:
                refresh_background()

            if (animation and i % 1 == 0) or i == self.max_iter - 1:
                # self.plot_start_goal_arrow(plot)
                # self.draw_graph(rnd, plot=plot)
                line_length = 0.3
                if rnd is not None:  # pygame plot the random sample
                    pygame.draw.line(DISPLAY, color=(255, 0, 255),
                                     start_pos=convert_pos([rnd.x, rnd.y]),
                                     end_pos=convert_pos([rnd.x + line_length * np.cos(rnd.yaw),
                                                          rnd.y + line_length * np.sin(rnd.yaw)]),
                                     width=1)
                for node in self.node_list:  # draw all line segments for all paths between nodes
                    if node.parent:
                        for j in np.arange(len(node.path_x) - 1):
                            start_pos = convert_pos([node.path_x[j], node.path_y[j]])
                            end_pos = convert_pos([node.path_x[j + 1], node.path_y[j + 1]])
                            pygame.draw.line(DISPLAY, (255, 0, 0), start_pos, end_pos)
                            pygame.draw.circle(DISPLAY, (255, 255, 255), convert_pos([node.x,node.y]), 4)
                pygame.display.flip()

                if video_generation:
                    video_capture.save_frame(screen=DISPLAY)

            if (not search_until_max_iter) and new_node:  # check reaching the goal
                best_node = self.search_best_goal_node()
                if best_node is not None:
                    return self.generate_final_course(best_node), iter_steps_time

        print("reached max iteration")
        best_node = self.search_best_goal_node()
        if best_node is not None:
            return self.generate_final_course(best_node), iter_steps_time
        else:
            print("Cannot find path")

        return None

    def rewire(self, new_node: Node, near_inds):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree

                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.

        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            improved_cost = near_node.cost > edge_node.cost
            if improved_cost:
                no_collision = self.check_collision(edge_node, self.obstacle_list)
                if no_collision:
                    near_node.overwrite_with(edge_node)
                    self.propagate_cost_to_leaves(new_node)

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node

            Returns.
            ------
                Node, a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        best_node = None
        min_cost = float("inf")
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and t_node.cost < min_cost and self.check_collision(t_node, self.obstacle_list):
                min_cost = t_node.cost
                best_node = t_node

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        return best_node

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def try_goal_path(self, node):
        goal = self.end

        new_end_node = self.steer(node, goal)
        if new_end_node is None:
            return

        if self.check_collision(new_end_node, self.obstacle_list):
            self.node_list.append(new_end_node)
            self.goal_node_list.append(new_end_node)

    def draw_graph(self, rnd=None, plot=plt):

        # plt.clf()
        # for stopping simulation with the esc key.
        # axes.gcf().canvas.mpl_connect('key_release_event',
        #                              lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plot.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plot.plot(node.path_x, node.path_y, "-g")  # "-g"

        for obstacle in self.obstacle_list:
            x = [x for (x, y) in obstacle]
            y = [y for (x, y) in obstacle]
            plt.fill(x, y)

        plot.plot(self.start.x, self.start.y, "xr")
        plot.plot(self.end.x, self.end.y, "xr")
        plot.axis([-2, 15, -2, 15])
        plot.grid(True)
        self.plot_start_goal_arrow(plot)

    def plot_start_goal_arrow(self, plot):
        reeds_shepp_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw, plot=plot)
        reeds_shepp_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw, plot=plot)

    def steer(self, from_node: Node, to_node):

        b_path = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature, self.step_size)

        px, py, pyaw, directions, modes, course_lengths = b_path
        if not px:
            return None

        cost = from_node.cost + sum([abs(l) for l in course_lengths])
        new_node = self.Node.create_with_path(px[-1], py[-1], pyaw[-1],
                                              px, py, pyaw,
                                              directions, modes,
                                              cost,
                                              from_node)

        return new_node

    def calc_new_cost(self, from_node, to_node):

        path = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature, self.step_size)
        if not path:
            return float("inf")
        course_lengths = path[-1]
        return from_node.cost + sum([abs(l) for l in course_lengths])

    def get_random_node(self):

        rnd = self.Node(random.uniform(self.min_x, self.max_x),
                        random.uniform(self.min_y, self.max_y),
                        random.uniform(-math.pi, math.pi)
                        )

        return rnd

    def search_best_goal_node(self):
        if len(self.goal_node_list) == 0:
            return None

        def get_cost(node):
            return node.cost

        best_node = min(self.goal_node_list, key=get_cost)

        return best_node

    def generate_final_course(self, goal_node):
        dummy_direction = 0
        dummy_mode = MODE_TO_INT['S']  # Mode of car. Can be straight, left or Right: 'S''L','R'
        path = [[self.end.x, self.end.y, self.end.yaw, dummy_direction, dummy_mode]]
        node = goal_node
        while node.parent:
            for (ix, iy, iyaw, idirection, imode) in zip(reversed(node.path_x), reversed(node.path_y),
                                                         reversed(node.path_yaw),
                                                         reversed(node.path_direction), reversed(node.path_mode)):
                path.append([ix, iy, iyaw, idirection, imode])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw, dummy_direction, dummy_mode])
        path.reverse()
        return np.array(path)


def main(max_iter=50):
    # ====Search Path with RRT====
    obstacleList = [
        [(-1, 6.4), (-1, 1.6), (1, 1.6), (1, 6.4)],
        [(8, 11.5), (8, 3.5), (12, 3.5), (12, 11.5)]
    ]  # list of polygons
    # one polygon looks like: [(x1, y1), (x2,y2), (x3,y3), (x4,y4)]
    #
    # # Set Initial parameters
    start = [4, 2.0, np.deg2rad(90.0)]
    goal = [6.0, 7.0, np.deg2rad(90.0)]

    rrt_star_reeds_shepp = RRTStarReedsShepp(start, goal,
                                             obstacleList,
                                             [0, 0, 12, 20], max_iter=max_iter)
    path = rrt_star_reeds_shepp.planning(animation=True)

    if path:
        end_config = tuple(path[0])
        end_poly = config_to_polygon(end_config)
        x = [x for (x, y) in end_poly]
        y = [y for (x, y) in end_poly]
        plt.fill(x, y)
        plt.show()
    #
    # # # Draw final path
    # if path:  # pragma: no cover
    #     rrt_star_reeds_shepp.draw_graph()
    #     plt.plot([x for (x, y, yaw) in path], [y for (x, y, yaw) in path],
    #              '-r')  # this final print doesn't run in pycharm
    #     plt.plot(10, 10, 'b')  # for testing
    #     plt.grid(True)
    #     plt.pause(0.001)
    #     plt.show()


if __name__ == '__main__':
    main()
