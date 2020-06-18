"""
Visibility Road Map Planner
author: Atsushi Sakai (@Atsushi_twi)
"""

import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import random

#from geometry import Geometry
#from dijkstra_search import DijkstraSearch
from navigation.geometry import Geometry
from navigation.dijkstra_search import DijkstraSearch
from shapely.geometry import Point, Polygon

show_animation = True


class VisibilityRoadMap:

    def __init__(self, robot_radius, do_plot=False):
        self.robot_radius = robot_radius
        self.do_plot = do_plot

    def planning(self, start_x, start_y, goal_x, goal_y, obstacles):

        nodes = self.generate_graph_node(start_x, start_y, goal_x, goal_y,
                                         obstacles)

        road_map_info = self.generate_road_map_info(nodes, obstacles)

        if self.do_plot:
            self.plot_road_map(nodes, road_map_info)
            plt.pause(1.0)

        rx, ry = DijkstraSearch(show_animation).search(
            start_x, start_y,
            goal_x, goal_y,
            [node.x for node in nodes],
            [node.y for node in nodes],
            road_map_info
        )

        return rx, ry

    def generate_graph_node(self, start_x, start_y, goal_x, goal_y, obstacles):

        # add start and goal as nodes
        nodes = [DijkstraSearch.Node(start_x, start_y),
                 DijkstraSearch.Node(goal_x, goal_y, 0, None)]

        # add vertexes in configuration space as nodes
        for obstacle in obstacles:

            cvx_list, cvy_list = self.calc_vertexes_in_configuration_space(
                obstacle.x_list, obstacle.y_list)

            for (vx, vy) in zip(cvx_list, cvy_list):
                nodes.append(DijkstraSearch.Node(vx, vy))

        #for node in nodes:
        #    plt.plot(node.x, node.y, "xr")

        return nodes

    def calc_vertexes_in_configuration_space(self, x_list, y_list):
        x_list = x_list[0:-1]
        y_list = y_list[0:-1]
        cvx_list, cvy_list = [], []

        n_data = len(x_list)

        for index in range(n_data):
            offset_x, offset_y = self.calc_offset_xy(
                x_list[index - 1], y_list[index - 1],
                x_list[index], y_list[index],
                x_list[(index + 1) % n_data], y_list[(index + 1) % n_data],
            )
            cvx_list.append(offset_x)
            cvy_list.append(offset_y)

        return cvx_list, cvy_list

    def generate_road_map_info(self, nodes, obstacles):

        road_map_info_list = []

        for target_node in nodes:
            road_map_info = []
            for node_id, node in enumerate(nodes):
                if np.hypot(target_node.x - node.x,
                            target_node.y - node.y) <= 0.1:
                    continue

                is_valid = True
                for obstacle in obstacles:
                    if not self.is_edge_valid(target_node, node, obstacle):
                        is_valid = False
                        break

                if is_valid:
                    road_map_info.append(node_id)

            road_map_info_list.append(road_map_info)

        return road_map_info_list

    @staticmethod
    def is_edge_valid(target_node, node, obstacle):

        for i in range(len(obstacle.x_list) - 1):
            p1 = Geometry.Point(target_node.x, target_node.y)
            p2 = Geometry.Point(node.x, node.y)
            p3 = Geometry.Point(obstacle.x_list[i], obstacle.y_list[i])
            p4 = Geometry.Point(obstacle.x_list[i + 1], obstacle.y_list[i + 1])

            if Geometry.is_seg_intersect(p1, p2, p3, p4):
                return False

        return True

    def calc_offset_xy(self, px, py, x, y, nx, ny):
        p_vec = math.atan2(y - py, x - px)
        n_vec = math.atan2(ny - y, nx - x)
        offset_vec = math.atan2(math.sin(p_vec) + math.sin(n_vec),
                                math.cos(p_vec) + math.cos(
                                    n_vec)) + math.pi / 2.0
        offset_x = x + self.robot_radius * math.cos(offset_vec)
        offset_y = y + self.robot_radius * math.sin(offset_vec)
        return offset_x, offset_y

    @staticmethod
    def plot_road_map(nodes, road_map_info_list):
        for i, node in enumerate(nodes):
            for index in road_map_info_list[i]:
                plt.plot([node.x, nodes[index].x],
                         [node.y, nodes[index].y], "-b")


class IncrementalVisibilityRoadMap:

    def __init__(self, robot_radius, do_plot=False):
        self.robot_radius = robot_radius
        self.do_plot = do_plot
        self.obs_nodes = []
        self.obs_roadmap_adj = []
        self.obstacles = []


    def addObstacle(self, obstacle):

        # add obstacle
        self.obstacles.append(obstacle)

        # add nodes for each vertex
        cvx_list, cvy_list = self.calc_vertexes_in_configuration_space(obstacle.x_list, obstacle.y_list)
        new_nodes = [DijkstraSearch.Node(vx, vy) for vx,vy in zip(cvx_list, cvy_list)]
        self.obs_nodes.extend(new_nodes)

        #check new edges for intersection with all objects
        for i,node in enumerate(new_nodes):
            self.obs_roadmap_adj.append(self.getValidNodeEdges(node))

        for i in range(len(new_nodes)):
            idx = len(self.obs_nodes)-len(new_nodes)+i
            for n in self.obs_roadmap_adj[idx]:
                self.obs_roadmap_adj[n].append(idx)




        #check old edges for intersection with this object
        for src_id, adj in enumerate(self.obs_roadmap_adj):
            rm = []
            for i,tar_id in enumerate(adj):
                if not self.validEdge(self.obs_nodes[src_id], self.obs_nodes[tar_id]):
                    rm.append(i)
            rm.reverse()
            for i in rm:
                del self.obs_roadmap_adj[src_id][i]


    def getValidNodeEdges(self, src_node):
        #draw a point to each other point and check valid
        node_adj = []
        for node_id, node in enumerate(self.obs_nodes):
            if self.validEdge(src_node, node):
                node_adj.append(node_id)

        return node_adj

    def validEdge(self, src_node, node):
        if np.hypot(src_node.x - node.x, src_node.y - node.y) <= 0.1:
                return False

        for obs in self.obstacles:
            if not self.is_edge_valid(src_node, node, obs):
                return False
        return True

    def planning(self, start_x, start_y, goal_x, goal_y):

        sg_nodes = [DijkstraSearch.Node(start_x, start_y),
                 DijkstraSearch.Node(goal_x, goal_y)]

        sg_edges = [[],[]]


        planNodes = self.obs_nodes.copy() + sg_nodes
        planRoadmap = self.obs_roadmap_adj.copy() + sg_edges

        for node_id in range(len(planRoadmap)):
            if self.validEdge(planNodes[node_id], planNodes[-2]) and node_id != len(planRoadmap)-2:
                planRoadmap[node_id].append(len(planNodes)-2)
                planRoadmap[-2].append(node_id)
            if self.validEdge(planNodes[node_id], planNodes[-1]) and node_id != len(planRoadmap)-1:
                planRoadmap[node_id].append(len(planNodes)-1)
                planRoadmap[-1].append(node_id)

        #print(planNodes)
        #print(planRoadmap)
        #self.plot_road_map(planNodes, planRoadmap)
        #plt.pause(5)

        rx, ry = DijkstraSearch(False).search(
            start_x, start_y,
            goal_x, goal_y,
            [node.x for node in planNodes],
            [node.y for node in planNodes],
            planRoadmap
        )

        return rx, ry


    def calc_vertexes_in_configuration_space(self, x_list, y_list):
        x_list = x_list[0:-1]
        y_list = y_list[0:-1]
        cvx_list, cvy_list = [], []

        n_data = len(x_list)

        for index in range(n_data):
            offset_x, offset_y = self.calc_offset_xy(
                x_list[index - 1], y_list[index - 1],
                x_list[index], y_list[index],
                x_list[(index + 1) % n_data], y_list[(index + 1) % n_data],
            )
            cvx_list.append(offset_x)
            cvy_list.append(offset_y)

        return cvx_list, cvy_list


    @staticmethod
    def is_edge_valid(target_node, node, obstacle):

        for i in range(len(obstacle.x_list) - 1):
            p1 = Geometry.Point(target_node.x, target_node.y)
            p2 = Geometry.Point(node.x, node.y)
            p3 = Geometry.Point(obstacle.x_list[i], obstacle.y_list[i])
            p4 = Geometry.Point(obstacle.x_list[i + 1], obstacle.y_list[i + 1])

            if Geometry.is_seg_intersect(p1, p2, p3, p4):
                return False

        return True

    def calc_offset_xy(self, px, py, x, y, nx, ny):
        p_vec = math.atan2(y - py, x - px)
        n_vec = math.atan2(ny - y, nx - x)
        offset_vec = math.atan2(math.sin(p_vec) + math.sin(n_vec),
                                math.cos(p_vec) + math.cos(
                                    n_vec)) + math.pi / 2.0
        offset_x = x + self.robot_radius * math.cos(offset_vec)
        offset_y = y + self.robot_radius * math.sin(offset_vec)
        return offset_x, offset_y

    @staticmethod
    def plot_road_map(nodes, road_map_info_list):
        for i, node in enumerate(nodes):
            for index in road_map_info_list[i]:
                if index >= len(nodes)-2 or i >= len(nodes)-2:
                    plt.plot([node.x, nodes[index].x],
                             [node.y, nodes[index].y], "-r")
                else:
                    plt.plot([node.x, nodes[index].x],
                             [node.y, nodes[index].y], "-b")



class ObstaclePolygon:

    def __init__(self, x_list, y_list):
        self.x_list = x_list
        self.y_list = y_list

        self.close_polygon()
        self.make_clockwise()

    def make_clockwise(self):
        if not self.is_clockwise():
            self.x_list = list(reversed(self.x_list))
            self.y_list = list(reversed(self.y_list))

    def is_clockwise(self):
        n_data = len(self.x_list)
        eval_sum = sum([(self.x_list[i + 1] - self.x_list[i]) *
                        (self.y_list[i + 1] + self.y_list[i])
                        for i in range(n_data - 1)])
        eval_sum += (self.x_list[0] - self.x_list[n_data - 1]) * \
                    (self.y_list[0] + self.y_list[n_data - 1])
        return eval_sum >= 0

    def close_polygon(self):
        is_x_same = self.x_list[0] == self.x_list[-1]
        is_y_same = self.y_list[0] == self.y_list[-1]
        if is_x_same and is_y_same:
            return  # no need to close

        self.x_list.append(self.x_list[0])
        self.y_list.append(self.y_list[0])

    def minDistanceToVertex(self, px,py):
        d = math.inf
        for x,y in zip(self.x_list,self.y_list):
            d = min(d, math.sqrt( (x-px)**2 + (y-py)**2))
        return d

    def contains_goal(self, goal):
        goal_point = Point(goal[0], goal[1])
        coords = []
        for x,y in zip(self.x_list, self.y_list):
            coords.append(Point(x, y))
        poly = Polygon(coords)
        return goal_point.within(poly)

    def plot(self, clr="-k"):
        plt.plot(self.x_list, self.y_list, clr)



def genRandomRectangle():
    width = random.randrange(5,50)
    height = random.randrange(5,50)
    botLeftX = random.randrange(1,100)
    botRightX = random.randrange(1,100)
    theta = random.random()*2*math.pi

    x = [random.randrange(1,50)]
    y = [random.randrange(1,50)]

    x.append(x[-1]+width)
    y.append(y[-1])

    x.append(x[-1])
    y.append(y[-1]+height)

    x.append(x[-1]-width)
    y.append(y[-1])

    for i in range(4):
        tx = x[i]*math.cos(theta) - y[i]*math.sin(theta)
        ty = x[i]*math.sin(theta) + y[i]*math.cos(theta)
        x[i] = tx
        y[i] = ty

    return ObstaclePolygon(x,y)


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx, sy = 10.0, 10.0  # [m]
    gx, gy = -50.0, 50.0  # [m]

    robot_radius = 5.0  # [m]

    obstacles=[]
    for i in range(4):
        obstacles.append(genRandomRectangle())


    if show_animation:  # pragma: no cover
        plt.plot(sx, sy, "or")
        plt.plot(gx, gy, "ob")
        for ob in obstacles:
            ob.plot()
        plt.axis("equal")
        plt.pause(1.0)

    rx, ry = VisibilityRoadMap(robot_radius, do_plot=show_animation).planning(
        sx, sy, gx, gy, obstacles)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.1)
        plt.show()


if __name__ == '__main__':
    main() 
