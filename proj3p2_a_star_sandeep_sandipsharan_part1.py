#!/usr/bin/env python3

import numpy as np
import time
import math
import pygame
import vidmaker
from sortedcollections import OrderedSet
import heapdict


'''
Github repository - https://github.com/sandipsharan/A_star_algorithm
'''

'''
Visualization Video link - https://drive.google.com/file/d/1J7fFrUmw66ZZ5l-3xftQIBs8zg-AzyZ3/view?usp=share_link
Gazebo Video Link - https://drive.google.com/file/d/1zMZkRd9BUZkixb4Scdb6FKqUckAuu_gh/view?usp=share_link
'''

start_time = time.time()


class A_star:

    # Function to flip the co-ordinate points
    def coords_pygame(self, coords, height):
        return (coords[0], height - coords[1])

    # Function to flip the co-ordinate points and covert them into cm
    def coords_cm_pygame(self, coords, height):
        return (coords[0]*100, height - (coords[1]*100))

    # Function to flip the co-ordinate points for a rectangle
    def rect_pygame(self, coords, height, obj_height):
        return (coords[0], height - coords[1] - obj_height)

    # Function to find the euclidean distance
    def euclidean_distance(self, x1, x2, y1, y2):
        return (np.sqrt((x1-x2)**2 + (y1-y2)**2))

    def create_map(self, d, explored, optimal_path, path):
        pygame.init()
        size = [600, 200]
        screen = pygame.display.set_mode(size)
        pygame.display.set_caption("Weighted A-star")
        video = vidmaker.Video("anime.mp4", late_export=True)
        clock = pygame.time.Clock()
        running = True
        x1, y1 = self.rect_pygame([150-d, 75-d], 200, 125+d)
        x3, y3 = self.rect_pygame([250-d, 0], 200, 125+d)
        x2, y2 = self.rect_pygame([150, 75], 200, 125)
        x4, y4 = self.rect_pygame([250, 0], 200, 125)

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            pygame.draw.rect(screen, "teal", [x1, y1, 15+(2*d), 125+d], 0)
            pygame.draw.rect(screen, "skyblue", [x2, y2, 15, 125], 0)
            pygame.draw.rect(screen, "teal", [x3, y3, 15+(2*d), 125+d], 0)
            pygame.draw.rect(screen, "skyblue", [x4, y4, 15, 125], 0)
            pygame.draw.rect(screen, "teal", [0, 0, d, 200], 0)
            pygame.draw.rect(screen, "teal", [0, 0, 600, d], 0)
            pygame.draw.rect(screen, "teal", [0, 200-d, 600, d], 0)
            pygame.draw.rect(screen, "teal", [600-d, 0, d, 200], 0)
            pygame.draw.circle(
                screen, "teal", self.coords_pygame((400, 110), 200), 50+d)
            pygame.draw.circle(screen, "skyblue",
                               self.coords_pygame((400, 110), 200), 50)
            for l in range(len(explored)):
                pygame.draw.lines(screen, "white", False,
                                  path[explored[l]][1], width=1)
                video.update(pygame.surfarray.pixels3d(
                    screen).swapaxes(0, 1), inverted=False)
                pygame.display.flip()
                clock.tick(500)
            for i in range(len(optimal_path)):
                if optimal_path[i] != initial_state:
                    pygame.draw.lines(screen, "red", False,
                                      path[optimal_path[i]][1], width=3)
                    video.update(pygame.surfarray.pixels3d(
                        screen).swapaxes(0, 1), inverted=False)
                    pygame.display.flip()
                    clock.tick(20)
            running = False
        pygame.display.flip()
        pygame.time.wait(3000)
        pygame.quit()
        video.export(verbose=True)

    def check_obstacles(self, d):
        obstacles = OrderedSet()
        for x in np.arange(0, 6.1, 0.01):
            for y in np.arange(0, 2.1, 0.01):
                if (x >= (1.5 - d) and y >= (0.75-d) and x <= (1.65 + d) and y <= 2):
                    obstacles.add((np.round(x, 2), np.round(y, 2)))
                if (x >= (2.5 - d) and y >= 0 and x <= (2.65 + d) and y <= (1.25 + d)):
                    obstacles.add((np.round(x, 2), np.round(y, 2)))
                if ((x-4)**2 + (y-1.1)**2 - (0.5+d)**2) <= 0:
                    obstacles.add((np.round(x, 2), np.round(y, 2)))
                if (x >= (6-d) or y >= (2-d) or x <= d or y <= d):
                    obstacles.add((np.round(x, 2), np.round(y, 2)))
        return obstacles

    def input_start(self, str):
        while True:
            print("Enter", str, "node (Sample: 10, 10 ): ")
            A = [float(i) for i in input().split(', ')]
            A_1 = (A[0]+0.5, A[1]+1)
            if A_1 in obstacle_space:
                print(
                    "The entered input lies on the obstacles (or) not valid, please try again")
            else:
                return A_1

    def input_cdr(self, str):
        while True:
            if str == 'RPM1' or str == 'RPM2':
                print("Enter", str, "(Sample: Enter a float): ")
                A = float(input())
                return (A*2*math.pi/60)
            elif str == 'start point' or str == 'goal point':
                print("Enter orientation of the", str,
                      "(Sample: Angles in degrees - 30): ")
                A = float(input())
                return A
            elif str == 'clearance':
                print("Enter", str, " in mm (Sample: 100): ")
                A = float(input())
                return A/1000

    def check_conditions(self, X_n, Y_n, X_i, Y_i, T_i, Thetan, cc, ls, vel):
        cost2_go = self.euclidean_distance(
            node_state_g[0], X_n, node_state_g[1], Y_n)
        final_cost = (cc + cost2_go*1.75)
        if Thetan > 360:
            Thetan = Thetan % 360
        elif -360 < Thetan < 0:
            Thetan += 360
        elif Thetan <= -360:
            Thetan = Thetan % 360 + 360
        current_pos = (X_n, Y_n, np.round(Thetan, 2))
        if (current_pos[0], current_pos[1]) not in obstacle_space:
            if current_pos in queue_nodes:
                if queue_nodes[current_pos][0] > final_cost:
                    queue_nodes[current_pos] = final_cost, cost2_go, cc
                    path_dict[current_pos] = (X_i, Y_i, T_i), ls, vel
                    visited_nodes.add(current_pos)
                    return
                else:
                    return
            queue_nodes[current_pos] = final_cost, cost2_go, cc
            path_dict[current_pos] = (X_i, Y_i, T_i), ls, vel
            visited_nodes.add(current_pos)
        return

    def Actions(self, ul, ur, pos, c2c):
        t = 0
        dt = 0.2
        Xn = pos[0]
        Yn = pos[1]
        Thetan = np.deg2rad(pos[2])
        ls = OrderedSet()
        ls.add(self.coords_cm_pygame((Xn, Yn), 200))
        cc = 0
        while t < 1:
            xi = Xn
            yi = Yn
            Xn += 0.5*R*(ul + ur)*np.cos(Thetan)*dt
            Yn += 0.5*R*(ul + ur)*np.sin(Thetan)*dt
            Thetan += (R/L)*(ur-ul)*dt
            t = t + dt
            cc += self.euclidean_distance(xi, Xn, yi, Yn)
            ls.add(self.coords_cm_pygame((Xn, Yn), 200))
        cc += c2c
        velocity = ((0.5*R*(ul + ur)*np.cos(Thetan)),
                    (0.5*R*(ul + ur)*np.sin(Thetan)), ((R/L)*(ur-ul)))
        Xn = np.round(Xn, 2)
        Yn = np.round(Yn, 2)
        Thetan = np.round(Thetan, 2)
        Thetan = np.rad2deg(Thetan)
        if 0 <= Xn <= 6 and 0 <= Yn <= 2:
            self.check_conditions(Xn, Yn, pos[0], pos[1],
                                  pos[2], Thetan, cc, ls, velocity)
        return

    def back_tracking(self, path, pre_queue):
        best_path = []
        path_vel = []
        best_path.append(pre_queue[0])
        parent_node = path[pre_queue[0]][0]
        vel_parent = path.get(pre_queue[0])[2]
        path_vel.append(vel_parent)
        best_path.append(parent_node)
        while parent_node != initial_state:
            vel_parent = path.get(parent_node)
            path_vel.append(vel_parent[2])
            parent_node = path[parent_node][0]
            best_path.append(parent_node)
        best_path.reverse()
        path_vel.reverse()
        print("Path Taken: ")
        for i in best_path:
            print(i)
        return best_path, path_vel

    def a_star(self):
        RPM1 = self.input_cdr('RPM1')
        RPM2 = self.input_cdr('RPM2')
        global action_set, initial_state, node_state_g, closed_list, queue_nodes, visited_nodes, path_dict, obstacle_space, R, L
        action_set = [0, RPM1], [RPM1, 0], [RPM1, RPM1], [0, RPM2], [
            RPM2, 0], [RPM2, RPM2], [RPM1, RPM2], [RPM2, RPM1]
        r = 0.105
        R = 0.033
        L = 0.16
        d = self.input_cdr('clearance')
        obstacle_space = self.check_obstacles((d+r))
        initial_state = self.input_start(
            'Start'), self.input_cdr('start point')
        initial_state = (initial_state[0][0],
                         initial_state[0][1], initial_state[1])
        node_state_g = self.input_start('Goal'), self.input_cdr('goal point')
        node_state_g = (node_state_g[0][0],
                        node_state_g[0][1], node_state_g[1])
        cost = 0
        closed_list = OrderedSet()
        cg = np.sqrt(
            (node_state_g[0]-initial_state[0])**2 + (node_state_g[1]-initial_state[1])**2)
        total_cost = cg + cost
        queue_nodes = heapdict.heapdict()
        path_dict = {}
        visited_nodes = OrderedSet()
        queue_nodes[(initial_state)] = total_cost, cg, cost
        while (len(queue_nodes) != 0):
            queue_pop = queue_nodes.popitem()
            position = queue_pop[0]
            x, y, theta = position
            cc = queue_pop[1][2]
            if (x, y) not in closed_list:
                closed_list.add((x, y))
                if self.euclidean_distance(node_state_g[0], x, node_state_g[1], y) > 0.15:
                    for i in action_set:
                        self.Actions(i[0], i[1], position, cc)
                else:
                    print("Goal reached")
                    back_track, velocity_path = self.back_tracking(
                        path_dict, queue_pop)
                    end_time = time.time()
                    path_time = end_time - start_time
                    print('Time to calculate path:', path_time, 'seconds')
                    self.create_map(d, visited_nodes, back_track, path_dict)
                    return
        print("Path cannot be acheived")


def main():
    astar = A_star()
    astar.a_star()


if __name__ == '__main__':
    main()
