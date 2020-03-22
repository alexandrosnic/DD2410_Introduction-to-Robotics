#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# {Alexandros Nicolaou}
# {19930331-T675}
# {alenic@kth.se}

from dubins import *
from math import *
from numpy import *
from operator import itemgetter

car=Car()



def Find_Node(car, dubins_path, visited):
    
    explored_Nodes = []
    new_Nodes = [[car.x0, car.y0, 0, [], [0], sqrt((car.x0 - car.xt)**2 + (car.y0 - car.yt)**2)]]
    
    while len(new_Nodes) > 0:
        x, y, theta, controls, times, dist = new_Nodes[0]
        del new_Nodes[0]

        if sqrt((x - car.xt)**2 + (y - car.yt)**2) <= 0.25:
            return controls, times


        visited.append([round(x, 1), round(y, 1)])
        for phi in [-pi/4, 0, pi/4]:

            evaluate_node, new_x, new_y, new_theta, controls1, times1, cost = Find_Path(x, y, phi, theta, car, reset_List(controls), reset_List(times))
            
            check_node = [round(new_x,1), round(new_y,1), round(new_theta,1)]
            if evaluate_node:
                if not check_node in explored_Nodes:
                    
                    new_Nodes.append([new_x, new_y, new_theta, controls1, times1, cost])
                    dubins_path.append(phi)
                    explored_Nodes.append(check_node)
                    
            sorted(new_Nodes, key=itemgetter(1))
    return [],[0]

def check_obstacle(x,y,car):
    for obstacle in car.obs:
        if sqrt((obstacle[0] - x)**2 + (obstacle[1] - y)**2) <= obstacle[2] + 0.1:
            return True
    return False



def Find_Path(x,y,phi,theta,car,controls,times):
    cost = 0
    for i in range(100 if phi == 0 else 157):
        dt=0.01
        x, y, theta = step(car, x, y, theta, phi)
        while theta >= pi:
            theta -= 2*pi
        while theta <= -2*pi:
            theta += pi  
        
        controls.append(phi)

        times.append(times[-1] + dt)

        safe_Obstacles = check_obstacle(x,y,car)

        if (car.xlb <= x <= car.xub and car.ylb <= y <= car.yub ):
            in_bounds = False
        else:
            in_bounds = True

            
        if safe_Obstacles or in_bounds:
            evaluate = False
            new_x = 0
            new_y = 0
            new_theta = 0
            new_controls = controls
            new_times = times
            cost = inf
            return evaluate, new_x, new_y, new_theta, new_controls, new_times, cost
        if sqrt((x - car.xt)**2 + (y - car.yt)**2) <= 0.25:
            evaluate = True
            new_x = x
            new_y = y
            new_theta = theta
            new_controls = controls
            new_times = times
            cost = 0
            return evaluate, new_x, new_y, new_theta, new_controls, new_times, cost

        else:
            evaluate = True
            new_x = x
            new_y = y
            new_theta = theta
            new_controls = controls
            new_times = times
    cost = sqrt((new_x - car.xt)**2 + (new_y - car.yt)**2)
    return evaluate, new_x, new_y, new_theta, new_controls, new_times, cost



def reset_List(listt):
    updateList = []
    for x in listt:
        updateList.append(x)
    return updateList

def solution(car):
    controls=[0]
    times=[0,0.01]
    controls, times = Find_Node(car, [], [])
    return controls, times