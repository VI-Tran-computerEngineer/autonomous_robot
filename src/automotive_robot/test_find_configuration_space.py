"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
"""
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os
import sys

from Robot_lib import *
from Robot_paths_lib import *
from Robot_draw_lib import *
from Robot_sight_lib import *
from Robot_map_lib import *
from Robot_world_lib import *
from Robot_csv_lib import *
from Robot_goal_lib import *
from Program_config import *
from Robot_control_panel import *
plt.figure(figsize=(5,5))


obstacles=[]

obstacles.append([
(237,208),(236,209),(226,209),(225,210),(223,210),(222,211),(223,211),(224,210),(225,210),(226,209),(230,209),(231,210),(229,212),(226,212),(225,213),(223,213),(225,213),(226,212),(229,212),(230,211),(231,211),(232,212),(237,212),(238,211),(237,211),(236,210),(236,209),(237,208)
])




# obstacle_in = [
#             (0,0), 
#             (10,0), 
#             (2.5,2.5),
#             (0,12),
#             (0,0)
# ]

# obstacles.append(obstacle_in)

# obstacles.append([(230,206),
# (229,207),
# (224,207),
# (223,208),
# (222,208),
# (222,210),
# (229,210),
# (230,209),
# (236,209),
# (237,208),
# (238,208),
# (238,206),
# (230,206)])
# obstacles.append([
#     (218,174),
# (218,190),
# (219,191),
# (221,191),
# (221,174),
# (218,174)
# ])
# obstacles.append([
#     (232,143),
# (228,147),
# (228,148),
# (227,149),
# (226,149),
# (225,150),
# (225,151),
# (223,153),
# (223,155),
# (226,158),
# (227,158),
# (232,163),
# (232,166),
# (233,166),
# (234,167),
# (237,167),
# (237,165),
# (235,163),
# (235,161),
# (231,157),
# (230,157),
# (227,154),
# (226,154),
# (224,152),
# (232,144),
# (232,143)
# ])


def preProcessObstacles(obs):
    
    exilon = 0.001
    for obstacle in obs:
        obstacle.pop()
        i = 0
        while (i < len(obstacle)):
            i_iplus = point_dist(obstacle[i], obstacle[(i+1)%len(obstacle)])
            i_iminus = point_dist(obstacle[i], obstacle[i-1])
            iminus_iplus = point_dist(obstacle[i-1], obstacle[(i+1)%len(obstacle)])

            if abs(i_iminus + i_iplus - iminus_iplus) < exilon:
                print("preProcessObstacles case 1")
                obstacle.pop(i)

            elif abs(iminus_iplus + i_iminus - i_iplus) < exilon:
                print("preProcessObstacles case 3")
                normalVector = normal_vector((obstacle[i], obstacle[(i+1)%len(obstacle)]), 0.01)
                new = tuple(np.subtract(obstacle[i], normalVector))
                obstacle.insert(i + 1, new)
                obstacle.insert(i + 2, obstacle[i-1])
                i += 3

            elif abs(iminus_iplus + i_iplus - i_iminus) < exilon:
                print("preProcessObstacles case 2")
                normalVector = normal_vector((obstacle[i], obstacle[(i+1)%len(obstacle)]), 0.01)
                new = tuple(np.subtract(obstacle[i], normalVector))
                obstacle.insert(i + 1, new)
                i += 2
            
            else:
                i += 1
        
        if (len(obstacle) > 0):
            obstacle.append(obstacle[0])  

def is_i_inside_polygon_2(obstacle, is_pt, i, j):
    polygon_2 = [is_pt]
    polygon_2.extend(copy.deepcopy(obstacle[i+1: j+1]))
    polygon_2.append(is_pt)

    # check if i_th point of obstacle is inside polygon_2
    alpha = 0 # (radian) goc hop voi truc x, cung chieu duong luong giac
    adjustDegree = 0.01 # (radian)
    reUpdateOtherPoint = False
    while True:
        vector = [math.cos(alpha), math.sin(alpha)]
        lineSegmentBoundaryPoint = (obstacle[i][0] + vector[0]*384*1.414, obstacle[i][1] + vector[1]*384*1.414)
        
        #plt.plot(  (mid_pt_transformed[0], lineSegmentBoundaryPoint[0]), (mid_pt_transformed[1], lineSegmentBoundaryPoint[1]), '-m')

        m = 0
        count= 0
        length = len(polygon_2) - 1
        while m < length:
            if inside_ls(polygon_2[m], [obstacle[i], lineSegmentBoundaryPoint] ) or inside_ls(polygon_2[m+1], [obstacle[i], lineSegmentBoundaryPoint] ):
                alpha += adjustDegree
                reUpdateOtherPoint = True
                break
            if line_across(    [obstacle[i], lineSegmentBoundaryPoint]  ,    [polygon_2[m], polygon_2[m+1]]   ):   
                count += 1
            m += 1

        if reUpdateOtherPoint:
            reUpdateOtherPoint = False
            continue

        if count % 2 == 0:
            return False
        else:
            return True

def popElementsOfRing(start, end, ring):
    if end > len(ring)-1 or start > len(ring) - 1:
        print("popElementsOfRing: wrong index range: start, end = ", start, end)
        return
    if start <= end:
        if end == len(ring)-1:
            del ring[start:]
        else:
            del ring[start:(end+1)]
    else:
        del ring[start:]
        del ring[:end+1]

def postProcessing(obs):
    for obstacle in obs:
        i = 0
        first_time = 1
        count0 = 0
        count1 = 0
        count2 = 0
        while (i < len(obstacle)-3):
            # print("In reArrangePointPosition")
            j = i + 2   
            if i != 0:
                first_time = 0
            while (j < len(obstacle) - 1 - first_time):
                count = 0
                if inside_ls( obstacle[i], (obstacle[j], obstacle[j+1])):
                    count += 1
                
                if inside_ls(obstacle[i+1], (obstacle[j], obstacle[j+1])):
                    count += 1

                if inside_ls(obstacle[j], (obstacle[i], obstacle[i+1])):
                    count += 1
                    
                if inside_ls(obstacle[j+1], (obstacle[i], obstacle[i+1])):
                    count += 1
                
                if count == 0:
                    is_pt = line_across(  (obstacle[i], obstacle[i+1]), (obstacle[j], obstacle[j+1])  )
                    if is_pt:
                        if is_i_inside_polygon_2(obstacle, is_pt, i, j):
                            popElementsOfRing(j+1, i, obstacle)
                            obstacle.insert(0, is_pt)
                            obstacle.append(is_pt)
                            i = 0
                        else:
                            for m in range(j - i):
                                obstacle.pop(i+1)
                            obstacle.insert(i+1, is_pt)

                        count0 += 1
                        j = i + 2
                    else:
                        j += 1


                elif count == 1:
                    print("i: ", i, " j:", j)
                    if inside_ls(obstacle[i], [obstacle[j], obstacle[j+1]]) or inside_ls(obstacle[j+1], [obstacle[i], obstacle[i+1]]):
                        print("in case is_pt is i or j+1")
                        for m in range(j - i):
                            obstacle.pop(i+1)
                    elif inside_ls(obstacle[i+1], [obstacle[j], obstacle[j+1]]):
                        print("in case is_pt is i+1")
                        for m in range(j - i - 1):
                            obstacle.pop(i+2)
                    else:
                        print("in case is_pt is j")
                        for m in range(j - i - 1):
                            obstacle.pop(i+1)
                            
                    j = i + 2
                    count1 += 1

                elif count == 2:
                    is_pt = line_across(  (obstacle[i], obstacle[i+1]), (obstacle[j], obstacle[j+1])  )
                    if is_pt and point_dist( is_pt, obstacle[j+1]) < 0.001:
                        for m in range(j - i):
                            obstacle.pop(i+1)
                        j = i + 2
                        count2 += 1
                    else:
                        print("2 doan trung lap")
                        j += 1

                else:
                    j += 1
            i += 1
        
        print("count0: ", count0, "   count1: ", count1, "   count2: ", count2)

preProcessObstacles(obstacles)
# map_display(plt, "After preprocessing!", obstacles, 'c')
# plt.grid(True)
# plt.show()


cspaces = find_configure_space_update_2(obstacles, 3.0+0.0045)
plt.subplot(1,2,1)
plt.grid(True)

print("User Current Version:-", sys.version)
map_display(plt, "", cspaces, 'k')
map_display(plt, "After Phong To!", obstacles, 'c')

postProcessing(cspaces)

plt.subplot(1,2,2)

map_display(plt, "", cspaces, 'k')
map_display(plt, "After PostProcess!", obstacles, 'c')


plt.grid(True)
plt.show()