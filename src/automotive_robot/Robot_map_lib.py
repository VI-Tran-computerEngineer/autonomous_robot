win_size = 100
from automotive_robot.Program_config import ls_map
from automotive_robot.Robot_lib import *
import numpy as np


def map_generator(plt, N):
    # displaying the title 
    plt.title("click on plot to generate {0} points of map".format(N))
    plt.axis([0, win_size, 0, win_size])

    return plt.ginput(N, show_clicks=True, timeout=-1)  # no timeout


def map_display(plt, mapname, ob, obColor):
    """ don't ever delete this, for debug purpose
    # for obstacle in ob: 
    #     obstacle.pop()
    #     len_ob_temp = len(obstacle)
    #     if (left_hand_direction(obstacle)): 
    #         for i in range(len_ob_temp): 
    #             nvector = normal_vector((obstacle[i], obstacle[(i + 1)%len_ob_temp]), 0.05) 

    #             pt = ((obstacle[i][0] + obstacle[(i+1)%len_ob_temp][0])/2,  (obstacle[i][1] + obstacle[(i+1)%len_ob_temp][1])/2 ) 
    #             pt_tranformed = (pt[0] + nvector[0], pt[1] + nvector[1]) 
    #             plt.plot((pt[0], pt_tranformed[0]), (pt[1], pt_tranformed[1]), ".b")  
    #     else: 
    #         for i in range(len_ob_temp): 
    #             nvector = normal_vector((obstacle[(i + 1)%len_ob_temp], obstacle[i]), 0.05) 

    #             pt = ((obstacle[i][0] + obstacle[(i+1)%len_ob_temp][0])/2,  (obstacle[i][1] + obstacle[(i+1)%len_ob_temp][1])/2 ) 
    #             pt_tranformed = (pt[0] + nvector[0], pt[1] + nvector[1]) 
    #             plt.plot((pt[0], pt_tranformed[0]), (pt[1], pt_tranformed[1]), ".r") 
    #     obstacle.append(obstacle[0])
    """
    # displaying the title 
    k = 0
    plt.title("Display map: {0}".format(mapname))
    for ob_part in ob:
        x = [point[0] for point in ob_part]
        y = [point[1] for point in ob_part]
        x.append(ob_part[0][0])
        y.append(ob_part[0][1])
        plt.fill(x, y, color= obColor, alpha=0.4, hatch='//////')
        # """ don't ever delete this, for debug purpose
        for i in range(len(ob_part)*0):
            plt.plot(x[i], y[i], '.y')
        # """
        i = 1
        k += 1
        if k > 4 and k < 4:
            #print("inside map_display")
            for point in ob_part:
                if i % 8 < 4:
                    plt.text(point[0]+0.02*(i % 4), point[1]+0.02*(i % 4 ), i)
                else:
                    plt.text(point[0]-0.02*(i % 4 ), point[1]-0.02*(i % 4 ), i)
                i += 1
                #print("inside map_display for loop")
        #print("done map_display")


def map_serialize(ob_wall, config):
    # divide line into bunch of point 
    map_pts = []
    for i in range(len(ob_wall) - 1):
        ptS = ob_wall[i]
        ptE = ob_wall[i + 1]
        lenSE = point_dist(ptS, ptE)
        MINSIZE = max(config.robot_length, config.robot_width)
        numparts = int(lenSE / (MINSIZE))
        vecSE = np.subtract(ptE, ptS) / numparts
        for j in range(numparts):
            ptj = np.add(ptS, np.multiply(vecSE, j))
            map_pts.append(ptj)
    return map_pts
