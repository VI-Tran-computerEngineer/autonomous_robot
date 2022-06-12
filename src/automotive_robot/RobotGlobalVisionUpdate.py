#!/usr/bin/env python
import os
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import rospy
import threading
from std_msgs.msg import String
from tf.msg import tfMessage
import time
import sys
import getpass
import copy
import matplotlib as mpl


from automotive_robot.Robot_lib import intersection
from automotive_robot.Robot_paths_lib import *
from automotive_robot.Robot_draw_lib import *
from automotive_robot.Robot_sight_lib import *
from automotive_robot.Robot_map_lib import *
from automotive_robot.Robot_world_lib import *
from automotive_robot.Robot_csv_lib import *
from automotive_robot.Robot_goal_lib import *
from automotive_robot.Program_config import *
from automotive_robot.Robot_control_panel import *

from automotive_robot.msg import Point
from automotive_robot.msg import Path

config = Config()

pub = rospy.Publisher('cmd_moving', Path, queue_size=1000)

# set configuration of robot
config.robot_type = RobotType.circle
robot_vision = config.robot_vision

# set same window size to capture pictures
plt.figure(figsize=(6, 6))

USER = getpass.getuser()

# get user input
menu_result = menu()
goal = [menu_result.gx, menu_result.gy]

no_way_to_goal = False  # there is no way to reach goal

# traversal sights to draw visible visited places
traversal_sights = []

# global active open points
g_active_open_pts = []

# visibility Graph which contains information of visited places
visibility_graph = graph_intiailze()

pythonChosenPoints = []

# visited path
visited_path = []

""" Convert a slam coordinate to a csv coordinate
20 units in csv = 1 unit in slam
x_csv belongs to range [0, 384]
y_csv belongs to range [0, 384]
x_slam belongs to range [-10 -> 9.2]
y_slam belongs to range [-10 -> 9.2]
slam (0, 0) -> csv (200,184) """


def convert_position_from_slam_to_csv(position):
    result = [0, 0]
    result[0] = (position[0])*20 + 200
    result[1] = -position[1]*20 + 184
    return (result[0], result[1])


""" Convert a csv coordinate to a slam coordinate"""


def convert_position_from_csv_to_slam(position):
    result = [0, 0]
    result[0] = (position[0] - 200)/20
    result[1] = -(position[1] - 184)/20
    return (result[0], result[1])


""" Pop a range of elements in a ring faction, that is 
treat the to-be-processed array as a ring(which has neither start nor end) """


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


""" check if wheather point i is inside a obstacle for postprocessing purpose"""


def is_i_inside_polygon_2(obstacle, is_pt, i, j):
    polygon_2 = [is_pt]
    polygon_2.extend(copy.deepcopy(obstacle[i+1: j+1]))
    polygon_2.append(is_pt)

    # check if i_th point of obstacle is inside polygon_2
    alpha = 0  # (radian) goc hop voi truc x, cung chieu duong luong giac
    adjustDegree = 0.01  # (radian)
    reUpdateOtherPoint = False
    while True:
        vector = [math.cos(alpha), math.sin(alpha)]
        lineSegmentBoundaryPoint = (
            obstacle[i][0] + vector[0]*384*1.414, obstacle[i][1] + vector[1]*384*1.414)

        #plt.plot(  (mid_pt_transformed[0], lineSegmentBoundaryPoint[0]), (mid_pt_transformed[1], lineSegmentBoundaryPoint[1]), '-m')

        m = 0
        count = 0
        length = len(polygon_2) - 1
        while m < length:
            if inside_ls(polygon_2[m], [obstacle[i], lineSegmentBoundaryPoint]) or inside_ls(polygon_2[m+1], [obstacle[i], lineSegmentBoundaryPoint]):
                alpha += adjustDegree
                reUpdateOtherPoint = True
                break
            if line_across([obstacle[i], lineSegmentBoundaryPoint],    [polygon_2[m], polygon_2[m+1]]):
                count += 1
            m += 1

        if reUpdateOtherPoint:
            reUpdateOtherPoint = False
            continue

        if count % 2 == 0:
            return False
        else:
            return True


""" Preprocess all obstacles, the obstacles got will have 
structure incompatible to configure, preprocess obstacles 
make obstacles become compatible to be configured """


def preProcessObstacles(obs):
    exilon = 0.001
    for obstacle in obs:
        obstacle.pop()
        i = 0
        while (i < len(obstacle)):
            i_iplus = point_dist(obstacle[i], obstacle[(i+1) % len(obstacle)])
            i_iminus = point_dist(obstacle[i], obstacle[i-1])
            iminus_iplus = point_dist(
                obstacle[i-1], obstacle[(i+1) % len(obstacle)])

            if abs(i_iminus + i_iplus - iminus_iplus) < exilon:
                print("preProcessObstacles case 1")
                obstacle.pop(i)

            elif abs(iminus_iplus + i_iminus - i_iplus) < exilon:
                print("preProcessObstacles case 3")
                normalVector = normal_vector(
                    (obstacle[i], obstacle[(i+1) % len(obstacle)]), 0.01)
                new = tuple(np.subtract(obstacle[i], normalVector))
                obstacle.insert(i + 1, new)
                obstacle.insert(i + 2, obstacle[i-1])
                i += 3

            elif abs(iminus_iplus + i_iplus - i_iminus) < exilon:
                print("preProcessObstacles case 2")
                normalVector = normal_vector(
                    (obstacle[i], obstacle[(i+1) % len(obstacle)]), 0.01)
                new = tuple(np.subtract(obstacle[i], normalVector))
                obstacle.insert(i + 1, new)
                i += 2

            else:
                i += 1

        if (len(obstacle) > 0):
            obstacle.append(obstacle[0])


""" After configuring obstacles, some obstacles edges will be overlap each other,
postProcessing will """


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
                if inside_ls(obstacle[i], (obstacle[j], obstacle[j+1])):
                    count += 1

                if inside_ls(obstacle[i+1], (obstacle[j], obstacle[j+1])):
                    count += 1

                if inside_ls(obstacle[j], (obstacle[i], obstacle[i+1])):
                    count += 1

                if inside_ls(obstacle[j+1], (obstacle[i], obstacle[i+1])):
                    count += 1

                if count == 0:
                    is_pt = line_across(
                        (obstacle[i], obstacle[i+1]), (obstacle[j], obstacle[j+1]))
                    if is_pt:
                        if is_i_inside_polygon_2(obstacle, is_pt, i, j):
                            popElementsOfRing(j+1, i, obstacle)
                            obstacle.insert(0, is_pt)
                            obstacle.append(is_pt)
                            i = 0
                            first_time = 1
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
                    is_pt = line_across(
                        (obstacle[i], obstacle[i+1]), (obstacle[j], obstacle[j+1]))
                    if is_pt and point_dist(is_pt, obstacle[j+1]) < 0.001:
                        print("i:", i)
                        print("j:", j)
                        if is_i_inside_polygon_2(obstacle, is_pt, i, j):
                            popElementsOfRing(j+1, i, obstacle)
                            obstacle.append(is_pt)
                            i = 0
                            first_time = 1
                        else:
                            popElementsOfRing(i+1, j, obstacle)

                        j = i + 2
                        count2 += 1
                    else:
                        print("2 doan trung lap")
                        j += 1

                else:
                    j += 1
            i += 1

        print("count0: ", count0, "   count1: ", count1, "   count2: ", count2)


""" Get current position from SLAM mapping into a file for robot_motion node to read """


def updateSlamCposForCpp(msg):
    while True:
        try:
            cmd = "timeout 1.75s rosrun tf tf_echo /map /imu_link > ~/catkin_ws/src/automotive_robot/src/automotive_robot/logSlamOdomCpp.log"
            os.system(cmd)
            f = open(
                f'/home/{USER}/catkin_ws/src/automotive_robot/src/automotive_robot/logSlamOdomCpp.log', 'r')
            # the next 4 lines are for catching exception.
            last_line = f.readlines()[-4]
            last_line = last_line.split("[")[1]
            last_line = last_line.split(', ')
            f.close()
            break
        except:
            print("\nGet Slam Odom FOR CPP failed!!!!!!!!\n")
            time.sleep(1)
            continue


""" draw all information that robot has gotten """


def show_animation(ob, world_name, center, local_open_pts, skeleton_path, asp, next_pt, critical_ls, r_goal, s_goal, obColor):
    ##############################################
    # for stopping simulation with the esc key.
    ##############################################
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

    ##############################################
    # draw world and map
    ##############################################

    if show_world and world_name is not None:
        world_display(plt, mpimg, world_name)

    # draw map obstacles
    if show_map:
        # print(ob)
        if world_name is None:  # not None:
            map_display(plt, ".csv", ob, obColor)
        # else:
        #     map_display(plt, map_name, ob)

    # show_traversalSights
    if show_traversalSights:
        for local in traversal_sights:
            lcenter = local[0]  # center of robot at local
            lc_sight = local[1]  # closed sight at local
            lo_sight = local[2]  # open sight at local
            plot_vision(plt, lcenter[0], lcenter[1],
                        robot_vision, lc_sight, lo_sight)

    if show_robot:
        plot_robot(plt, center[0], center[1], 0, config)

    if show_goal:
        plot_goal(plt, goal, r_goal, s_goal)

    # plot robot's vision at local (center)
    #plot_vision(plt, center[0], center[1], robot_vision, closed_sights, open_sights)

    if show_local_openpt and len(local_open_pts) > 0:
        plot_points(plt, local_open_pts, ls_lopt)

    if show_active_openpt and len(g_active_open_pts) > 0:
        plot_points(plt, g_active_open_pts, ls_aopt)

    if show_visibilityGraph:
        plot_visibilityGraph(plt, visibility_graph, ls_vg)

    if show_visitedPath:
        plot_paths(plt, visited_path, ls_vp, ls_goingp)

    if show_sketelonPath:
        plot_lines(plt, skeleton_path, ls_sp)

    if show_approximately_shortest_path:
        plot_lines(plt, asp, ls_asp)

    if show_critical_line_segments:
        plot_critical_line_segments(plt, critical_ls, ls_cls)

        # display next point if existing
    if show_next_point:
        if len(next_pt) > 0:
            plot_point(plt, next_pt, ls_nextpt)

    # to set equal make sure x y axises are same resolution

    plt.axis("equal")
    plt.axis([100, 284, 284, 100])
    plt.xticks(np.arange(0, 384, 20))
    plt.yticks(np.arange(4, 384, 20))
    plt.grid(True)


""" check wheather if a point is inside an obstacle """


def is_inside_obstacles(pt, obstacles):
    for obstacle in obstacles:
        j = 0
        count = 0
        length = len(obstacle) - 1
        while j < length:
            if line_across([pt, [0, 0]],    [obstacle[j], obstacle[j+1]]):
                count += 1
            j += 1
        if (count % 2 == 1):
            print("current position is inside vat can phong to!")
            return True
    return False


""" Extract relative goal value in the message received from robot_motion node
through topic update_Vision """


def extract_relative_goal_in_message(msg):
    temp = msg.data.split(", ")
    relative_goal = [float(temp[0][1:]), float(temp[1][0:-1])]
    print("updateGlobalVisionCallBack: got relative goal = ", relative_goal)
    return convert_position_from_slam_to_csv(relative_goal)


""" Get current position from SLAM mapping into a file for robotGlobalVisionUpdate to read  """


def get_robot_current_position_from_slam():
    while True:
        try:
            cmd = "timeout 1.75s rosrun tf tf_echo /map /imu_link > ~/catkin_ws/src/automotive_robot/src/automotive_robot/logSlamOdom.log"
            os.system(cmd)
            f = open(
                f'/home/{USER}/catkin_ws/src/automotive_robot/src/automotive_robot/logSlamOdom.log', 'r')
            last_line = f.readlines()[-4]
            last_line = last_line.split("[")[1]
            last_line = last_line.split(', ')
            f.close()
            break
        except:
            print("Get Slam Odom failed!!!!!!!!")
            time.sleep(1)
            continue

    cpos = [0, 0]
    cpos[0] = float(format(float(last_line[0]), '.5f'))
    cpos[1] = float(format(float(last_line[1]), '.5f'))

    print("\ncurrent position: ", cpos)

    cpos = (convert_position_from_slam_to_csv(cpos))

    return cpos


""" The number of points in asp is very high (normally 30 -> 60 points), 
this function reduces points in asp that are very close to other points """


def reduce_close_points_in_asp():
    global asp

    tempVar = 0
    while tempVar < len(asp) - 2:
        if (point_dist(asp[tempVar], asp[tempVar+1]) < 4):
            asp.pop(tempVar+1)
            print("Has popped 1 point from asp khoang cach gan nhau")
        elif (tempVar < len(asp) - 3 and point_dist(asp[tempVar], asp[tempVar+2]) < 4):
            asp.pop(tempVar+2)
            asp.pop(tempVar+1)
            print("Has popped 2 points from asp khoang cach gan nhau")
        else:
            tempVar += 1


""" Assume 3 points A, B, C. If A, B, C belong to the same line and B lies between A and C.
This function will removes B, this function has not yet used because the 
distance between A and C might be very large (the larger the distance, the greater the moving error is) """


def reduce_points_belong_to_same_line_in_asp():
    global asp
    tempVar = 0
    print("asp after pop cac diem gan nhau: ", asp)
    while tempVar < len(asp) - 2 and len(asp) > 3:
        vector12 = [asp[tempVar+1][0] - asp[tempVar]
                    [0], asp[tempVar+1][1] - asp[tempVar][1]]
        vector23 = [asp[tempVar+2][0] - asp[tempVar + 1]
                    [0], asp[tempVar+2][1] - asp[tempVar + 1][1]]
        cosAlpha = (vector12[0]*vector23[0] + vector12[1]*vector23[1]) / (math.sqrt(vector12[0]*vector12[0] +
                                                                                    vector12[1]*vector12[1]) * math.sqrt(vector23[0]*vector23[0] + vector23[1]*vector23[1]))

        if (cosAlpha >= 1):
            alpha = 0
        else:
            alpha = math.acos(cosAlpha)

        if (alpha < 5/180*math.pi):
            asp.pop(tempVar + 1)
            print("Has popped 1 point from asp 3 diem thang hang voi alpha: ",
                  alpha/math.pi*180, " degree")
        else:
            tempVar = tempVar + 1


""" Publish asp to topic cmd_moving """


def publish_asp_to_robot_motion_node(asp):
    message = []
    for i in range(len(asp)):
        temp = Point()
        temp.x = (asp[i][0]-200)/20
        temp.y = -(asp[i][1]-184)/20
        message.append(temp)
    print("Done calculate path")
    pub.publish(message)


""" Main call back function, called whenever a message is sent on topic update_vison by
robot motion node """


def updateGlobalVisionCallBack(msg):
    # global varibles
    global no_way_to_goal
    global g_active_open_pts
    global visibility_graph
    global ranks_new
    global skeleton_path
    global flag_plot
    global semaphore
    global cpos
    global next_pt
    global filenum
    global pythonChosenPoints
    global Done
    global goal
    global GotGoalRelativeCoordinate

    print("\n----------------------------------------------------------------------------------------------------------------------------------------")
    print("updateGlobalVisionCallBack: received message = " + msg.data)

    # extract relative goal in received message from robot_motion node
    if not GotGoalRelativeCoordinate:
        GotGoalRelativeCoordinate = True
        goal = extract_relative_goal_in_message(msg)
    else:
        # Waiting for SLAM to stabilize
        time.sleep(9.0)

    # clear graph
    flag_plot = -1
    semaphore.release()

    # get robot current position using SLAM mapping localization.
    cpos = get_robot_current_position_from_slam()

    # semaphore.acquire()  # don't ever delete this, to catch obstacle csv file to debug pre, post processing and configure space (debug command 1)

    cmd = 'rosrun map_server map_saver -f ./vision'
    os.system(cmd)
    world_name = "vision.pgm"

    # read world map then get obstacles information
    read_map_from_world(world_name)

    #
    obsBeforeConfig = read_map_csv(world_name + ".csv")

    #
    preProcessObstacles(obsBeforeConfig)
    print("Has finished preprocess!")

    #
    ob = copy.deepcopy(obsBeforeConfig)

    #
    ranks_new = []
    skeleton_path = []
    center = (cpos[0], cpos[1])

    #
    ob = find_configure_space_update_2(ob, config.robot_radius)
    print("Has finished configure obstacle!")

    #
    postProcessing(ob)
    print("Has finished post processing!")

    #
    if is_inside_obstacles(center, ob):
        pointUsedForScanArount = copy.deepcopy(pythonChosenPoints[-1])
    else:
        pointUsedForScanArount = copy.deepcopy(center)

    # scan to get sights at local
    closed_sights, open_sights = scan_around(
        pointUsedForScanArount, robot_vision, ob, goal)

    # check if the robot saw or reach the given goal
    r_goal, s_goal = check_goal(
        center, goal, config, robot_vision, closed_sights)
    print("r_goal: ", r_goal, "\ns_goal: ", s_goal)

    # initial local open points as empty
    local_open_pts = []
    if not r_goal and not s_goal:
        # get local open points
        local_open_pts = get_local_open_points(open_sights)

        # check whether local open points are active
        l_active_open_pts = get_active_open_points(local_open_pts, traversal_sights,  # khong quan tam center va goal
                                                   robot_vision, center, goal)

        # Used to remove wrong local active points whose trajectory line segment cross an obstacle
        # Please debug it =)). I believe it can work, for somehow it hasn't removed wrong local active points.
        wrongIdx = 0
        for localPoint in l_active_open_pts:
            removed = False
            for obstacle in ob:
                i = 0
                length = len(obstacle) - 1
                while i < length:
                    if line_across([center, localPoint], [obstacle[i], obstacle[i+1]]):
                        print("local active open points before delete:",
                              l_active_open_pts)
                        np.delete(l_active_open_pts, wrongIdx, 0)
                        print("Has removed wrong open point")
                        print("local active open points after delete:",
                              l_active_open_pts)
                        removed = True
                        break
                    i += 1

                if removed:
                    break
            wrongIdx += 1

        # take 5 digits after floating point for compatible format.
        for point in l_active_open_pts:
            point[0] = float(format(float(point[0]), '.5f'))
            point[1] = float(format(float(point[1]), '.5f'))

        # Ranking new active openPts then stack to global set.
        if len(l_active_open_pts) > 0:
            ranks_new = np.array([ranking(center, pt, goal)
                                  for pt in l_active_open_pts])

        # stack local active open point to global set
        g_active_open_pts = store_global_active_points(
            g_active_open_pts, l_active_open_pts, ranks_new)

        # add new active open points to graph_insert
        if (type(next_pt) == type(None)):
            graph_add_lOpenPts(visibility_graph, center, l_active_open_pts)
        else:
            graph_add_lOpenPts(visibility_graph, copy.deepcopy(
                next_pt), l_active_open_pts)

            nextPtNeighbours = copy.deepcopy(
                visibility_graph[tuple((next_pt[0], next_pt[1]))])
            for neighbour in nextPtNeighbours:
                updateIx = visibility_graph[neighbour].index(
                    tuple((next_pt[0], next_pt[1])))
                visibility_graph[neighbour][updateIx] = copy.deepcopy(cpos)

            visibility_graph[cpos] = copy.deepcopy(nextPtNeighbours)
            visibility_graph.pop(tuple((next_pt[0], next_pt[1])))

        # pick next point to make a move
        picked_idx, next_pt = pick_next(g_active_open_pts)
        if picked_idx != -1:
            # find the shortest skeleton path from current position (center) to next point
            skeleton_path = BFS_skeleton_path(
                visibility_graph, tuple(center), tuple(next_pt))

            # then remove picked point from active global open point
            g_active_open_pts = np.delete(
                g_active_open_pts, picked_idx, axis=0)
        else:
            print("No way to reach the goal!")
            no_way_to_goal = True

    else:
        next_pt = goal
        # find the shortest path from center to next point
        skeleton_path = [center, goal]
        Done = True

    if not r_goal:
        # record the sight
        traversal_sights.append([center, closed_sights, open_sights])

        if print_traversalSights:
            print("traversal_sights:", traversal_sights)

        global asp
        asp, critical_ls = approximately_shortest_path(
            skeleton_path, traversal_sights, robot_vision)

        print("\nasp before reduction: ", asp, "\n")

        #
        reduce_close_points_in_asp()

        print("\nasp after reduction: ", asp, "\n")

        #
        publish_asp_to_robot_motion_node(asp)

        # recode visited path
        visited_path.append(asp)
        pythonChosenPoints.append(copy.deepcopy(asp[-1]))

    else:
        Done = True
        print("Goal!!")
        return

    if show_animation:
        show_animation(obsBeforeConfig, None, center, local_open_pts, skeleton_path, asp if not Done else skeleton_path,
                       next_pt, critical_ls, r_goal, s_goal, obColor='c')
        map_display(plt, ".csv", ob, 'k')

        # draw devian line segment from expected point to go to and robot exact position.
        for i in range(len(traversal_sights)-1):
            # the color of the line is solid yellow (-y)
            plot_line(plt, [traversal_sights[i+1][0],
                            pythonChosenPoints[i]], '-y')

        # calculate devian from expected point to go to and robot exact position.
        if len(traversal_sights) > 1:
            currentDevian = point_dist(traversal_sights[len(
                traversal_sights)-1][0], pythonChosenPoints[len(traversal_sights)-2])

            # 100 cm = 20 pixels => 1 pixel = 5 cm.
            print("current Devian: ", currentDevian*5)

        # update graph
        flag_plot = 1
        semaphore.release()


cpos = None
next_pt = None
Done = False
GotGoalRelativeCoordinate = False

""" Main entry, this function does publish, subscribe registration. And send a trigger message 
to topic request_change_view to start the whole system. """


def main():
    global goal
    global Done
    global semaphore
    global flag_plot

    print(__file__ + " start!!")
    print("User Current Version:-", sys.version)
    rospy.init_node('robot_global_vision_update', anonymous=False)

    # publish registration
    RequestChangeViewPub = rospy.Publisher(
        'request_change_view', String, queue_size=10)
    print("Waiting for all register publishes stablization (10 seconds)")
    for kl in range(10):
        time.sleep(1)
        print(kl + 1)

    # subsribe registration
    rospy.Subscriber("update_vision", String, updateGlobalVisionCallBack)
    rospy.Subscriber("updateSlamPosCpp", String, updateSlamCposForCpp)
    print("Has subscribed to topic \"update_vision, updateSlamPosCpp\", Waiting for registering subcribes stablization")
    for kl in range(2):
        time.sleep(1)
        print(kl + 1)

    # publish absolute goal to topic request_change_view
    sentGoal = String()
    sentGoal.data = str(goal)
    RequestChangeViewPub.publish(sentGoal)

    flag_plot = 0
    semaphore = threading.Semaphore(0)
    while not rospy.is_shutdown():
        semaphore.acquire()
        if flag_plot == -1:
            # input("main hold semaphore, non-main thread is temporarily stoped before clear") # don't ever delete, for debug purpose 1 (debug command 2)
            plt.clf()
            plt.pause(0.1)
            # semaphore.release() # don't ever delete, for debug purpose 1 (debug command 3)
            # time.sleep(1) # don't ever delete, for debug purpose 1 (debug command 4)

            # NOTE: uncomment 4 debug commands to go to debug mode

        elif flag_plot == 1:
            plt.draw()
            plt.pause(0.1)
            if Done:
                break

    rospy.spin()


if __name__ == '__main__':
    main()
