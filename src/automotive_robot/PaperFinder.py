#!/usr/bin/env python

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import laser_geometry.laser_geometry as lg

import sys
import rospy
import cv2
import matplotlib.pyplot as plt
import csv
import numpy as np
from keras.models import load_model
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import math
import time
from automotive_robot.Phase1ProcessingLib import *
import getpass

USER = getpass.getuser()
model = load_model(
    f'/home/{USER}/catkin_ws/src/automotive_robot/src/automotive_robot/mnist.h5')
runEdge = 0.704875  # for real run
RobotPosition = [0, 0]  # May change for future extension
alpha = (49/2) / 180 * math.pi
OdomAngleXAxis = -0.002

""" recognize a single digit from an image using Machine Learning """


def predict_digit(img):
    # height, width, _ = img.shape
    # print("height:", height, "width:", width)
    # resize image to 28x28 pixels
    img = cv2.resize(img, (28, 28))
    # convert rgb to grayscale
    img = Image.fromarray(img).convert('L')
    img = np.array(img)
    # reshaping to support our model input and normalizing
    img = img.reshape(1, 28, 28, 1)
    img = img/255.0
    # predicting the class
    res = model.predict([img])[0]
    # print("result:",res)
    return res


# define a sort key
def sort_key(company):
    return company[0]


def map_display(plt, mapname, ob, obColor):
    k = 0
    # displaying the title
    plt.title("Display map: {0}".format(mapname))
    for ob_part in ob:
        x = [point[0] for point in ob_part]
        y = [point[1] for point in ob_part]
        x.append(ob_part[0][0])
        y.append(ob_part[0][1])
        plt.fill(tuple(x), tuple(y), color=obColor, alpha=0.4, hatch='//////')

        i = 0
        k += 1
        if k > 0 and k < 0:
            for point in ob_part:
                if i % 8 < 4:
                    plt.text(point[0]+0.001*(i % 8), point[1]+0.001*(i % 8), i)
                else:
                    plt.text(point[0]-0.001*(i % 8), point[1]-0.001*(i % 8), i)
                i += 1


""" Get all central coordinates of number contours and dot contours 
as well as number contours edges and dot contours edges """


def getCentersAndEdges(obs, centers, edges):
    for ob in obs:
        xs = [point[0] for point in ob]
        ys = [point[1] for point in ob]
        xedge = abs(np.min(xs) - np.max(xs))
        yedge = abs(np.min(ys) - np.max(ys))
        edge = xedge if (xedge > yedge) else yedge

        xcenter = (np.min(xs) + np.max(xs)) / 2
        ycenter = (np.min(ys) + np.max(ys)) / 2

        centers.append([xcenter, ycenter])
        edges.append(edge)


""" Reduce number contours that are just parts of the same number """


def removeObThatAlmostInsideAnotherOb(obs, centers, edges):
    global hierarchy

    i = 0
    popI = False
    while (i < len(obs)):
        j = i + 1
        while (j < len(obs)):
            if point_dist(centers[i], centers[j]) < 0.5*max(edges[i], edges[j]):
                print("Has popped!")
                popIdx = j if edges[i] > edges[j] else i
                obs.pop(popIdx)
                centers.pop(popIdx)
                edges.pop(popIdx)
                hierarchy = np.delete(hierarchy, popIdx, axis=1)
                if popIdx == i:
                    popI = True
                    break
                continue
            j += 1

        if popI:
            popI = False
            continue
        i += 1

    return hierarchy


""" Find all single numbers in an image  """


def findNumbers(rows, edges, cv_image, height, width):
    global runEdge

    numbers = []
    for row in rows:
        number = 0
        # sort the companies by revenue
        row.sort(key=sort_key, reverse=False)
        # print(edges)
        # print(row)
        firstNumberEdge = edges[row[0][2]]
        dotIdx = 1
        while dotIdx < len(row) and firstNumberEdge / edges[row[dotIdx][2]] < 1.25:
            dotIdx += 1

        for i in range(dotIdx):
            edge = edges[row[i][2]] * runEdge
            xcenter, ycenter = row[i][:2]

            top = int(-ycenter - edge) if int(-ycenter - edge) > 0 else 0
            bottom = int(-ycenter + edge) if int(-ycenter +
                                                 edge) < height else height - 1
            left = int(xcenter - edge) if int(xcenter - edge) > 0 else 0
            right = int(xcenter + edge) if int(xcenter +
                                               edge) < width else width - 1

            temp_result = predict_digit(cv_image[top: bottom, left: right])
            number = number * 10 + int(np.argmax(temp_result))

        power = -1
        for i in range(dotIdx+1, len(row)):
            edge = edges[row[i][2]] * runEdge
            xcenter, ycenter = row[i][:2]

            top = int(-ycenter - edge) if int(-ycenter - edge) > 0 else 0
            bottom = int(-ycenter + edge) if int(-ycenter +
                                                 edge) < height else height - 1
            left = int(xcenter - edge) if int(xcenter - edge) > 0 else 0
            right = int(xcenter + edge) if int(xcenter +
                                               edge) < width else width - 1

            temp_result = predict_digit(cv_image[top: bottom, left: right])
            number += int(np.argmax(temp_result)) * (10 ** power)
            power -= 1

        numbers.append(number)

    return numbers


""" Convert all number contours and dot contours belonging to the same line
in the image to a real number """


def findRows(centers, edges):
    global hierarchy

    rows = []
    idx = 0
    foundRow = False
    for center in centers:
        for row in rows:
            if edges[row[0][2]] / edges[idx] > 0.8:
                if (row[0][1] - 0.75*edges[row[0][2]] < center[1]) and (center[1] < row[0][1] + 0.75*edges[row[0][2]]) and (hierarchy[0][idx][3] == hierarchy[0][row[0][2]][3]):
                    row.append([center[0], center[1], idx])
                    foundRow = True
                    idx += 1
                    break
            else:
                if (center[1] - 0.75*edges[idx] < row[0][1]) and (center[1] + 0.75*edges[idx] > row[0][1]) and (hierarchy[0][idx][3] == hierarchy[0][row[0][2]][3]):
                    row.append([center[0], center[1], idx])
                    foundRow = True
                    idx += 1
                    break

        if foundRow:
            foundRow = False
            continue

        temp_row = [[center[0], center[1], idx]]
        rows.append(temp_row)
        idx += 1
    return rows


""" Get all number contours and dot contours detected from an image to a csv file """


def getNumbersInBoxesToCsvFile(file_name, contours):
    global hierarchy
    global Gx
    foundGx = False

    max_sibling_count = 0
    sibling_counts = []

    for node in hierarchy[0]:
        if node[3] == -1:
            sibling_counts.append(0)
            continue
        sibling_count = 0
        i = node[0]
        while(i != -1):
            sibling_count += 1
            i = hierarchy[0][i][0]

        i = node[1]
        while(i != -1):
            sibling_count += 1
            i = hierarchy[0][i][1]

        if sibling_count > max_sibling_count:
            max_sibling_count = sibling_count
        sibling_counts.append(sibling_count)

    i = 0
    file_name += ".csv"
    f = open(file_name, 'w', newline='')
    writer = csv.writer(f, delimiter=",")
    data_header = ["x", "y"]
    sibling_thresh = 10

    lengthContours = len(contours)
    while not foundGx and i < lengthContours:
        if sibling_counts[i] >= sibling_thresh:
            foundGx = True
            father = contours[hierarchy[0][i][3]]
            x_list = [pt[0][0] for pt in father]
            Gx = (max(x_list) + min(x_list))/2
            print("Gx:", Gx)
        else:
            i += 1

    i = 0
    while i < len(contours):
        if sibling_counts[i] >= sibling_thresh:
            print("sibling_counts:", sibling_counts[i])
            part = contours[i]
            writer.writerow(data_header)
            for pt in part:
                writer.writerow([pt[0][0], -pt[0][1]])
            i += 1
        else:
            contours.pop(i)
            sibling_counts.pop(i)
            hierarchy = np.delete(hierarchy, i, axis=1)
    f.close()
    del contours
    return hierarchy


def read_map_csv(mapname):
    first_line = True
    obstacles = []
    ob_part = []
    with open(mapname, newline='') as f:
        reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONE)
        for row in reader:
            # print (row[0])
            if row[0].find('x') != -1:
                # print ("find x")
                if len(ob_part) > 1:
                    ob_part.append(ob_part[0])
                    obstacles.append(ob_part)
                ob_part = []
                continue
            ob_part.append([float(row[0]), float(row[1])])
        if len(ob_part) > 1:
            ob_part.append(ob_part[0])
            obstacles.append(ob_part)
    return obstacles


def point_dist(p, q):
    '''
    calculate distance of 2 point
    '''
    return math.hypot(q[0] - p[0], q[1] - p[1])


getImage = False
get360List = False


class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.lp = lg.LaserProjection()
        self.change_view_pub = rospy.Publisher(
            'request_change_view', String, queue_size=10)
        print("\nWaiting for all register publishes stablization (5 seconds)")
        for kl in range(5):
            time.sleep(1)
            print(kl + 1)
        print("\nNow all nodes in the system can starting registering subsribe!!!")
        self.image_sub = rospy.Subscriber(
            "/raspicam_node/image/compressed", CompressedImage, self.getPapercallback, queue_size=10)
        self.done_change_view_sub = rospy.Subscriber(
            "done_change_view", String, self.doneChangeViewCallBack)
        self.laser_scan_sub = rospy.Subscriber(
            "/scan", LaserScan, self.scan_cb, queue_size=10)

    """ Main call back in Paper Finder, triggered whenever a message is received from topic done_change_view """

    def getPapercallback(self, data):
        # return
        global getImage
        global GoalAbsoluteCoordinate
        if not getImage:
            return
        getImage = False
        print("getPapercallback executed")

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        global hierarchy
        height, width, _ = cv_image.shape
        print("height:", height, "width:", width)
        cv2.imshow('Raw image', cv_image)
        cv2.waitKey(0)
        src_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        src_gray = cv2.blur(src_gray, (3, 3))

        canny_output = cv2.Canny(src_gray, 65, 180)
        # cv2.imshow('Canny', canny_output)
        # cv2.waitKey(0)
        contours, hierarchy = cv2.findContours(
            canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # plt.subplot(1, 2 ,1 )
        # for ob_part in contours:
        #     x = [point[0][0] for point in ob_part]
        #     y = [point[0][1] for point in ob_part]
        #     x.append(ob_part[0][0][0])
        #     y.append(ob_part[0][0][1])
        #     plt.fill(tuple(x), tuple(y), alpha=0.4, hatch='//////')

        world_name = "contours_results"
        getNumbersInBoxesToCsvFile(world_name, contours)
        obs = read_map_csv(world_name + ".csv")

        if not obs:
            print("no paper in current frame!!!!")
            self.change_view_pub.publish("Not found Paper!")
            return

        global get360List
        global _360List
        get360List = True
        centers = []
        edges = []

        getCentersAndEdges(obs, centers, edges)

        removeObThatAlmostInsideAnotherOb(obs, centers, edges)

        # plt.subplot(1, 2 ,2)
        map_display(plt, ".csv", obs, 'c')
        print("number of obstacles detected: ", len(obs))
        plt.show()

        rows = findRows(centers, edges)
        numbers = findNumbers(rows, edges, cv_image, height, width)
        print("numbers:", numbers)

        # for i in range(len(numbers)):
        #     print("number ", i, ": ", numbers[i])

        global Gx
        global alpha
        global OdomAngleXAxis
        Beta = math.atan((width - 2*Gx) / width * math.tan(alpha))*180/math.pi
        Beta = 360 + Beta if Beta < 0 else Beta
        print("Beta:", Beta)

        chosenIdx = None  # floor index
        i = 0
        while(chosenIdx == None):
            # print(_360List[i], int(Beta), chosenIdx)
            if abs(int(Beta) - _360List[i].index) < 0.001:
                chosenIdx = i
            elif abs(int(Beta) - _360List[-i].index) < 0.001:
                chosenIdx = -i

            i += 1

        for i in range(20):
            print("+", i, "=", _360List[i].x, _360List[i].y,
                  "    -", i, "=", _360List[-i].x, _360List[-i].y)

        # chosenIdx = tempList.index(min(tempList))
        print("360 index:", _360List[chosenIdx].index)
        PaperX = (1 - (Beta - int(Beta))) * _360List[chosenIdx].x + (
            1 - (int(Beta+1) - Beta)) * _360List[chosenIdx+1].x
        PaperY = (1 - (Beta - int(Beta))) * _360List[chosenIdx].y + (
            1 - (int(Beta+1) - Beta)) * _360List[chosenIdx+1].y
        print("PaperPositionVirtual", [PaperX, PaperY])
        RotateEdge = math.sqrt(PaperX**2 + PaperY**2)
        pointAngle = math.atan2(PaperY, PaperX)
        PaperX = math.cos(pointAngle + OdomAngleXAxis)*RotateEdge
        PaperY = math.sin(pointAngle + OdomAngleXAxis)*RotateEdge
        PaperPositionInRobotPerspective = [PaperX, PaperY]
        print("PaperPositionInRobotPerspective:",
              PaperPositionInRobotPerspective)
        WallVectorAngle = findWallVectorAngle(
            _360List, RobotPosition, OdomAngleXAxis)
        print("WallVectorAngle:", WallVectorAngle)
        AbsoluteOxUnitVectorInRobotPerspective, AbsoluteOyUnitVectorInRobotPerspective = \
            absoluteUnitVectorsInRobotPerspective(
                WallVectorAngle, Beta=numbers[0]/180*math.pi)
        print("AbsoluteOxUnitVectorInRobotPerspective: ",
              AbsoluteOxUnitVectorInRobotPerspective)
        print("AbsoluteOyUnitVectorInRobotPerspective: ",
              AbsoluteOyUnitVectorInRobotPerspective)
        GoalRelativeCoordinate = findGoalRelativeCoordinate(GoalAbsoluteCoordinate, [numbers[2], numbers[1]],
                                                            PaperPositionInRobotPerspective, AbsoluteOxUnitVectorInRobotPerspective, AbsoluteOyUnitVectorInRobotPerspective)

        self.change_view_pub.publish(str(GoalRelativeCoordinate))
        exit(0)
        # cv2.imshow("Image window", cv_image)
        # time.sleep(10)

    def doneChangeViewCallBack(self, msg):
        print("message received in doneChangeViewCallBack: ", msg.data)
        global getImage
        global OdomAngleXAxis
        if "goal" in msg.data:
            global GoalAbsoluteCoordinate
            temp = msg.data.split(";")
            temp1 = temp[0][6:-1].split(', ')
            GoalAbsoluteCoordinate = [float(temp1[0]), float(temp1[1])]
            print("doneChangeViewCallBack: got absolute goal = ",
                  GoalAbsoluteCoordinate)
            OdomAngleXAxis = float(temp[1][6:])
        else:
            OdomAngleXAxis = float(msg.data[5:])
        print("doneChangeViewCallBack: got odomAngle = ", OdomAngleXAxis)
        getImage = True

    def scan_cb(self, msg):
        global get360List
        if not get360List:
            return

        get360List = False

        global _360List

        # convert the message of type LaserScan to a PointCloud2
        pc2_msg = self.lp.projectLaser(msg)

        # convert it to a generator of the individual points
        point_generator = pc2.read_points(pc2_msg)

        # get list of the individual points which is less efficient
        _360List = pc2.read_points_list(pc2_msg)


def main(args):
    rospy.init_node('paper_finder', anonymous=False)
    print("Done init")
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
