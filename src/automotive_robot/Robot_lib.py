from cmath import sqrt
from os import system
import numpy as np
import math
import copy


def intersection(x, y, radius, ls_points):  # line segment points
    ''' 
    find the two different points where a line intersections a circle 
    '''
    is_pt = []
    p1x, p1y = ls_points[0]
    p2x, p2y = ls_points[1]
    dx, dy = np.subtract(p2x, p1x), np.subtract(p2y, p1y)
    a = dx ** 2 + dy ** 2
    b = 2 * (dx * (p1x - x) + dy * (p1y - y))
    c = (p1x - x) ** 2 + (p1y - y) ** 2 - radius ** 2
    discriminant = b ** 2 - 4 * a * c
    if discriminant > 0:
        t1 = (-b + discriminant ** 0.5) / (2 * a)
        t2 = (-b - discriminant ** 0.5) / (2 * a)
        pt1 = [dx * t1 + p1x, dy * t1 + p1y]
        pt2 = [dx * t2 + p1x, dy * t2 + p1y]
        is_pt = (pt1, pt2)

    return is_pt


def inside_ls(point, ls_points):  # line segment points
    '''
    check if a point is whether inside given line segment 
    return True if inside, otherwise return False
    using total distance to check, 
    '''
    dp1 = point_dist(point, ls_points[0])
    dp2 = point_dist(point, ls_points[1])
    dls = point_dist(ls_points[1], ls_points[0])
    dp = dp1 + dp2
    return math.isclose(dp, dls)


def rotate_vector_center(center, v, radians):
    '''
    rotate a vector with a angle of radians around center point
    '''
    vector_vc = np.subtract(v, center)
    r_vector_vc = rotate_vector(vector_vc, radians)
    result = np.add(center, r_vector_vc)
    return np.add(center, r_vector_vc)


def rotate_vector(v, radians):
    '''
    rotate vector with a angle of radians around (0,0)
    '''
    x, y = v
    rx = x * math.cos(radians) + y * math.sin(radians)
    ry = -x * math.sin(radians) + y * math.cos(radians)
    return rx, ry


def unit_vector(vector):
    '''
    Return the unit vector of the vector
    '''
    return vector / np.linalg.norm(vector)


def unsigned_angle(v1, v2):
    '''
    Find angle between two vectors
    '''
    usa = signed_angle(v1, v2)
    if usa < 0:
        usa = 2 * math.pi + usa
    return usa

def unsigned_angle(center, ptA, ptB):
    '''
    Find angle among 3 points (ptA- center- ptB)
    '''
    vector_a = np.subtract(ptA, center)
    vector_b = np.subtract(ptB, center)
    usa = signed_angle(vector_a, vector_b)
    if usa < 0:
        usa = 2 * math.pi + usa
    return usa

def unsigned_angle_xAxis(point):
    '''
    Find unsigned angle between point and x axis
    '''

    angle = math.atan2(point[1], point[0])
    if angle < 0:
        angle = 2 * math.pi + angle
    return angle


def signed_angle_xAxis(point):
    '''
    Finds signed angle between point and x axis
    return angle (+) for anti clockwise, and (-) for clockwise
    '''
    angle = math.atan2(point[1], point[0])
    # print (math.degrees(angle))
    return angle


def signed_angle(v1, v2):
    '''
    Finds angle between two vectors
    return angle (+) for anti clockwise, and (-) for clockwise
    '''
    angle = signed_angle_xAxis(v1)  # cal angle between vector (A) and ox axis
    v_b = rotate_vector(v2, angle)  # rotate vector B according to rotation_radians 
    return signed_angle_xAxis(v_b)


def get_angle_info(center, ptA, ptB):
    '''
    return angle, start edge , end edge of given angle in anti-clockwise
    '''

    vector_a = np.subtract(ptA, center)
    vector_b = np.subtract(ptB, center)

    angle = signed_angle(vector_a, vector_b)

    if angle < 0:
        vs = ptB  # start 
        ve = ptA  # end
    else:
        vs = ptA  # start
        ve = ptB  # end
    return angle, vs, ve


def inside_angle_area(check_pt, center, ref_boundaries):
    ''' 
    check if a check_pt is where inside (ref[start]- center - ref[end]) area
    return True if inside
        additional code = 0, on the first of edge ref_boundaries
        additional code = 1, on the second of edge ref_boundaries
        additional code = 2, in closed area of ref_boundaries
    return Flase if outside
    using math.isclose to avoid the error of floating check_pt computation
    '''
    vector_a = np.subtract(ref_boundaries[0], center)
    vector_b = np.subtract(ref_boundaries[1], center)
    vector_p = np.subtract(check_pt, center)

    ref_angle = signed_angle(vector_a, vector_b)
    cpt_angle = signed_angle(vector_a, vector_p)
    diff_angle = abs(ref_angle - cpt_angle)
    rel_tol = 0.0000001
    ret_result = False  # return result
    ret_code = 0  # return cod3
    if abs(cpt_angle) < rel_tol:
        # print ("the point is on edge 0")
        ret_result = True
        ret_code = 0
    elif diff_angle < rel_tol:
        # print ("the point is on edge 1")
        ret_result = True
        ret_code = 1

    elif ref_angle * cpt_angle < 0:  # diff side
        # print ("the point is outside ref area")
        ret_result = False
    else:
        # compare angles in unsigned
        abs_angle_sight = abs(ref_angle)
        abs_angle_check_pt = abs(cpt_angle)
        if abs_angle_sight > abs_angle_check_pt:
            # print ("The point is in closed ref angle")
            ret_result = True
            ret_code = 2
        else:
            # print ("the point is outside ref area_1")
            ret_result = False
    return ret_result, ret_code


def inside_closed_angle_area(check_pt, center, ref_boundaries):
    ''' 
    check if a check_pt is where inside closed angle of (ref[start]- center - ref[end]) area
    return True if inside (not boundary)
    return False if outside
    '''
    in_status, in_code = inside_angle_area(check_pt, center, ref_boundaries)
    return in_status and in_code == 2


def center_triangle(triangle):
    '''
    return center of triangle
    '''
    return np.mean(triangle, axis=0)


def center_triangles(triangles):
    ''' 
    return a list of center of triangles
    '''
    return [center_triangle(triangle) for triangle in triangles]


def inside_triangle(point, triangle):
    '''
    if point is at the edge of triangle: return true and code = 0,1,2
    if point is inside closed triangle: return true and code = 4
    if point is outside triangle: return false
    '''
    ptin_0, code0 = inside_angle_area(point, triangle[0], [triangle[1], triangle[2]])
    ptin_1, code1 = inside_angle_area(point, triangle[1], [triangle[0], triangle[2]])
    ptin = np.logical_and(ptin_0, ptin_1)
    if ptin:
        code = code0 + code1
    else:
        code = 0
    return ptin, code


def point_belong_triangle(point, triangle):
    '''
    return true if point is vertex of triangle
    otherwise return false
    '''
    result = belong_triangle(point, triangle)
    return np.sum(result) > 0


def belong_triangle(point, triangle):
    '''
    return true if the given points is one of triangle's vertices
    '''
    pdist_a = point_dist(triangle[0], point)
    pdist_b = point_dist(triangle[1], point)
    pdist_c = point_dist(triangle[2], point)
    at_a = math.isclose(pdist_a, 0)
    at_b = math.isclose(pdist_b, 0)
    at_c = math.isclose(pdist_c, 0)
    return at_a, at_b, at_c


def mutual_edge(triA, triB):
    '''
    return true if 2 triangle have mutual edge
    '''
    ret_result = False
    check_a = belong_triangle(triA[0], triB)
    check_b = belong_triangle(triA[1], triB)
    check_c = belong_triangle(triA[2], triB)
    result = np.logical_or(check_a, check_b)
    result = np.logical_or(result, check_c)
    return np.sum(result) == 2


def get_pairs_triangles(triangles):
    '''
    return list of the edges of triangles
    '''
    edges = []
    for i in range(len(triangles) - 1):
        for j in range(i + 1, len(triangles)):
            if mutual_edge(triangles[i], triangles[j]):
                edges.append([i, j])
    return edges


def midpoint(P, Q):
    '''
    return mid point of 2 point Q, P
    '''
    x = (P[0] + Q[0]) / 2
    y = (P[1] + Q[1]) / 2
    return x, y


def line_from_points(P, Q):
    '''
    return a, b, c of line from point P and Q
    where ax + by = c
    '''
    a = Q[1] - P[1] 
    b = P[0] - Q[0] 
    c = a * (P[0]) + b * (P[1]) 
    return a, b, c


def belong_line(point, line):
    '''
    check if the point is whether belong line or not
    '''
    a, b, c = line_from_points(line[0], line[1])
    return math.isclose(a * point[0] + b * point[1], c)


def point_dist(p, q):
    '''
    calculate distance of 2 point
    '''
    return math.hypot(q[0] - p[0], q[1] - p[1])


def line_across(line1, line2):
    '''
    check whether 2 lines are cross each others or not
    return cross point if they are
    return None if not
    '''
    is_pt = line_intersection(line1, line2)
    ret_result = None
    if is_pt is None:
        if point_dist(line1[0],line2[0]) < 0.001 or point_dist(line1[0],line2[1]) < 0.001:
            return line1[0]
        elif point_dist(line1[1],line2[0]) < 0.001 or point_dist(line1[1],line2[1]) < 0.001:
            return line1[1]
            
    elif inside_ls(is_pt, line1) and inside_ls(is_pt, line2):
        ret_result = is_pt
    return ret_result


def line_intersection(line1, line2):
    '''
    return intersection point of 2 lines
    if that point does not exist, return none
    '''
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return None
    # raise Exception('lines do not intersect')
    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y


def get_middle_direction(center, radius, pair):
    '''
    given 2 points and a center, this function finds a direction where is from center to middle 
    of 2 points with length of radius
    '''
    midpt = midpoint(pair[0], pair[1])
    pt_is = intersection(center[0], center[1], radius, [center, midpt])

    if inside_ls(midpt, [pt_is[0], center]):
        return pt_is[0]
    else:
        return pt_is[1]


def get_index_true(status):
    ''' 
    return index of true elements
    '''
    return np.where(status)[0]


def get_index_false(status):
    ''' 
    return index of False elements
    '''
    not_status = np.logical_not(status)
    return np.where(not_status)[0]


def print_pairs(message_ID, pairs):
    print("{0}, len: {1}".format(message_ID, len(pairs)))
    for pair in pairs:
        print(pair[0], pair[1])


def print_point(message_ID, point_x, point_y):
    print(message_ID)
    for i in range(len(point_x)):
        print("{0} {1}".format(point_x[i], point_y[i]))


def print_cpairs(message_ID, cpairs):  # print circle pairs
    print("{0}, len: {1}".format(message_ID, len(cpairs)))
    for pairs in cpairs:
        print(pairs)


def approximately_num(num):
    return format(float(num), '.10f')


import matplotlib.pyplot as plt


def find_configure_space(obstacles, robot_radius):
    ''' this function is to find a configure space which is robot free-collision ares '''
    nvs = []
    cspaces = []

    # get normal vectors of all obstacles
    for obstacle in obstacles:
        nv = []
        for i in range(len(obstacle)):
            if i == len(obstacle) - 1:  # last one
                nvector = normal_vector((obstacle[i], obstacle[0]), robot_radius)
            else:
                nvector = normal_vector((obstacle[i], obstacle[i + 1]), robot_radius)
            nv.append(nvector)
        nvs.append(nv)

    # get bisectors and limited line segment at joint points
    lim_lss = []
    for obstacle in obstacles:
        lim_ls = []
        for i in range(len(obstacle)):
            if i == len(obstacle) - 1:  # last item
                ls = cal_bisector(obstacle[i - 1], obstacle[i], obstacle[0], robot_radius)
            else:
                ls = cal_bisector(obstacle[i - 1], obstacle[i], obstacle[i + 1], robot_radius)
            lim_ls.extend(ls)
        lim_lss.append(lim_ls)

    for lim_ls in lim_lss:
        print("__________________", lim_ls)
        i = 0
        for ls in lim_ls:
            plt.plot(ls[0], ls[1], ".r")
            plt.text(ls[0], ls[1], ".{0}".format(i))
            i = i + 1

    # find extend of obstacles
    j = 0
    extend_cspaces = []
    for obstacle in obstacles:
        extend_cspace = []
        for i in range(len(obstacle)):
            if i == len(obstacle) - 1:  # last one
                extend_cspace.append(np.add(obstacle[i], nvs[j][i]))
                extend_cspace.append(np.add(obstacle[0], nvs[j][i]))
            else:
                extend_cspace.append(np.add(obstacle[i], nvs[j][i]))
                extend_cspace.append(np.add(obstacle[i + 1], nvs[j][i]))
        extend_cspaces.append(extend_cspace)
        j = j + 1

    for extend_cspace in extend_cspaces:
        i = 0
        for pt in extend_cspace:
            plt.plot(pt[0], pt[1], ".b")
            plt.text(pt[0], pt[1], ",{0}".format(i))
            i = i + 1

    # find boundary of configuration spaces
    j = 0
    for nv in nvs:
        cspace = []
        for i in range(len(nv)):
            ls_Eob = (extend_cspaces[j][2 * i], extend_cspaces[j][2 * i + 1])
            if i == len(nv) - 1:
                lsA = (lim_lss[j][2 * i], lim_lss[j][2 * i + 1])
                lsB = (lim_lss[j][0], lim_lss[j][1])
            else:
                lsA = (lim_lss[j][2 * i], lim_lss[j][2 * i + 1])
                lsB = (lim_lss[j][2 * i + 2], lim_lss[j][2 * i + 3])
            jointPtA = line_intersection(ls_Eob, lsA)
            jointPtB = line_intersection(ls_Eob, lsB)
            cspace.append(jointPtA)
            cspace.append(jointPtB)
        cspaces.append(cspace)
        j = j + 1
    return cspaces

def find_configure_space_update_1(obstacles, robot_radius):
    cspaces = []
    for obstacle in obstacles:
        obstacle.pop()
        len_ob_temp = len(obstacle)
        cspace = []
        extend_cspace = []
        if (left_hand_direction(obstacle)):
            for i in range(len_ob_temp):
                nvector = normal_vector((obstacle[i], obstacle[(i + 1)%len_ob_temp]), robot_radius)
                extend_cspace.append(np.add(obstacle[i], nvector))
                extend_cspace.append(np.add(obstacle[(i + 1)%len_ob_temp], nvector))
        else:
            for i in range(len_ob_temp):
                nvector = normal_vector((obstacle[(i + 1)%len_ob_temp], obstacle[i]), robot_radius)
                extend_cspace.append(np.add(obstacle[i], nvector))
                extend_cspace.append(np.add(obstacle[(i + 1)%len_ob_temp], nvector))

        for i in range(len_ob_temp):
            a,b,c = line_from_points(obstacle[i],obstacle[i - 1])
            flag_check = a*obstacle[(i + 1)%len_ob_temp][0] + b*obstacle[(i + 1)%len_ob_temp][1] - c
            concave_edge = 0
            for j in range(len_ob_temp):
                if ((a*obstacle[j][0] + b*obstacle[j][1] - c)*flag_check >= 0):            
                    continue
                else:
                    concave_edge += 1
                    break
            
            a,b,c = line_from_points(obstacle[i],obstacle[(i + 1)%len_ob_temp])
            flag_check = a*obstacle[i - 1][0] + b*obstacle[i - 1][1] - c
            for j in range(len_ob_temp):
                if ((a*obstacle[j][0] + b*obstacle[j][1] - c)*flag_check >= 0):
                    continue
                else:
                    concave_edge += 1
                    break
            
            if (concave_edge == 2):
                cspace.append(line_intersection([extend_cspace[2*i],extend_cspace[2*i + 1]], [extend_cspace[2*i - 1],extend_cspace[2*i - 2]]))
            else:
                lim_ls = cal_bisector(obstacle[i - 1], obstacle[i],obstacle[(i + 1)%len_ob_temp], robot_radius)
                
                cspace.append(line_intersection(lim_ls, [extend_cspace[2*i - 1],extend_cspace[2*i - 2]]))
                cspace.append(line_intersection(lim_ls, [extend_cspace[2*i],extend_cspace[2*i + 1]]))
        cspace.append(cspace[0])
        cspaces.append(cspace)

    for cspace in cspaces:
        i = 1
        for pt in cspace:
            plt.plot(pt[0], pt[1], ".b")
            plt.text(pt[0], pt[1], ",{0}".format(i))
            i = i + 1
    return cspaces

def find_configure_space_update_2(obstacles, robot_radius):
    cspaces = []
    for obstacle in obstacles:
        obstacle.pop()
        len_ob_temp = len(obstacle)
        cspace = []
        extend_cspace = []
        if (left_hand_direction(obstacle)):
            for i in range(len_ob_temp):
                nvector = normal_vector((obstacle[i], obstacle[(i + 1)%len_ob_temp]), robot_radius)

                # pt = ((obstacle[i][0] + obstacle[(i+1)%len_ob_temp][0])/2,  (obstacle[i][1] + obstacle[(i+1)%len_ob_temp][1])/2 )
                # pt_tranformed = (pt[0] + nvector[0]/20, pt[1] + nvector[1]/20)
                # plt.plot((pt[0], pt_tranformed[0]), (pt[1], pt_tranformed[1]), ".b")


                extend_cspace.append(np.add(obstacle[i], nvector))
                extend_cspace.append(np.add(obstacle[(i + 1)%len_ob_temp], nvector))
        else:
            for i in range(len_ob_temp):
                nvector = normal_vector((obstacle[(i + 1)%len_ob_temp], obstacle[i]), robot_radius)

                # pt = ((obstacle[i][0] + obstacle[(i+1)%len_ob_temp][0])/2,  (obstacle[i][1] + obstacle[(i+1)%len_ob_temp][1])/2 )
                # pt_tranformed = (pt[0] + nvector[0]/20, pt[1] + nvector[1]/20)
                # plt.plot((pt[0], pt_tranformed[0]), (pt[1], pt_tranformed[1]), ".r")

                extend_cspace.append(np.add(obstacle[i], nvector))
                extend_cspace.append(np.add(obstacle[(i + 1)%len_ob_temp], nvector))

        for i in range(len_ob_temp):
            a,b,c = line_from_points(obstacle[(i + 1)%len_ob_temp],obstacle[i - 1])
            temp_intersection_point = None
            if (point_dist(obstacle[i],obstacle[i-1]) <= point_dist(obstacle[i],obstacle[(i+1)%len_ob_temp])):
                vector = unit_vector((obstacle[i][0] - obstacle[(i+1)%len_ob_temp][0],obstacle[i][1] - obstacle[(i+1)%len_ob_temp][1]))*(point_dist(obstacle[i],obstacle[(i+1)%len_ob_temp]) - point_dist(obstacle[i],obstacle[i-1]))
                temp_intersection_point = line_intersection([np.add(obstacle[(i+1)%len_ob_temp], vector),np.add(extend_cspace[2*i + 1],vector)],
                [obstacle[i-1],extend_cspace[2*i - 2]])
                sign2 = a*extend_cspace[2*i - 2][0] + b*extend_cspace[2*i - 2][1] - c
            else:
                vector = unit_vector((obstacle[i][0] - obstacle[i - 1][0],obstacle[i][1] - obstacle[i - 1][1]))*( - point_dist(obstacle[i],obstacle[(i+1)%len_ob_temp]) + point_dist(obstacle[i],obstacle[i-1]))
                temp_intersection_point = line_intersection([obstacle[(i+1)%len_ob_temp],extend_cspace[2*i + 1]],
                [np.add(obstacle[i-1],vector),np.add(extend_cspace[2*i - 2],vector)])
                sign2 = a*extend_cspace[2*i + 1][0] + b*extend_cspace[2*i + 1][1] - c

            
            if (temp_intersection_point == None):
                print(obstacle)
            sign1 = a*temp_intersection_point[0] + b*temp_intersection_point[1] - c
            

            if (sign1*sign2 > 0):
                cspace.append(line_intersection([extend_cspace[2*i],extend_cspace[2*i + 1]], [extend_cspace[2*i - 1],extend_cspace[2*i - 2]]))
            else:
                lim_ls = cal_bisector(obstacle[i - 1], obstacle[i],obstacle[(i + 1)%len_ob_temp], robot_radius)
                
                cspace.append(line_intersection(lim_ls, [extend_cspace[2*i - 1],extend_cspace[2*i - 2]]))
                cspace.append(line_intersection(lim_ls, [extend_cspace[2*i],extend_cspace[2*i + 1]]))
        cspace.append(cspace[0])
        cspaces.append(cspace)
        obstacle.append(obstacle[0])
    return cspaces

def normal_vector(linesegment, robot_radius): # up xuong tu i -> i + 1
    ''' this function is to find a  normal vector of a line segment '''
    # calculate normal vector
    nv = ((linesegment[1][1] - linesegment[0][1]), -(linesegment[1][0] - linesegment[0][0]))
    # return normal vector with length of robot's radius
    rnv = unit_vector(nv) * robot_radius
    return rnv


def cal_bisector(ptA, ptMid, ptB, robot_radius):
    '''
    this function calculate a bisector of 2 vector (mid,A) and (mid,B) then return line segment with normal
    vector of bisector
    '''
    vector_a = np.subtract(ptMid, ptA)
    vector_b = np.subtract(ptMid, ptB)
    u_vec_a = unit_vector(vector_a)
    u_vec_b = unit_vector(vector_b)
    vector_bs = np.add(u_vec_a, u_vec_b)
    vector_bs = unit_vector(vector_bs) * robot_radius
    vector_bs = np.add(ptMid, vector_bs)
    vector_ab = np.subtract(u_vec_a, u_vec_b)
    limit_ls = np.add(vector_bs, vector_ab)
    line_segment_bs = (vector_bs, limit_ls)
    return line_segment_bs

def get_intersections_2circles(center_0, r0, center_1, r1):
    '''
    get intersection of 2 circles
    '''
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1
    x0, y0 = center_0
    x1, y1 = center_1
    d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    # non intersecting
    if d > r0 + r1:
        return None
    # One circle within other
    if d < abs(r0 - r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
        h = math.sqrt(r0 ** 2 - a ** 2)
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d

        return (x3, y3), (x4, y4)

''' check whether a point is inside a polygon or not using ray_tracing_method'''
def point_inside_polygons(pt, polygons):
    result = False
    for polygon in polygons:
        if ray_tracing_method(pt[0], pt[1], polygon):
            return True
    return False

# Ray tracing
def ray_tracing_method(x,y,poly):

    n = len(poly)
    inside = False

    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside

def distance_from_point_to_linesegment(point, linesegment):
    #distance from point M to linesegment AB
    MA = point_dist(point, linesegment[0])
    MB = point_dist(point, linesegment[1])
    AB = point_dist(linesegment[0], linesegment[1])
    if (max(MA, MB, AB) == AB):
        a,b,c = line_from_points(linesegment[0], linesegment[1]) 
        perpendicular_projection_linesegment =  abs(a*point[0] + b*point[1] - c)/math.sqrt(a*a + b*b)
        return perpendicular_projection_linesegment
    else:
        return min(MA, MB)


def is_safe_point(point,obstacles,robot_radius):
    for obstacle in obstacles:
        for linesegment in obstacle:
            if (distance_from_point_to_linesegment(point,linesegment) <= robot_radius):
                return False
                
    return True

# import matplotlib.pyplot as plt
def left_hand_direction(obstacle):
    i = 0
    adjust = 2
    adjustStep = 0.1
    reUpdateMidPt = False
    while True:
        if point_dist(obstacle[i], obstacle[i+1]) < 0.01:
            i += 1
            continue

        mid_pt = (  obstacle[i][0] + (obstacle[i+1][0] - obstacle[i][0])/adjust   ,  obstacle[i][1] + (obstacle[i+1][1] - obstacle[i][1])/adjust  )
        normalVector = normal_vector((obstacle[i], obstacle[i+1]), 0.0005)
        mid_pt_transformed = (mid_pt[0] + normalVector[0], mid_pt[1] + normalVector[1])
        lineSegmentBoundaryPoint = (mid_pt_transformed[0] + normalVector[0]*2000*384*1.414, mid_pt_transformed[1] + normalVector[1]*2000*384*1.414)
        
        #plt.plot(  (mid_pt_transformed[0], lineSegmentBoundaryPoint[0]), (mid_pt_transformed[1], lineSegmentBoundaryPoint[1]), '-m')

        j = 0
        countFirst = 0
        lengthFirst = len(obstacle)
        while j < lengthFirst:
            if inside_ls(obstacle[j], [mid_pt_transformed, lineSegmentBoundaryPoint] ) or inside_ls(obstacle[(j+1)%lengthFirst], [mid_pt_transformed, lineSegmentBoundaryPoint] ):
                # print("readjust first time normal vector for left hand with:")
                # print("mid_pt: ", mid_pt)
                # print("mid_pt transformed: ", mid_pt_transformed)
                # print("obstacle[i]: ", obstacle[i])
                # print("obstacle[i+1]: ", obstacle[i+1])

                adjust += adjustStep
                reUpdateMidPt = True
                break
            if line_across(    [mid_pt_transformed, lineSegmentBoundaryPoint]  ,    [obstacle[j], obstacle[(j+1)%lengthFirst]]   ):   
                countFirst += 1
            j += 1

        if reUpdateMidPt:
            reUpdateMidPt = False
            continue

        if countFirst % 2 == 0:
            normalVector = (-normalVector[0], -normalVector[1])
            mid_pt_transformed = (mid_pt[0] + normalVector[0], mid_pt[1] + normalVector[1])
            lineSegmentBoundaryPoint = (mid_pt_transformed[0] + normalVector[0]*2000*384*1.414, mid_pt_transformed[1] + normalVector[1]*2000*384*1.414)
            #plt.plot(  (mid_pt_transformed[0], lineSegmentBoundaryPoint[0]), (mid_pt_transformed[1], lineSegmentBoundaryPoint[1]), '-k')
            k = 0
            countSecond = 0
            lengthSecond = len(obstacle)
            while k < lengthSecond:
                if inside_ls(obstacle[k], [mid_pt_transformed, lineSegmentBoundaryPoint] ) or inside_ls(obstacle[(k+1)%lengthSecond], [mid_pt_transformed, lineSegmentBoundaryPoint] ):
                    print("readjust second time normal vector for left hand with adjust: ", adjust)
                    adjust += adjustStep
                    reUpdateMidPt = True
                    break
                if line_across(    [mid_pt_transformed, lineSegmentBoundaryPoint]    ,    [obstacle[k], obstacle[(k+1)%lengthSecond]]     ):
                    countSecond += 1
                k += 1

            if reUpdateMidPt:
                reUpdateMidPt = False
                continue

            if countSecond % 2 == 0:
                i += 1
            else:
                return True
        else:
            return False

def left_hand_direction_update(obstacle):
    i = 0
    adjust = 2
    adjustStep = 0.1
    reUpdateMidPt = False
    obstacle.append(obstacle[0])
    while True:
        mid_pt = (  obstacle[i][0] + (obstacle[i+1][0] - obstacle[i][0])/adjust   ,  obstacle[i][1] + (obstacle[i+1][1] - obstacle[i][1])/adjust  )
        normalVector = normal_vector((obstacle[i], obstacle[i+1]), 0.0001)
        mid_pt_transformed = (mid_pt[0] + normalVector[0], mid_pt[1] + normalVector[1])

        if point_inside_polygons(mid_pt_transformed, obstacle) == False: #countFirst % 2 == 0:
            normalVector = (-normalVector[0], -normalVector[1])
            mid_pt_transformed = (mid_pt[0] + normalVector[0], mid_pt[1] + normalVector[1])

            if point_inside_polygons(mid_pt_transformed, obstacle) == False: #countSecond % 2 == 0:
                i += 1
            else:
                obstacle.pop()
                return True
        else:
            obstacle.pop()
            return False
