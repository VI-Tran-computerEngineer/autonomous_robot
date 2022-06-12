import math 
import numpy as np

def point_dist(p, q):
    '''
    calculate distance of 2 point
    '''
    return math.hypot(q[0] - p[0], q[1] - p[1])


# Return unit vector differs from Ox axis an "angel"(in radian) degree, anti-clockwise
def vectorArcordingToAngel(angel):
    return [math.cos(angel), math.sin(angel)]

# import copy
# from collections import namedtuple
def findWallVectorAngle(_360List, RobotPosition, OdomAngleXAxis):
    # _360Temp = []
    # point = namedtuple('point', ['x', 'y','index'])
    # for idx in range(len(_360List)):
    #     virtualCoordinateX = _360List[idx].x
    #     virtualCoordinateY = _360List[idx].y
    #     RotateEdge = math.sqrt(virtualCoordinateX**2 + virtualCoordinateY**2)
    #     pointAngle = math.atan2(virtualCoordinateY, virtualCoordinateX)
    #     virtualCoordinateX = math.cos(pointAngle + OdomAngleXAxis)*RotateEdge
    #     virtualCoordinateY = math.sin(pointAngle + OdomAngleXAxis)*RotateEdge
    #     virtualPoint = point(virtualCoordinateX, virtualCoordinateY, idx)
    #     _360Temp.append(virtualPoint)
   
    # _360List  = copy.deepcopy(_360Temp)
    
    Min = 0
    MinDistance = point_dist(RobotPosition, [_360List[0].x, _360List[0].y] ) # khoang cach la trung binh cong m/2 + 1 + m/2 node, nho lam
    i = len(_360List) - 1
    while point_dist( RobotPosition, [_360List[i].x, _360List[i].y] ) < MinDistance:
        MinDistance = point_dist( RobotPosition, [_360List[i].x, _360List[i].y] )
        Min = i
        i -= 1

    if Min == 0:
        i = 1
        while point_dist( RobotPosition, [_360List[i].x, _360List[i].y] ) < MinDistance:
            MinDistance = point_dist( RobotPosition, [_360List[i].x, _360List[i].y] )
            Min = i
            i += 1

    print("Min idx: ", _360List[Min].index, "  Min Distance:", MinDistance)

    virtualCoordinateX = _360List[Min].x
    virtualCoordinateY = _360List[Min].y
    RotateEdge = math.sqrt(virtualCoordinateX**2 + virtualCoordinateY**2)
    pointAngle = math.atan2(virtualCoordinateY, virtualCoordinateX)
    virtualCoordinateX = math.cos(pointAngle + OdomAngleXAxis)*RotateEdge
    virtualCoordinateY = math.sin(pointAngle + OdomAngleXAxis)*RotateEdge

    print("Wall Point:", [virtualCoordinateX, virtualCoordinateY])

    NomalWallVector = [virtualCoordinateX - RobotPosition[0]  ,   virtualCoordinateY - RobotPosition[1]]
    print("NomalWallVector:", NomalWallVector)
    NomalWallVectorAngle = math.atan2(NomalWallVector[1], NomalWallVector[0])
    return NomalWallVectorAngle - math.pi/2

def absoluteUnitVectorsInRobotPerspective(WallVectorAngel, Beta):
    AbsoluteOxUnitVectorAngelInRobotPerspective = WallVectorAngel + Beta
    AbsoluteOyUnitVectorAngelInRobotPerspective = AbsoluteOxUnitVectorAngelInRobotPerspective + math.pi/2

    return vectorArcordingToAngel(AbsoluteOxUnitVectorAngelInRobotPerspective) ,  vectorArcordingToAngel(AbsoluteOyUnitVectorAngelInRobotPerspective)

def findGoalRelativeCoordinate(GoalAbsoluteCoordinate, PaperAbsoluteCoordinate, \
    PaperRelativeCoordinate, AbsoluteOxUnitVectorInRobotPerspective, AbsoluteOyUnitVectorInRobotPerspective):

    AbsoluteStranformVectorFromPaperToGoal = [GoalAbsoluteCoordinate[0] - PaperAbsoluteCoordinate[0], \
                                             GoalAbsoluteCoordinate[1] - PaperAbsoluteCoordinate[1]]

    RelativeStranformVectorFromPaperToGoal = [AbsoluteStranformVectorFromPaperToGoal[0]*AbsoluteOxUnitVectorInRobotPerspective[0] + \
    AbsoluteStranformVectorFromPaperToGoal[1]*AbsoluteOyUnitVectorInRobotPerspective[0] , \
    AbsoluteStranformVectorFromPaperToGoal[0]*AbsoluteOxUnitVectorInRobotPerspective[1] + \
    AbsoluteStranformVectorFromPaperToGoal[1]*AbsoluteOyUnitVectorInRobotPerspective[1] ]

    return [PaperRelativeCoordinate[0] + RelativeStranformVectorFromPaperToGoal[0] , \
            PaperRelativeCoordinate[1] + RelativeStranformVectorFromPaperToGoal[1]  ]

# # Return unit vector
# def normalVectorWithDirection(inputVector, right):
#     # The math.atan2() method returns the arc tangent of y/x, in radians. Where x and y are the coordinates of a point (x,y).
#     # The returned value is between PI and -PI. math.atan2(y, x)
#     normalRightAngel = math.atan2(inputVector[1], inputVector[0]) + (- math.pi/2 if right else math.pi/2)
#     return [math.cos(normalRightAngel), math.sin(normalRightAngel)]


# def findPaperPositionInRobotPerspective(RobotPosition, PaperDiection, PaperDistance):
#     DirectionVector = vectorArcordingToAngel(PaperDiection)
#     StranformVector = [DirectionVector[0]*PaperDistance, DirectionVector[1]*PaperDistance]

#     return [   RobotPosition[0]+StranformVector[0]  ,  RobotPosition[1]+StranformVector[1]   ]
