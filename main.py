# Initial python script for optimization final project

from math import sqrt, pi, floor, ceil, acos
import matplotlib.pyplot as plt
from scipy.optimize import minimize, brute
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import numpy as np
from numpy import linalg as LA

materials = ["Aluminum", "Steel", "ABS"]
crossSectionShapes = ["Square", "Round", "Rectangle"]

shapeBaseLength = 0.05  # meters, if square or rectangle
shapeBaseHeight = 0.05  # meters, if rectangle
shapeBaseDiameter = 0.05  # meters, if circle

def GetMaterialProperties(materialType):
    if materialType == "Aluminum":
        rho = 2800  # kg/m3
        E = 71E9  # Pascals
        Sy = 95E7  # Pascals
        Su = 110E7  # Pascals
    elif materialType == "Steel":
        rho = 7750  # kg/m3
        E = 207E9  # Pascals
        Sy = 250E7  # Pascals
        Su = 400E7  # Pascals
    elif materialType == "ABS":
        rho = 1115 # kg/m3
        E = 2E9  # Pascals
        Sy = 4E7  # Pascals, no yield strength listed
        Su = 4E7  # Pascals
    else:
        rho = -9999
        E = -9999
        Sy = -9999
        Su = -9999
        print("Invalid material selected!")

    # print('\u03C1 = ', rho)
    # print("E =", E)
    # print("Sy =", Sy)
    # print("Su =", Su)

    return rho, E, Sy, Su


def GetPropertiesOfSections(crossSectionShape):
    if crossSectionShape == "Round":
        A = (pi*(shapeBaseDiameter**2))/4
        Ix = (pi*(shapeBaseDiameter**4))/64
        Iy = Ix
    elif crossSectionShape == "Rectangle":
        A = shapeBaseLength*shapeBaseHeight
        Ix = (shapeBaseLength * (shapeBaseHeight ** 3)) / 12
        Iy = (shapeBaseHeight * (shapeBaseLength ** 3)) / 12
    elif crossSectionShape == "Square":
        A = shapeBaseLength**2
        Ix = (shapeBaseLength * (shapeBaseLength ** 3)) / 12
        Iy = (shapeBaseLength * (shapeBaseLength ** 3)) / 12

    # print("A =", A)
    # print("Ix =", Ix)
    # print("Iy =", Iy)

    return A, Ix, Iy


def GetLengthOfLine(lineToGetLength):
    return sqrt(distance(lineToGetLength.points[0], lineToGetLength.points[1]))


def GetMassOfLine(lineToCalcMass):
    A, Ix, Iy = GetPropertiesOfSections(crossSectionShapes[0])
    rho, E, Sy, Su = GetMaterialProperties(materials[0])
    massOfLine = GetLengthOfLine(lineToCalcMass) * A * rho  # mass in kilograms

    return massOfLine


def GetMassOfAllLines(linesToGetMass):
    totalMass = 0
    for m in range(len(linesToGetMass)):
        totalMass += GetMassOfLine(linesToGetMass[m])
        # print("Mass of line ", m, ":", GetMassOfLine(linesToGetMass[m]))

    return totalMass


def PrintMassOfAllLines(linesToGetMass):
    totalMass = 0
    for m in range(len(linesToGetMass)):
        totalMass += GetMassOfLine(linesToGetMass[m])
        print("Mass of line ", m, ":", GetMassOfLine(linesToGetMass[m]))
    print("Total mass:", totalMass)

class point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __str__(self):
        return f"x:{self.x} y:{self.y}"


class line:
    def __init__(self, firstInitialPointForLine, secondInitialPointForLine):
        self.points = [firstInitialPointForLine, secondInitialPointForLine]

    def __str__(self):
        return f"({self.points[0].x},{self.points[0].y}), ({self.points[1].x},{self.points[1].y})"


def distance(a,b):
    # print(a)
    # print(b)
    return sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

def is_between_points(a,b,c):
    return distance(a,c) + distance(c,b) == distance(a,b)
def is_on_line(lineForDistance,pointOnLine):
    # print(lineForDistance)
    # print(point)
    return distance(lineForDistance.points[0],pointOnLine) + distance(pointOnLine,lineForDistance.points[1]) == distance(lineForDistance.points[0],lineForDistance.points[1])

def is_on_any_line(guessPoint):
    trueStatements = [is_on_line(i, guessPoint) for i in polygonLines]
    return any(trueStatements)


def plotShape(linesToPlot,  numberOfPolygonLines):
    plt.axis('off')
    xValuesToPlotShape = []
    yValuesToPlotShape = []
    xValuesToPlotPath = []
    yValuesToPlotPath = []
    for p in range(len(linesToPlot)):
        if p < numberOfPolygonLines:
            plt.plot(linesToPlot[p].points[0].x, linesToPlot[p].points[0].y, 'r*')
            plt.plot(linesToPlot[p].points[1].x, linesToPlot[p].points[1].y, 'r*')
            xValuesToPlotShape.append(linesToPlot[p].points[0].x)
            xValuesToPlotShape.append(linesToPlot[p].points[1].x)
            yValuesToPlotShape.append(linesToPlot[p].points[0].y)
            yValuesToPlotShape.append(linesToPlot[p].points[1].y)
        else:
            plt.plot(linesToPlot[p].points[0].x, linesToPlot[p].points[0].y, 'go')
            plt.plot(linesToPlot[p].points[1].x, linesToPlot[p].points[1].y, 'go')
            xValuesToPlotPath.append(linesToPlot[p].points[0].x)
            xValuesToPlotPath.append(linesToPlot[p].points[1].x)
            yValuesToPlotPath.append(linesToPlot[p].points[0].y)
            yValuesToPlotPath.append(linesToPlot[p].points[1].y)

    xValuesToPlotShape.pop()
    yValuesToPlotShape.pop()

    xValuesToPlotPath.pop()
    yValuesToPlotPath.pop()

    plt.plot(xValuesToPlotShape, yValuesToPlotShape, 'b--')
    plt.plot(xValuesToPlotPath, yValuesToPlotPath, 'g-')

    # plt.plot(xGuessPoints, yGuessPoints, 'c.')
    # plt.plot(xGuessPoints, yGuessPoints, 'm')

    ax = plt.gca()
    ax.set_aspect(1)

    plt.xlabel('Y')
    plt.ylabel('X')
    plt.show()


def FindAxisLimits(arrayOfPoints):
    minXAxis = 10000
    maxXAxis = -10000
    minYAxis = 10000
    maxYAxis = -10000

    numberOfPointsNotOnEdge = round(len(arrayOfPoints))

    for q in range(numberOfPointsNotOnEdge):
        if arrayOfPoints[q].x > maxXAxis:
            maxXAxis = arrayOfPoints[q].x
        elif arrayOfPoints[q].x < minXAxis:
            minXAxis = arrayOfPoints[q].x
        if arrayOfPoints[q].y > maxYAxis:
            maxYAxis = arrayOfPoints[q].y
        elif arrayOfPoints[q].y < minYAxis:
            minYAxis = arrayOfPoints[q].y

    return minXAxis, maxXAxis, minYAxis, maxYAxis


def GetPointDistanceFromLine(middlePoint, polygonLine):
    p1 = np.array([polygonLine.points[0].x, polygonLine.points[0].y])
    p2 = np.array([polygonLine.points[1].x, polygonLine.points[1].y])
    p3 = np.array([middlePoint.x, middlePoint.y])

    return np.cross(p2 - p1, p3 - p1) / LA.norm(p2 - p1)


# def constraint1(optimalPoints):
#     returnVec = []
#
#     # for distanceIndex in range(len(polygonLines)):
#     #     returnVec.append(GetPointDistanceFromLine(currPoint5, polygonLines[distanceIndex])-minDistanceFromLine)
#
#     returnVec.append(GetPointDistanceFromLine(point(optimalPoints[0], optimalPoints[1]), polygonLines[2])
#                      - minDistanceFromLine)
#
#     return returnVec
#
#
# def constraint2(optimalPoints):
#     point6ThatShouldBeOnLine = point(optimalPoints[2], optimalPoints[3])
#     return distance(line1.points[0], point6ThatShouldBeOnLine) + distance(point6ThatShouldBeOnLine, line1.points[1]) \
#         - distance(line1.points[0], line1.points[1])
#
#
# def constraint3(optimalPoints):
#     point7ThatShouldBeOnLine = point(optimalPoints[4], optimalPoints[5])
#     return distance(line2.points[0], point7ThatShouldBeOnLine) + distance(point7ThatShouldBeOnLine, line2.points[1]) \
#         - distance(line2.points[0], line2.points[1])
#
#
# def constraint4(optimalPoints):
#     point8ThatShouldBeOnLine = point(optimalPoints[6], optimalPoints[7])
#     return distance(line3.points[0], point8ThatShouldBeOnLine) + distance(point8ThatShouldBeOnLine, line3.points[1]) \
#         - distance(line3.points[0], line3.points[1])
#
#
# def constraint5(optimalPoints):
#     point9ThatShouldBeOnLine = point(optimalPoints[8], optimalPoints[9])
#     return distance(line4.points[0], point9ThatShouldBeOnLine) + distance(point9ThatShouldBeOnLine, line4.points[1]) \
#         - distance(line4.points[0], line4.points[1])
#

def constraint6(optimalPoints):
    distanceBetweenPoints = []
    currentPoint5 = point(optimalPoints[0], optimalPoints[1])
    currentPoint6 = point(optimalPoints[2], optimalPoints[3])
    currentPoint7 = point(optimalPoints[4], optimalPoints[5])
    currentPoint8 = point(optimalPoints[6], optimalPoints[7])
    currentPoint9 = point(optimalPoints[8], optimalPoints[9])

    distanceBetweenPoints.append(distance(currentPoint6, currentPoint7) - minDistanceBetweenPathNodes)
    distanceBetweenPoints.append(distance(currentPoint7, currentPoint8) - minDistanceBetweenPathNodes)
    distanceBetweenPoints.append(distance(currentPoint8, currentPoint9) - minDistanceBetweenPathNodes)
    distanceBetweenPoints.append(distance(currentPoint9, currentPoint6) - minDistanceBetweenPathNodes)

    distanceBetweenPoints.append(distance(currentPoint6, currentPoint5) - minDistanceBetweenPathNodes)
    distanceBetweenPoints.append(distance(currentPoint7, currentPoint5) - minDistanceBetweenPathNodes)
    distanceBetweenPoints.append(distance(currentPoint8, currentPoint5) - minDistanceBetweenPathNodes)
    distanceBetweenPoints.append(distance(currentPoint9, currentPoint5) - minDistanceBetweenPathNodes)

    return distanceBetweenPoints
#
# def constraint7(optimalPoints):
#     returnVec = []
#
#     minXCon, maxXCon, minYCon, maxYCon = FindAxisLimits([point1, point2, point3, point4])
#
#     returnVec.append(optimalPoints[0] - minXCon)
#     returnVec.append(maxXCon - optimalPoints[0])
#     returnVec.append(optimalPoints[1] - minYCon)
#     returnVec.append(maxYCon - optimalPoints[1])
#
#     return returnVec
#
#
# def constraint8(optimalPoints):
#     yOfLine2 = line2.points[1].y + (((line2.points[0].y - line2.points[1].y)/(line2.points[0].x - line2.points[1].x))
#                                     *(optimalPoints[0] - line2.points[1].x))
#
#     return yOfLine2 - optimalPoints[1]
#
#
# def constraint9(optimalPoints):
#     yOfLine4 = line4.points[1].y + (((line4.points[0].y - line4.points[1].y)/(line4.points[0].x - line4.points[1].x))
#                                     *(optimalPoints[0] - line4.points[1].x))
#
#     return optimalPoints[1] - yOfLine4
#
#
# def constraint10(optimalPoints):
#     xOfLine1 = line1.points[1].x + (((line1.points[1].x - line1.points[0].x) / (line1.points[1].y - line1.points[0].y))
#                                     * (optimalPoints[1] - line1.points[1].y))
#
#     return optimalPoints[0] - xOfLine1
#
#
# def constraint11(optimalPoints):
#     xOfLine3 = line3.points[1].x + (((line3.points[1].x - line3.points[0].x) / (line3.points[1].y - line3.points[0].y))
#                                     * (optimalPoints[1] - line3.points[1].y))
#
#     # print("point5 is to the left of", xOfLine3 - optimalPoints[0])
#     return xOfLine3 - optimalPoints[0]
#

def PointIsBoundedInPolygonConstraint(optimalPoints):
    returnVec = []
    yOfLine2 = line2.points[1].y + (((line2.points[0].y - line2.points[1].y) / (line2.points[0].x - line2.points[1].x))
                                    * (optimalPoints[0] - line2.points[1].x))

    yOfLine4 = line4.points[1].y + (((line4.points[0].y - line4.points[1].y) / (line4.points[0].x - line4.points[1].x))
                                    * (optimalPoints[0] - line4.points[1].x))

    xOfLine1 = line1.points[1].x + (((line1.points[1].x - line1.points[0].x) / (line1.points[1].y - line1.points[0].y))
                                    * (optimalPoints[1] - line1.points[1].y))

    xOfLine3 = line3.points[1].x + (((line3.points[1].x - line3.points[0].x) / (line3.points[1].y - line3.points[0].y))
                                    * (optimalPoints[1] - line3.points[1].y))

    returnVec.append(yOfLine2 - optimalPoints[1])
    returnVec.append(optimalPoints[1] - yOfLine4)
    returnVec.append(optimalPoints[0] - xOfLine1)
    returnVec.append(xOfLine3 - optimalPoints[0])

    return returnVec

def PathStartPointsFallOnLines(optimalPoints):
    returnVec = []

    point6ThatShouldBeOnLine = point(optimalPoints[2], optimalPoints[3])
    returnVec.append(distance(line1.points[0], point6ThatShouldBeOnLine) + distance(point6ThatShouldBeOnLine, line1.points[1]) \
            - distance(line1.points[0], line1.points[1]))

    point7ThatShouldBeOnLine = point(optimalPoints[4], optimalPoints[5])
    returnVec.append(distance(line2.points[0], point7ThatShouldBeOnLine) + distance(point7ThatShouldBeOnLine, line2.points[1]) \
            - distance(line2.points[0], line2.points[1]))

    point8ThatShouldBeOnLine = point(optimalPoints[6], optimalPoints[7])
    returnVec.append(distance(line3.points[0], point8ThatShouldBeOnLine) + distance(point8ThatShouldBeOnLine, line3.points[1]) \
            - distance(line3.points[0], line3.points[1]))

    point9ThatShouldBeOnLine = point(optimalPoints[8], optimalPoints[9])
    returnVec.append(distance(line4.points[0], point9ThatShouldBeOnLine) + distance(point9ThatShouldBeOnLine, line4.points[1]) \
            - distance(line4.points[0], line4.points[1]))

    return returnVec


# con1 = {'type': 'ineq', 'fun': constraint1}  # Point is a minimum distance from line
# con2 = {'type': 'eq', 'fun': constraint2}  # 2-5, starts on lines
# con3 = {'type': 'eq', 'fun': constraint3}
# con4 = {'type': 'eq', 'fun': constraint4}
# con5 = {'type': 'eq', 'fun': constraint5}
con6 = {'type': 'ineq', 'fun': constraint6}  # Minimum distance between path nodes, not on top of each other
# con7 = {'type': 'ineq', 'fun': constraint7}  # Point is within bounding limits, replaced with polygon bounds
# con8 = {'type': 'ineq', 'fun': constraint8}  # 8-11, is within polygon
# con9 = {'type': 'ineq', 'fun': constraint9}
# con10 = {'type': 'ineq', 'fun': constraint10}
# con11 = {'type': 'ineq', 'fun': constraint11}
con12 = {'type': 'ineq', 'fun': PointIsBoundedInPolygonConstraint}
con13 = {'type': 'eq', 'fun': PathStartPointsFallOnLines}

cons = [con6, con12, con13]


def functionToMinimize(optimalPoints):
    point5New = point(optimalPoints[0], optimalPoints[1])
    point6New = point(optimalPoints[2], optimalPoints[3])
    point7New = point(optimalPoints[4], optimalPoints[5])
    point8New = point(optimalPoints[6], optimalPoints[7])
    point9New = point(optimalPoints[8], optimalPoints[9])
    xGuessPoints.append(optimalPoints[0])
    yGuessPoints.append(optimalPoints[1])

    line5Opt = line(point6New, point5New)
    line6Opt = line(point7New, point5New)
    line7Opt = line(point8New, point5New)
    line8Opt = line(point9New, point5New)
    pathLinesNew = [line5Opt, line6Opt, line7Opt, line8Opt]

    return GetMassOfAllLines(pathLinesNew)


# ##DEFINE PANEL POLYGON## #
point1 = point(0, 0)
point2 = point(0, 1)
point3 = point(3, 0.75)
point4 = point(2, 0.25)

minX, maxX, minY, maxY = FindAxisLimits([point1, point2, point3, point4])

line1 = line(point1, point2)
line2 = line(point2, point3)
line3 = line(point3, point4)
line4 = line(point1, point4)

polygonLines = [line1, line2, line3, line4]


# point5InitialGuess = [(0.25, 0.375)]
# point6InitialGuess = [(0, 0.375)]
# point7InitialGuess = [(0.375, 1)]
# point8InitialGuess = [(1, 0.375)]
# point9InitialGuess = [(0.75, 0)]

point5InitialGuess = [((minX+maxX)/2, (minY+maxY)/2)]
point6InitialGuess = [(0, 0.5)]
point7InitialGuess = [(0.5, 1)]
point8InitialGuess = [(1, 0.5)]
point9InitialGuess = [(0.5, 0)]

initialPointsGuesses = [point5InitialGuess, point6InitialGuess, point7InitialGuess, point8InitialGuess, point9InitialGuess]


# minDistance = 0.1
minDistanceBetweenPathNodes = 0.5
# minDistanceFromLine = 0.25
xGuessPoints = []
yGuessPoints = []


opt = {'maxiter': 1000}
result = minimize(functionToMinimize, initialPointsGuesses, constraints=cons, options=opt)
print(result)

point5 = point(result.x[0], result.x[1])
point6 = point(result.x[2], result.x[3])
point7 = point(result.x[4], result.x[5])
point8 = point(result.x[6], result.x[7])
point9 = point(result.x[8], result.x[9])

# print("Point 5:", point5)
# print("Point 6:", point6)
# print("Point 7:", point7)
# print("Point 8:", point8)
# print("Point 9:", point9)

# for w in range(len(polygonLines)):
#     print(GetPointDistanceFromLine(point5, polygonLines[w]))

# xOfLine3 = line3.points[1].x + (((line3.points[1].x - line3.points[0].x)/(line3.points[1].y - line3.points[0].y))*(point5.y - line3.points[1].y))

line5 = line(point6, point5)
line6 = line(point7, point5)
line7 = line(point8, point5)
line8 = line(point9, point5)

pathLines = [line5, line6, line7, line8]

plotLines = polygonLines.copy()
plotLines.extend(pathLines)

PrintMassOfAllLines(pathLines)
plotShape(plotLines, len(polygonLines))




# print("point 5 is between:", is_between_points(point1, point2,point5), ". This should be False.")
# print("point 1 is between:", is_between_points(point1, point4, point1), ". This should be True.")
# print("Point 1 is on line 1:", is_on_line(line1, point1), ". This should be True.")
# print("Point 3 is on line 1:", is_on_line(line1, point3), ". This should be False.")
# print("Point 5 is on line 4:", is_on_line(line4, point5), ". This should be False.")
# print("Point 6 is on line 4:", is_on_line(line4, point6), ". This should be True.")

# print("Point 1 is on any line", is_on_any_line(point1), ". This should be True.")
# print("Point 2 is on any line", is_on_any_line(point2), ". This should be True.")
# print("Point 3 is on any line", is_on_any_line(point3), ". This should be True.")
# print("Point 4 is on any line", is_on_any_line(point4), ". This should be True.")
# print("Point 5 is on any line", is_on_any_line(point5), ". This should be False.")
# print("Point 6 is on any line", is_on_any_line(point6), ". This should be True.")
