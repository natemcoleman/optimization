from math import sqrt, pi
import numpy as np
from numpy import linalg as la


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
        rho = 1115  # kg/m3
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


def GetPropertiesOfSections(crossSectionShape, baseLength, baseHeight, diameter):
    if crossSectionShape == "Round":
        A = (pi * (diameter ** 2)) / 4
        Ix = (pi * (diameter ** 4)) / 64
        Iy = Ix
    elif crossSectionShape == "Rectangle":
        A = baseLength * baseHeight
        Ix = (baseLength * (baseHeight ** 3)) / 12
        Iy = (baseHeight * (baseLength ** 3)) / 12
    elif crossSectionShape == "Square":
        A = baseLength ** 2
        Ix = (baseLength * (baseLength ** 3)) / 12
        Iy = (baseLength * (baseLength ** 3)) / 12
    # elif crossSectionShape == "I-Beam":
    # A, Ix, Iy = OptimizeIBeam()
    else:
        A = -9999
        Ix = -9999
        Iy = -9999
        print("Invalid cross-sectional shape!")

    # print("A =", A)
    # print("Ix =", Ix)
    # print("Iy =", Iy)

    return A, Ix, Iy


def GetLengthOfLine(lineToGetLength):
    return sqrt(distance(lineToGetLength.points[0], lineToGetLength.points[1]))


def GetMassOfLine(lineToCalcMass, A, rho):
    massOfLine = GetLengthOfLine(lineToCalcMass) * A * rho  # mass in kilograms

    return massOfLine


def GetMassOfAllLines(linesToGetMass, A, rho):
    totalMass = 0
    for m in range(len(linesToGetMass)):
        totalMass += GetMassOfLine(linesToGetMass[m], A, rho)

    return totalMass


def PrintMassOfAllLines(linesToGetMass, A, rho):
    totalMass = 0
    for m in range(len(linesToGetMass)):
        totalMass += GetMassOfLine(linesToGetMass[m], A, rho)
        print("Mass of line ", m, ":", GetMassOfLine(linesToGetMass[m], A, rho), "kg")
    print("Total mass:", totalMass, "kg")


def TempStiffnessCalc(allLines, E, Ix, Iy):
    stiffness = 0
    kx = 0
    ky = 0
    gamma = 0.8517
    xLengths = []
    yLengths = []

    for stiffnessIndex in range(len(allLines)):
        xLengths.append(abs(allLines[stiffnessIndex].points[0].x - allLines[stiffnessIndex].points[1].x))
        yLengths.append(abs(allLines[stiffnessIndex].points[0].y - allLines[stiffnessIndex].points[1].y))

    for stiffnessIndex in range(len(xLengths)):
        kx += (pi * (gamma ** 2) * E * Ix) / (xLengths[stiffnessIndex])
        ky += (pi * (gamma ** 2) * E * Iy) / (yLengths[stiffnessIndex])

    stiffness = kx
    stiffness -= ky

    return stiffness


def distance(a, b):
    return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def distanceTuple(a, b):
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def is_between_points(a, b, c):
    return distance(a, c) + distance(c, b) == distance(a, b)


def is_on_line(lineForDistance, pointOnLine):
    return distance(lineForDistance.points[0], pointOnLine) + distance(pointOnLine,
                                                                       lineForDistance.points[1]) == distance(
        lineForDistance.points[0], lineForDistance.points[1])


def is_on_any_line(guessPoint, polygonLinesCon):
    trueStatements = [is_on_line(i, guessPoint) for i in polygonLinesCon]
    return any(trueStatements)


def FindAxisLimits(arrayOfPoints):
    minXAxis = 10000
    maxXAxis = -10000
    minYAxis = 10000
    maxYAxis = -10000

    numberOfPointsNotOnEdge = round(len(arrayOfPoints))

    for q in range(numberOfPointsNotOnEdge):
        if arrayOfPoints[q].x > maxXAxis:
            maxXAxis = arrayOfPoints[q].x
        if arrayOfPoints[q].x < minXAxis:
            minXAxis = arrayOfPoints[q].x
        if arrayOfPoints[q].y > maxYAxis:
            maxYAxis = arrayOfPoints[q].y
        if arrayOfPoints[q].y < minYAxis:
            minYAxis = arrayOfPoints[q].y

    return minXAxis, maxXAxis, minYAxis, maxYAxis


def FindAxisLimitsOfLines(arrayOfLines):
    minXAxis = 10000
    maxXAxis = -10000
    minYAxis = 10000
    maxYAxis = -10000

    minXAxis = float(minXAxis)
    maxXAxis = float(maxXAxis)
    minYAxis = float(minYAxis)
    maxYAxis = float(maxYAxis)

    xPoints = []
    yPoints = []

    for q in range(len(arrayOfLines)):
        xPoints.append(arrayOfLines[q].points[0].x)
        xPoints.append(arrayOfLines[q].points[1].x)
        yPoints.append(arrayOfLines[q].points[0].y)
        yPoints.append(arrayOfLines[q].points[1].y)

    numberOfPointsNotOnEdge = round(len(xPoints))

    for q in range(numberOfPointsNotOnEdge):
        if xPoints[q] > maxXAxis:
            maxXAxis = xPoints[q]
        if xPoints[q] < minXAxis:
            minXAxis = xPoints[q]
        if yPoints[q] > maxYAxis:
            maxYAxis = yPoints[q]
        if yPoints[q] < minYAxis:
            minYAxis = yPoints[q]

    return minXAxis, maxXAxis, minYAxis, maxYAxis


def GetMidpointOfLine(currLine):
    return [(((currLine.points[0].x + currLine.points[1].x) / 2), ((currLine.points[0].y + currLine.points[1].y) / 2))]


def GetPointDistanceFromLine(middlePoint, polygonLine):
    p1 = np.array([polygonLine.points[0].x, polygonLine.points[0].y])
    p2 = np.array([polygonLine.points[1].x, polygonLine.points[1].y])
    p3 = np.array([middlePoint.x, middlePoint.y])

    return np.cross(p2 - p1, p3 - p1) / la.norm(p2 - p1)


def CalculateStiffnessOfAllPanels(panels):
    totalStiffness = 0

    for i in range(len(panels)):
        totalStiffness += CalculateStiffnessOfPanel(panels[i])

    return totalStiffness


def intersectionOfLines(line1, line2):
    """
    Find the intersection point of two lines.

    Each line is defined by two tuples that represent the x,y coordinates of
    its endpoints.

    Returns the intersection point if one exists, or None if the lines are parallel.
    """
    # Unpack the coordinates of the endpoints of the first line
    x1, y1 = line1[0]
    x2, y2 = line1[1]

    # Unpack the coordinates of the endpoints of the second line
    x3, y3 = line2[0]
    x4, y4 = line2[1]

    # Calculate the slopes and y-intercepts of the two lines
    slope1 = (y2 - y1) / (x2 - x1) if x2 - x1 != 0 else float('inf')
    slope2 = (y4 - y3) / (x4 - x3) if x4 - x3 != 0 else float('inf')
    y_int1 = y1 - slope1 * x1 if slope1 != float('inf') else None
    y_int2 = y3 - slope2 * x3 if slope2 != float('inf') else None

    # Check if the lines are parallel
    if slope1 == slope2:
        return None

    # Calculate the x-coordinate of the intersection point
    if slope1 == float('inf'):
        x_int = x1
    elif slope2 == float('inf'):
        x_int = x3
    else:
        x_int = (y_int2 - y_int1) / (slope1 - slope2)

    # Check if the intersection point is within the range of the first line
    if x_int < min(x1, x2) or x_int > max(x1, x2):
        return None

    # Check if the intersection point is within the range of the second line
    if x_int < min(x3, x4) or x_int > max(x3, x4):
        return None

    # Check if the intersection point is within the range of the second line
    if x_int < min(x3, x4) or x_int > max(x3, x4):
        return None

    # Calculate the y-coordinate of the intersection point
    if slope1 == float('inf'):
        y_int = slope2 * x_int + y_int2
    else:
        y_int = slope1 * x_int + y_int1

    # Check if the intersection point is the same as any of the line endpoints
    distanceBuffer = 1e-8
    if distanceTuple((x_int, y_int), line1[0]) <= distanceBuffer or distanceTuple((x_int, y_int), line1[1]) <= \
            distanceBuffer or distanceTuple((x_int, y_int), line2[0]) <= distanceBuffer or distanceTuple((x_int, y_int)
        , line2[1]) <= distanceBuffer:
        return None

    # Return the intersection point as a tuple
    return (x_int, y_int)


def GetPanelBisectionAndPathLineIntersectionPointsForAPanel(listOfPanelCorners, pathlines):
    bisectionLine = ((listOfPanelCorners[0].x, listOfPanelCorners[0].y), (listOfPanelCorners[2].x, listOfPanelCorners[2].y))
    nonBisectionLine = ((listOfPanelCorners[1].x, listOfPanelCorners[1].y), (listOfPanelCorners[3].x, listOfPanelCorners[3].y))

    bisectionIntersectionPoints = []
    nonBisectionIntersectionPoints = []

    for i in range(len(pathlines)):
        intersectionPointBisection = intersectionOfLines(bisectionLine, ((pathlines[i].points[0].x, pathlines[i].points[0].y), (pathlines[i].points[1].x, pathlines[i].points[1].y)))
        intersectionPointNonBisection = intersectionOfLines(nonBisectionLine, ((pathlines[i].points[0].x, pathlines[i].points[0].y), (pathlines[i].points[1].x, pathlines[i].points[1].y)))

        if intersectionPointBisection is not None:
            bisectionIntersectionPoints.append(intersectionPointBisection)
        if intersectionPointNonBisection is not None:
            nonBisectionIntersectionPoints.append(intersectionPointNonBisection)

    return bisectionIntersectionPoints, nonBisectionIntersectionPoints


def CalculateStiffnessOfPanel(pathLinesNew, listOfCornerPoints, bMp=None, nbMp=None):
    # path lines are the green lines in the output that connect the black points. Corner points are the red points
    # that are the corners of each panel

    # define moment vectors
    bMp[0] = [(listOfCornerPoints.points[2].x - listOfCornerPoints.points[0].x),
            (listOfCornerPoints.points[2].y - listOfCornerPoints.points[0].y)]
    nbMp[0] = [(listOfCornerPoints.points[1].x - listOfCornerPoints.points[3].x),
            (listOfCornerPoints.points[1].y - listOfCornerPoints.points[3].y)]

    bMp[1] = [(listOfCornerPoints.points[4].x - listOfCornerPoints.points[3].x),
            (listOfCornerPoints.points[4].y - listOfCornerPoints.points[3].y)]
    nbMp[1] = [(listOfCornerPoints.points[0].x - listOfCornerPoints.points[6].x),
            (listOfCornerPoints.points[0].y - listOfCornerPoints.points[6].y)]

    bMp[2] = [(listOfCornerPoints.points[6].x - listOfCornerPoints.points[5].x),
            (listOfCornerPoints.points[6].y - listOfCornerPoints.points[5].y)]
    nbMp[2] = [(listOfCornerPoints.points[4].x - listOfCornerPoints.points[7].x),
            (listOfCornerPoints.points[4].y - listOfCornerPoints.points[7].y)]

    bMp[3] = [(listOfCornerPoints.points[8].x - listOfCornerPoints.points[7].x),
            (listOfCornerPoints.points[8].y - listOfCornerPoints.points[7].y)]
    nbMp[3] = [(listOfCornerPoints.points[6].x - listOfCornerPoints.points[9].x),
            (listOfCornerPoints.points[6].y - listOfCornerPoints.points[9].y)]

    bMp[4] = [(listOfCornerPoints.points[3].x - listOfCornerPoints.points[12].x),
            (listOfCornerPoints.points[3].y - listOfCornerPoints.points[12].y)]
    nbMp[4] = [(listOfCornerPoints.points[2].x - listOfCornerPoints.points[13].x),
            (listOfCornerPoints.points[2].y - listOfCornerPoints.points[13].y)]

    bMp[5] = [(listOfCornerPoints.points[5].x - listOfCornerPoints.points[13].x),
            (listOfCornerPoints.points[5].y - listOfCornerPoints.points[13].y)]
    nbMp[5] = [(listOfCornerPoints.points[3].x - listOfCornerPoints.points[14].x),
            (listOfCornerPoints.points[3].y - listOfCornerPoints.points[14].y)]

    # Note that the bisection lines switch directions
    bMp[6] = [(listOfCornerPoints.points[13].x - listOfCornerPoints.points[16].x),
            (listOfCornerPoints.points[13].y - listOfCornerPoints.points[16].y)]
    nbMp[6] = [(listOfCornerPoints.points[14].x - listOfCornerPoints.points[15].x),
            (listOfCornerPoints.points[14].y - listOfCornerPoints.points[15].y)]

    bMp[7] = [(listOfCornerPoints.points[15].x - listOfCornerPoints.points[18].x),
            (listOfCornerPoints.points[15].y - listOfCornerPoints.points[18].y)]
    nbMp[7] = [(listOfCornerPoints.points[16].x - listOfCornerPoints.points[17].x),
            (listOfCornerPoints.points[16].y - listOfCornerPoints.points[17].y)]

    bMp[8] = [(listOfCornerPoints.points[16].x - listOfCornerPoints.points[19].x),
            (listOfCornerPoints.points[16].y - listOfCornerPoints.points[19].y)]
    nbMp[8] = [(listOfCornerPoints.points[7].x - listOfCornerPoints.points[18].x),
            (listOfCornerPoints.points[7].y - listOfCornerPoints.points[18].y)]

    bMp[9] = [(listOfCornerPoints.points[7].x - listOfCornerPoints.points[11].x),
            (listOfCornerPoints.points[7].y - listOfCornerPoints.points[11].y)]
    nbMp[9] = [(listOfCornerPoints.points[9].x - listOfCornerPoints.points[19].x),
            (listOfCornerPoints.points[9].y - listOfCornerPoints.points[19].y)]

    # calculate slopes of moment vectors
    slope_b = np.zeros(len(bMp))
    slope_nb = np.zeros(len(nbMp))
    for i in range(len(bMp)):
        slope_b[i] = bMp[i][1]/bMp[i][0]
        slope_nb[i] = nbMp[i][1]/nbMp[i][0]

    # slopes of beams

    # find intersection points

    # calculate length of beam (L), intersect distance from end of beam (q)

    # calculate np.cross() of bMp and nbMp with pathLinesNew

    # calculate magnitude of vectors np.la.norm?

    # divide results of cross-product by magnitudes to get sin(theta)
    # return angle

    # calculate stiffness
    # NEED E, I

    # kb[i] = 3*E*I*L[i]*angle/(L[i]**2 - 3*L[i]*q[i] + 3*q[i]**2)
    # knb[i] = 3*E*I*L[i]*angle/(L[i]**2 - 3*L[i]*q[i] + 3*q[i]**2)

    # return kb, knb


